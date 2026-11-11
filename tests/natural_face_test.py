"""Tests for the natural face demo (natural_face.py) and the session features
it uses: the temperature stop, --jaw-release and FaceSession.extra."""

from __future__ import annotations

from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace

sys.modules.setdefault("serial", SimpleNamespace(Serial=object))
FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))
sys.path.insert(0, str(FIRMWARE / "tests"))

import eye_tour  # noqa: E402
import face_stream as fs  # noqa: E402
import natural_face as nf  # noqa: E402
import workspace_guard as wg  # noqa: E402
from face_stream_test import Clock  # noqa: E402

# The operator's 2026-11-10 marks (panel 184058/184403).
MARKS = {0: {"neutral": 3208, "end_1": 2990, "end_2": 3476},
         1: {"neutral": 2068, "end_1": 2934, "end_2": 490},
         2: {"neutral": 143, "end_1": -2, "end_2": 424},
         5: {"neutral": 372, "end_1": -1221},
         6: {"neutral": 451, "end_1": 1988}}


def limits(marks=MARKS):
    return eye_tour.tour_limits({i: {k: {"present_tick": v} for k, v in m.items()}
                                 for i, m in marks.items()})


def test_plan_is_repeatable_and_bounded() -> None:
    settings = nf.NaturalSettings(seconds=40.0, seed=3)
    a, b = nf.NaturalPlan.make(settings), nf.NaturalPlan.make(settings)
    assert a.syllables == b.syllables and a.blinks == b.blinks and a.saccades == b.saccades
    assert a.syllables and a.blinks and len(a.saccades) > 10
    for t in [x / 50 for x in range(0, 40 * 50)]:
        actions, j = a.face(t)
        assert 0.0 <= j <= 1.0
        d, v = actions["right"]
        assert 0.0 <= d <= settings.smile_diag + 1e-9 and 0.0 <= v <= settings.smile_up + 1e-9
    assert a.face(40.5) is None
    assert a.face(0.5)[1] == 0.0 and a.face(0.5)[0]["right"] == (0.0, 0.0)   # rest first
    assert all(start >= settings.smile_in_s + settings.ramp_s for start, _l, _a in a.syllables)
    assert all(start + length <= 40.0 - settings.ramp_s for start, length, _a in a.syllables)
    no_talk = nf.NaturalPlan.make(nf.NaturalSettings(seconds=20.0, talk=False))
    assert no_talk.syllables == [] and all(no_talk.face(t / 10)[1] == 0.0 for t in range(200))


def test_eye_targets_stay_inside_the_marks() -> None:
    lim = limits()
    eyes = nf.EyeLids(nf.NaturalPlan.make(nf.NaturalSettings()), lim)
    # Yaw: both eyes move by the same ticks, limited by the smaller span each way.
    right = eyes.eye_targets(1.0, 0.0)
    assert right[0] - 3208 == right[2] - 143 == -145            # min(218, 145) toward end_1
    left = eyes.eye_targets(-1.0, 0.0)
    assert left[0] - 3208 == left[2] - 143 == 268               # min(268, 281) toward end_2
    assert eyes.eye_targets(0.0, 1.0)[1] == 490 and eyes.eye_targets(0.0, -1.0)[1] == 2934
    for x in (-1.0, -0.3, 0.0, 0.4, 1.0):
        for y in (-1.0, 0.0, 1.0):
            for i, tick in eyes.eye_targets(x, y).items():
                assert lim[i]["low"] <= tick <= lim[i]["high"], (i, tick)
    assert eyes.lid_targets(1.0) == {5: -1221, 6: 1988} and eyes.lid_targets(0.0) == {5: 372, 6: 451}
    bad = dict(MARKS)
    bad[2] = {"neutral": 143, "end_1": -2, "end_2": -50}           # both ends on one side
    try:
        nf.EyeLids(nf.NaturalPlan.make(nf.NaturalSettings()), limits(bad))
        raise AssertionError("same-side ends accepted")
    except ValueError:
        pass


def session(seconds=14.0, temperatures=None, release=None):
    clock = Clock()
    board = fs.face_preview_controller(clock)
    if temperatures:
        board.temperatures = dict(temperatures)
    marks = eye_tour.preview_marks(board)
    plan = nf.NaturalPlan.make(nf.NaturalSettings(seconds=seconds, seed=2))
    extra = nf.EyeLids(plan, eye_tour.tour_limits(marks))
    neutrals = fs.preview_neutrals()
    if release:
        neutrals = fs.with_jaw_release(neutrals, release)
    link = fs.StreamLink(board, clock)
    face = fs.FaceSession(link, neutrals, Path(tempfile.mkdtemp()), 1000.0, 1500.0,
                          say=lambda _text: None, extra=extra)
    return clock, board, face, plan, extra


def test_natural_session_runs_on_the_simulated_board() -> None:
    clock, board, face, plan, extra = session()
    sent = []
    face.on_command = lambda _t, _a, _j, targets: sent.append(dict(targets))
    face.start()
    assert face.run(plan.face, sleep=lambda _s: clock.advance(0.005)) == "finished"
    lim = extra.lim
    for targets in sent:
        for i in nf.EXTRA_SERVOS:
            assert lim[i]["low"] <= targets[i] <= lim[i]["high"], (i, targets[i])
    # The blinks closed the lids, and fast: the lids' span at lid speed.
    assert extra.blink_log and all("total_s" in b for b in extra.blink_log), extra.blink_log
    span = max(abs(lim[i]["ends"][0] - lim[i]["neutral"]) for i in nf.LIDS)
    assert all(b["closed_s"] <= span / nf.LID_SPEED_TPS + nf.LID_SPEED_TPS / nf.LID_ACCEL_TPS2
               + nf.BLINK_CLOSE_SLACK_S + 0.05 for b in extra.blink_log), extra.blink_log
    assert extra.peak_ma and set(extra.peak_ma) == set(nf.EXTRA_SERVOS)
    assert extra.saccade_log
    # At the end the eyes are back at neutral and the lids open.
    for i in nf.EXTRA_SERVOS:
        assert abs(board.goal[i] - lim[i]["neutral"]) <= 1, (i, board.goal[i])
    # The envelope covered the eyes and lids at their speed.
    profile = fs.headctl.read_profile(board)
    slot = {s["index"]: s for s in profile["joints"]}
    assert slot[5]["min_tick"] <= lim[5]["low"] and slot[5]["max_tick"] >= lim[5]["high"]
    assert slot[5]["max_position_fraction_per_control_cycle"] * (
        slot[5]["max_tick"] - slot[5]["min_tick"]) >= nf.LID_SPEED_TPS * 0.002
    assert face.log["max_temperature_c"]


def test_temperature_stop() -> None:
    clock, board, face, plan, _extra = session(temperatures={7: fs.TEMP_STOP_C})
    face.start()
    reason = face.run(plan.face, sleep=lambda _s: clock.advance(0.005))
    assert reason.startswith("temperature stop") and "7 at 52 C" in reason, reason
    clock, board, face, plan, _extra = session(seconds=10.0, temperatures={5: fs.TEMP_STOP_C - 1})
    face.start()
    assert face.run(plan.face, sleep=lambda _s: clock.advance(0.005)) == "finished"
    assert face.log["max_temperature_c"]["5"] == fs.TEMP_STOP_C - 1


def test_jaw_release_pays_the_down_lines_out_with_the_jaw() -> None:
    neutrals = fs.with_jaw_release(fs.preview_neutrals(), {"down": 60})
    guard = fs.guard_from_neutrals(neutrals)
    for index in (8, 9):
        closed = guard.neutral_at(index, 0.0)
        opened = guard.neutral_at(index, 1.0)
        assert wg.PULL[index] * (opened - closed) == -60            # toward slack
        assert wg.PULL[index] * (guard.neutral_at(index, 0.5) - closed) == -30
    for index in (11, 14, 15, 19):
        assert guard.neutral_at(index, 1.0) == guard.neutral_at(index, 0.0)
    # Per line: a line's own value wins over its role's.
    per_line = fs.guard_from_neutrals(fs.with_jaw_release(fs.preview_neutrals(), {"down": 60, "9": 25}))
    assert wg.PULL[8] * (per_line.neutral_at(8, 1.0) - per_line.neutral_at(8, 0.0)) == -60
    assert wg.PULL[9] * (per_line.neutral_at(9, 1.0) - per_line.neutral_at(9, 0.0)) == -25
    # In a smile, down follows and is never pulled past its neutral at the
    # present jaw opening: with the jaw open it is paid out by the release.
    clock, board, face, plan, _extra = session(seconds=14.0, release={"down": 60})
    face.start()
    worst = {8: 0, 9: 0}

    def watch(_t, _a, j, targets):
        line_j = face.controller.guard.jaw_fraction(board.present(wg.JAW))
        for index in worst:
            past = wg.PULL[index] * (targets[index] - face.controller.guard.neutral_at(index, line_j))
            worst[index] = max(worst[index], past)
    face.on_command = watch
    assert face.run(plan.face, sleep=lambda _s: clock.advance(0.005)) == "finished"
    assert all(v <= wg.FOLLOW_MAX_STEP_TICKS + 5 for v in worst.values()), worst


def test_smooth_step_limits_speed_and_acceleration() -> None:
    p, v, dt = 0.0, 0.0, 0.02
    path = []
    for _ in range(200):
        q, w = nf.smooth_step(p, v, 1900.0, 4000.0, 40000.0, dt)
        # The last step lands on the target from at most one step of speed.
        limit = 40000.0 * dt * (2.0 if q == 1900.0 else 1.0) + 1e-6
        assert abs(w) <= 4000.0 + 1e-6 and abs(w - v) <= limit, (v, w)
        assert q <= 1900.0
        path.append(q)
        p, v = q, w
    assert p == 1900.0 and v == 0.0
    # Close time near the trapezoid's d/v + v/a (0.475 + 0.1 s).
    reached = next(n for n, q in enumerate(path) if q == 1900.0) * dt
    assert 0.5 <= reached <= 0.65, reached
    q, w = nf.smooth_step(10.0, -500.0, 0.0, 4000.0, 40000.0, dt)   # never past the target
    assert q >= 0.0


def test_eye_tour_temperature_stop() -> None:
    """head-eyes stops at TEMP_STOP_C too (operator rule 2026-11-10)."""
    clock = Clock()
    board = fs.face_preview_controller(clock)
    board.temperatures = {6: fs.TEMP_STOP_C}
    link = fs.StreamLink(board, clock)
    run = Path(tempfile.mkdtemp())
    tour = eye_tour.EyeTour(link, eye_tour.tour_limits(eye_tour.preview_marks(board)), run, 2500.0,
                            280, run / "STOP", None, lambda _text: None)
    tour.start()
    reason = tour.check()
    tour.finish(reason or "test")
    assert reason is not None and reason.startswith("temperature stop") and "6 at 52 C" in reason, reason


def test_lid_ladder_and_blink_depth() -> None:
    ladder = nf.NaturalPlan.make(nf.NaturalSettings(seconds=14.0, talk=False, smile_up=0.0,
                                                    ladder=(0.3, 0.6, 1.0)))
    assert ladder.blinks == [] and ladder.syllables == []
    assert ladder.ladder_fraction(0.2) == 0.0 and ladder.ladder_fraction(0.6) == 0.3
    assert ladder.ladder_fraction(0.5 + 1.5 * 2 + 0.1) == 1.0 and ladder.ladder_fraction(13.0) == 0.0
    assert [label for _t, label in ladder.marks()] == ["lid_030", "lid_060", "lid_100"]
    # A shallow, fast-closing, slower-opening blink on the simulated board.
    clock = Clock()
    board = fs.face_preview_controller(clock)
    plan = nf.NaturalPlan.make(nf.NaturalSettings(seconds=12.0, seed=2, blink_depth={5: 0.6, 6: 0.6}))
    lim = eye_tour.tour_limits(eye_tour.preview_marks(board))
    extra = nf.EyeLids(plan, lim, lid_speed_tps=5000.0, lid_open_speed_tps=2000.0)
    face = fs.FaceSession(fs.StreamLink(board, clock), fs.preview_neutrals(), Path(tempfile.mkdtemp()),
                          1000.0, 1500.0, say=lambda _text: None, extra=extra)
    deepest = {5: 0.0, 6: 0.0}

    def watch(_t, _a, _j, targets):
        for i in nf.LIDS:
            span = lim[i]["ends"][0] - lim[i]["neutral"]
            deepest[i] = max(deepest[i], (targets[i] - lim[i]["neutral"]) / span)
    face.on_command = watch
    face.start()
    assert face.run(plan.face, sleep=lambda _s: clock.advance(0.005)) == "finished"
    assert all(0.55 <= v <= 0.61 for v in deepest.values()), deepest     # never past the depth
    assert extra.blink_log and all("total_s" in b for b in extra.blink_log)


def test_enable_retries_once_after_a_bus_fault_with_torque_off() -> None:
    """2026-11-10: 4 of ~30 Enables faulted BUS in the preparation, torque
    verified off. With retry on: clear it, enable again, once only."""
    for faults, retry, ok in ((1, True, True), (2, True, False), (1, False, False)):
        clock = Clock()
        board = fs.face_preview_controller(clock)
        board.enable_bus_faults = faults
        face = fs.FaceSession(fs.StreamLink(board, clock), fs.preview_neutrals(), Path(tempfile.mkdtemp()),
                              say=lambda _text: None, retry_enable_bus_fault=retry)
        try:
            face.start()
            started = True
        except RuntimeError as error:
            started = False
            assert "BUS" in str(error), error
        assert started == ok, (faults, retry)
        if ok:
            assert face.log["enable_retries"] == 1
            face.finish("test")


def test_explore_covers_the_action_space_on_the_simulated_board() -> None:
    plan = nf.ExplorePlan.make(nf.NaturalSettings(seconds=30.0, seed=4))
    seen = {k: [] for k in nf.EXPLORE_KEYS}
    for n in range(0, 3000):
        action = plan.action(n / 100)
        for k in nf.EXPLORE_KEYS:
            seen[k].append(action[k])
    vs = [v for _d, v in seen["right"]]
    assert min(vs) < -0.5 and max(vs) > 0.5                  # down and up both driven
    assert max(seen["jaw"]) > 0.7 and max(x for x, _y in seen["gaze"]) > 0.5
    assert plan.face(31.0) is None and plan.action(29.9)["jaw"] == 0.0   # ends at rest
    clock = Clock()
    board = fs.face_preview_controller(clock)
    extra = nf.EyeLids(plan, eye_tour.tour_limits(eye_tour.preview_marks(board)))
    face = fs.FaceSession(fs.StreamLink(board, clock), fs.preview_neutrals(), Path(tempfile.mkdtemp()),
                          1000.0, 4000.0, say=lambda _text: None, extra=extra,
                          upper_ranges=dict(fs.UPPER_SESSION_RANGES))
    face.start()
    assert face.run(lambda t: plan.face(t), sleep=lambda _s: clock.advance(0.005)) == "finished"
    assert face.controller.clamped == {}


def test_validate_plan_is_stratified_and_holds_each_pose() -> None:
    n = 12
    plan = nf.ValidatePlan.make_validate(n, seed=3)
    assert len(plan.poses) == n and plan.blinks == []
    poses = [w[1] for w in plan.waypoints[1:-1]]
    bins = sorted(int((p["right"][1] + 1.0) / 2.0 * n) for p in poses)
    assert bins == list(range(n))                              # v: one sample per bin
    bins = sorted(int(p["jaw"] * n) for p in poses)
    assert bins == list(range(n))
    for (start, end), pose in zip(plan.poses, poses):
        for t in (start + 0.01, (start + end) / 2, end - 0.01):  # held through the window
            got = plan.action(t)["right"]
            assert all(abs(x - y) < 1e-9 for x, y in zip(got, pose["right"])), (t, got, pose)
    assert plan.face(plan.settings.seconds + 0.1) is None


def main() -> None:
    test_plan_is_repeatable_and_bounded()
    test_eye_targets_stay_inside_the_marks()
    test_smooth_step_limits_speed_and_acceleration()
    test_natural_session_runs_on_the_simulated_board()
    test_temperature_stop()
    test_jaw_release_pays_the_down_lines_out_with_the_jaw()
    test_eye_tour_temperature_stop()
    test_lid_ladder_and_blink_depth()
    test_enable_retries_once_after_a_bus_fault_with_torque_off()
    test_explore_covers_the_action_space_on_the_simulated_board()
    test_validate_plan_is_stratified_and_holds_each_pose()
    print("natural face: all tests passed")


if __name__ == "__main__":
    main()
