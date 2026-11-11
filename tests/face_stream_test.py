"""Regression tests for the ENABLED-mode face session, on the simulated board."""

from __future__ import annotations

from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace

sys.modules.setdefault("serial", SimpleNamespace(Serial=object))
FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))

import face_stream as fs  # noqa: E402
import range_finder  # noqa: E402
import tuning_core  # noqa: E402
import workspace_guard as wg  # noqa: E402


NEUTRAL_FOR_GUARD = {8: 2300, 11: 2700, 14: 3100, 9: 1750, 15: 450, 19: 2750}


class Clock:
    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        # A little time passes on every read, as with a real clock, so wait
        # loops see the simulated board send its next STATE.
        self.now += 0.0001
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def session(board_changes=None, neutrals=None, **kwargs):
    clock = Clock()
    board = fs.face_preview_controller(clock)
    if board_changes:
        board_changes(board)
    link = fs.StreamLink(board, clock)
    face = fs.FaceSession(link, neutrals or fs.preview_neutrals(), Path(tempfile.mkdtemp()),
                          say=lambda _text: None, **kwargs)
    return clock, board, face


def run(clock, face, frames, extra_s=0.0):
    script = fs.keyframe_script([fs.parse_keyframe(text) for text in frames])
    return face.run(script, sleep=lambda _s: clock.advance(0.005))


def load(board, index):
    return board.load_ma(index) * wg.PULL[index]


def test_q_maps_back_to_the_same_tick() -> None:
    for tick_range in ((0, 4095), (-158, 551), (1734, 2652), (3880, 4050), (-20000, 30000)):
        low, high = tick_range
        for tick in (low, low + 1, (low + high) // 2, high - 1, high):
            assert fs.firmware_tick_for_q(fs.q_for_tick(tick, tick_range), tick_range) == tick
    try:
        fs.q_for_tick(4096, (0, 4095))
    except ValueError:
        pass
    else:
        raise AssertionError("a tick outside the range was mapped")


def test_rest_pretensions_every_line() -> None:
    clock, board, face = session()
    face.start()
    assert board.state == fs.ENABLED_STATE and board.rejected_targets == 0
    assert run(clock, face, ["hold=4"]) == "finished"
    for index in wg.PULL:
        assert abs(load(board, index) - wg.PRELOAD_MA) <= wg.BAND_MA, (index, board.load_ma(index))
    assert abs(board.goal[wg.JAW] - 4000) <= 1
    assert board.rejected_targets == 0
    assert board.state == tuning_core.HOMING_REQUIRED_STATE           # torque off at the end
    held = {i for i in board.active_servos if i not in (wg.JAW, *wg.PULL, *wg.UPPER_PULL)}
    assert all(board.goal[i] == board.neutral[i] for i in held)       # off-face servos hold
    for index in wg.UPPER_PULL:            # no panel marks: the upper face rests where it was
        assert board.goal[index] == board.neutral[index], (index, board.goal[index])


def test_up_and_diag_drive_while_down_pays_out() -> None:
    clock, board, face = session()
    face.start()
    start_8 = board.goal[8]
    assert run(clock, face, ["hold=3", "hold=6 right=1,1"]) == "finished"
    neutral = fs.preview_neutrals().closed
    assert board.goal[14] == neutral[14] + 700 and board.goal[11] == neutral[11] + 580
    assert wg.PULL[8] * (board.goal[8] - start_8) < -100               # down followed out
    assert abs(load(board, 8) - wg.PRELOAD_MA) <= wg.BAND_MA
    assert face.stopped == "finished" and board.rejected_targets == 0


def test_down_drives_alone_and_up_diag_pay_out() -> None:
    clock, board, face = session()
    face.start()
    assert run(clock, face, ["hold=3", "hold=4 left=0.7,-1"]) == "finished"
    neutral = fs.preview_neutrals().closed
    assert board.goal[9] == neutral[9] + wg.PULL[9] * 300
    for index in (15, 19):
        # Up and diag pay out (by UP_DIAG_DOWN_COMPENSATION x down's pull):
        # never over the preload, so they do not hold the corner up.
        assert load(board, index) <= wg.PRELOAD_MA + wg.BAND_MA, (index, board.load_ma(index))
        pulled = wg.PULL[index] * (board.goal[index] - neutral[index])
        assert pulled <= -round(fs.UP_DIAG_DOWN_COMPENSATION * 300) + 2, (index, pulled)


def test_line_neutrals_follow_the_jaw() -> None:
    base = fs.preview_neutrals()
    opened = dict(base.closed); opened[14] = base.closed[14] - 50
    neutrals = fs.SessionNeutrals(base.closed, opened, 4000, 3880, wg.PRELOAD_MA,
                                  base.controller_session_id)
    clock, board, face = session(neutrals=neutrals)
    face.start()
    assert run(clock, face, ["hold=3", "hold=6 right=0,1 jaw=1"]) == "finished"
    assert abs(board.goal[wg.JAW] - 3880) <= 1
    jaw_j = face.controller.guard.jaw_fraction(board.present(wg.JAW))
    assert board.goal[14] == round(base.closed[14] - 50 * jaw_j) + 700


def test_a_stalled_host_trips_the_watchdog() -> None:
    clock, board, face = session()
    face.start()
    script = fs.keyframe_script([fs.parse_keyframe("hold=5")])
    for _ in range(100):
        face.step(script); clock.advance(0.005)
    clock.advance(0.2)                                    # the host stops sending
    assert not face.step(script)
    assert board.fault == tuning_core.headctl.FAULT_NAMES.index("WATCHDOG")
    assert "fault" in face.stopped.lower() or "left ENABLED" in face.stopped


def test_enable_is_refused_outside_the_profile_range() -> None:
    clock, board, face = session(lambda b: b.ranges.update({8: (0, 2000)}), envelope=False)
    try:
        face.start()
    except RuntimeError as error:
        assert "ERANGE" in str(error)
    else:
        raise AssertionError("Enable outside the range was accepted")


def test_a_line_over_the_abort_load_turns_torque_off() -> None:
    def stiff(board):
        pull, taut, _k, coupling = board.tendons[14]
        board.tendons[14] = (pull, taut, 12.0, coupling)
    clock, board, face = session(stiff)
    face.start()
    reason = run(clock, face, ["hold=2", "hold=6 right=0,1"])
    assert "line abort" in reason, reason
    assert board.state == tuning_core.HOMING_REQUIRED_STATE


def test_the_profile_range_clamps_targets() -> None:
    clock, board, face = session(lambda b: b.ranges.update({14: (0, 3400)}), envelope=False)
    face.start()
    run(clock, face, ["hold=2", "hold=4 right=0,1"])
    assert board.goal[14] == 3400 and face.controller.clamped.get(14, 0) > 0


def test_jaw_held_high_backs_off_then_stops() -> None:
    clock, board, face = session(lambda b: b.springs.update({wg.JAW: (4000, 200.0)}))
    face.start()                                          # goal 3880: about 425 mA
    reason = run(clock, face, ["hold=1", "hold=20 jaw=1"])
    assert "backed off closed" in reason, reason
    assert board.state == tuning_core.HOMING_REQUIRED_STATE


def test_neutrals_from_another_boot_are_refused() -> None:
    base = fs.preview_neutrals()
    stale = fs.SessionNeutrals(base.closed, base.open, 4000, 3880, wg.PRELOAD_MA, 1234)
    clock, board, face = session(neutrals=stale)
    try:
        face.start()
    except RuntimeError as error:
        assert "restart" in str(error)
    else:
        raise AssertionError("neutrals from another boot were accepted")
    assert board.state == tuning_core.HOMING_REQUIRED_STATE           # nothing enabled


def test_session_neutral_file_round_trip() -> None:
    slices = []
    for jaw_tick, offset in ((4001, 0), (3882, 40)):
        slices.append({"jaw_tick": jaw_tick, "neutral": {
            str(i): {"goal_tick": 1000 + 10 * i + offset, "present_tick": 0, "current_ma": 30}
            for i in wg.PULL}})
    log = {"boxes": {"preload_ma": "35"}, "slices": slices, "source": "x_ranges.json"}
    data = range_finder.session_neutral(log, 7, 77)
    path = Path(tempfile.mkdtemp()) / ("t" + range_finder.SESSION_NEUTRAL_SUFFIX)
    tuning_core.write_json_atomically(path, data)
    loaded = fs.load_session_neutrals(path)
    assert loaded.closed[15] == 1150 and loaded.open[15] == 1190
    assert (loaded.jaw_closed_tick, loaded.jaw_open_tick, loaded.preload_ma,
            loaded.controller_session_id) == (4001, 3882, 35, 77)
    assert fs.newest_session_neutrals(path.parent) == path
    guard = fs.guard_from_neutrals(loaded)
    assert guard.neutral_at(15, 1.0) == 1190
    far = fs.SessionNeutrals(loaded.closed, loaded.open, 4000, 3950, 35, 77)
    try:
        fs.guard_from_neutrals(far)
    except ValueError as error:
        assert "3880" in str(error)
    else:
        raise AssertionError("an open slice far from 3880 was accepted")


def test_session_envelope_numbers() -> None:
    clock, board, face = session()
    face.link.connect()
    profile = fs.headctl.read_profile(board)
    present = {i: board.present(i) for i in board.active_servos}
    present[13] = 9000                                   # off the face, outside 0..4095
    n = fs.preview_neutrals()
    out, notes = fs.session_envelope(profile, n, present)
    slots = {s["index"]: s for s in out["joints"]}
    assert (slots[14]["min_tick"], slots[14]["max_tick"]) == (n.closed[14] - 1000, n.closed[14] + 710)
    assert (slots[15]["min_tick"], slots[15]["max_tick"]) == (n.closed[15] - 590, n.closed[15] + 1000)
    nine = (n.closed[9] - 1000, n.closed[9] + 310) if wg.PULL[9] > 0 else (n.closed[9] - 310, n.closed[9] + 1000)
    assert (slots[9]["min_tick"], slots[9]["max_tick"]) == nine
    assert (slots[7]["min_tick"], slots[7]["max_tick"], slots[7]["home_tick"]) == (3870, 4050, 4000)
    assert (slots[13]["min_tick"], slots[13]["max_tick"], slots[13]["home_tick"]) == (6952, 11048, 9000)
    assert (slots[0]["min_tick"], slots[0]["max_tick"]) == (0, 4095)    # inside: kept
    assert all(slots[i]["home_tick"] == n.closed[i] for i in wg.PULL)
    assert any(note.startswith("servo 13:") for note in notes)
    # Both jaw slices count: the pull end is the furthest of the two.
    opened = dict(n.closed); opened[19] = n.closed[19] - 120
    out, _ = fs.session_envelope(profile, fs.SessionNeutrals(n.closed, opened, 4000, 3880, 35, None),
                                 present)
    assert {s["index"]: s for s in out["joints"]}[19]["min_tick"] == n.closed[19] - 120 - 710
    # Far slack is taken in; a line past its pull end is refused.
    far = dict(present); far[11] = n.closed[11] - 1500
    out, _ = fs.session_envelope(profile, n, far)
    assert {s["index"]: s for s in out["joints"]}[11]["min_tick"] == n.closed[11] - 1600
    for index, tick in ((14, n.closed[14] + 800), (7, 3800)):
        bad = dict(present); bad[index] = tick
        try:
            fs.session_envelope(profile, n, bad)
        except ValueError:
            continue
        raise AssertionError(f"servo {index} at {tick} was accepted")


def test_session_envelope_is_uploaded_once() -> None:
    clock, board, face = session()
    face.start()
    assert board.uploads == 1 and board.generation == 18
    assert board.ranges[15] == (fs.preview_neutrals().closed[15] - 590,
                                fs.preview_neutrals().closed[15] + 1000)
    assert face.log["envelope"]["generation"] == 18
    run(clock, face, ["hold=1"])
    board.state = tuning_core.HOMING_REQUIRED_STATE      # torque is off after the run
    link = fs.StreamLink(board, clock)
    again = fs.FaceSession(link, fs.preview_neutrals(), Path(tempfile.mkdtemp()),
                           say=lambda _text: None)
    again.start()                                         # same envelope: no second upload
    assert board.uploads == 1 and again.log["envelope"]["generation"] is None
    run(clock, again, ["hold=1"])


def test_demo_path_is_smooth_and_inside_the_action_space() -> None:
    points = fs.demo_waypoints(leg_s=2.0, hold_s=0.5)
    script = fs.path_script(points, loops=2)
    assert abs(script.total_s - 2 * sum(w.move_s + w.hold_s for w in points)) < 1e-9
    assert script(0.0) == ({"right": (0.0, 0.0), "left": (0.0, 0.0)}, 0.0)
    assert script(2.0 + 0.25)[0]["right"] == (0.0, 1.0)          # first waypoint, held
    assert script(script.total_s) is None
    last = None
    t = 0.0
    while t < script.total_s:
        actions, j = script(t)
        values = (*actions["right"], *actions["left"], j)
        assert 0.0 <= actions["right"][0] <= 1.0 and -1.0 <= actions["right"][1] <= 1.0
        assert 0.0 <= j <= 1.0
        for side in ("right", "left"):                            # down only after rest
            d, v = actions[side]
            assert not (v < 0.0 and d > 1e-9), (t, side, d, v)
        if j > 0.0:                                               # no down with the jaw open
            assert actions["right"][1] >= 0.0 and actions["left"][1] >= 0.0
        if last is not None:                                      # 20 ms apart: small changes
            assert max(abs(a - b) for a, b in zip(values, last)) < 0.02, (t, values, last)
        last = values
        t += 0.02
    assert last == (0.0, 0.0, 0.0, 0.0, 0.0)                     # ends at rest


def test_a_driver_never_lets_go_of_the_slack_it_took_in() -> None:
    clock, board, face = session()
    face.start()
    frames = [fs.parse_keyframe("hold=3"), fs.parse_keyframe("hold=1 right=0.01,0")]
    script = fs.keyframe_script(frames)
    rest_11 = None
    while face.step(script):
        clock.advance(0.005)
        if clock.now - face.started_s < 3.0:
            rest_11 = board.goal[11]                  # the follower at the preload
        else:
            # Driving at 1 % (6 ticks from neutral) does not let 11 go back
            # from where the follower had it.
            assert wg.PULL[11] * board.goal[11] >= wg.PULL[11] * rest_11 - 1, (board.goal[11], rest_11)
    assert face.stopped == "finished"


def test_demo_runs_on_the_simulated_board() -> None:
    clock, board, face = session()
    face.start()
    script = fs.path_script(fs.demo_waypoints(leg_s=1.5, hold_s=0.2))
    assert face.run(script, sleep=lambda _s: clock.advance(0.005)) == "finished"
    assert board.rejected_targets == 0 and not face.controller.clamped


def test_a_less_open_jaw_limit_from_the_session_file() -> None:
    slices = [{"jaw_tick": 3998, "jaw_target": 4000, "neutral": {
                  str(i): {"goal_tick": 1000 + i, "present_tick": 0, "current_ma": 30} for i in wg.PULL}},
              {"jaw_tick": 3903, "jaw_target": 3900, "neutral": {
                  str(i): {"goal_tick": 1100 + i, "present_tick": 0, "current_ma": 30} for i in wg.PULL}}]
    data = range_finder.session_neutral({"boxes": {"preload_ma": "35"}, "slices": slices}, 7, 5)
    path = Path(tempfile.mkdtemp()) / ("t" + range_finder.SESSION_NEUTRAL_SUFFIX)
    tuning_core.write_json_atomically(path, data)
    n = fs.load_session_neutrals(path)
    assert n.jaw_open_limit == 3900
    guard = fs.guard_from_neutrals(n)
    assert guard.jaw_target(1.0) == 3900 and guard.neutral_at(15, 1.0) == 1115
    assert guard.jaw_fraction(3950) == 0.5
    clock, board, face = session()
    face.link.connect()
    profile = fs.headctl.read_profile(board)
    present = {i: board.present(i) for i in board.active_servos}
    present.update(n.closed)
    out, _ = fs.session_envelope(profile, n, present)
    jaw = {s["index"]: s for s in out["joints"]}[7]
    assert (jaw["min_tick"], jaw["max_tick"]) == (3890, 4050)
    for target in (3850, 3990):                           # more open than mapped, or almost shut
        bad = fs.SessionNeutrals(n.closed, n.open, 3998, target, 35, 5, None, target)
        try:
            fs.guard_from_neutrals(bad)
        except ValueError:
            continue
        raise AssertionError(f"open limit {target} was accepted")
    old = fs.SessionNeutrals(n.closed, n.open, 3998, 3883, 35, 5)  # no target: the mapped 3880
    assert fs.guard_from_neutrals(old).jaw_target(1.0) == 3880


def test_skipped_servos_keep_their_profile_range() -> None:
    saved_skipped = tuning_core.SKIPPED_SERVOS
    tuning_core.SKIPPED_SERVOS = (5, 6)
    try:
        clock, board, face = session()
        face.link.connect()
        profile = fs.headctl.read_profile(board)
        present = {i: board.present(i) for i in board.active_servos}
        n = fs.preview_neutrals()
        out, notes = fs.session_envelope(profile, n, present)
        before = {s["index"]: s for s in profile["joints"]}
        after = {s["index"]: s for s in out["joints"]}
        for index in tuning_core.SKIPPED_SERVOS:
            assert after[index] == before[index]
            assert not any(note.startswith(f"servo {index}:") for note in notes)
        present[5] = 9000                                 # outside: refused, not widened
        try:
            fs.session_envelope(profile, n, present)
        except ValueError as error:
            assert "skipped" in str(error)
        else:
            raise AssertionError("a skipped servo outside its range was accepted")
    finally:
        tuning_core.SKIPPED_SERVOS = saved_skipped


def test_rest_brings_every_line_back_to_neutral() -> None:
    clock, board, face = session()
    face.start()
    neutral = fs.preview_neutrals().closed
    frames = ["hold=3", "hold=5 right=1,1 left=0,-1", "hold=5"]
    assert run(clock, face, frames) == "finished"
    for index in wg.PULL:
        # Back by position: no line stays pulled past its neutral (before
        # 2026-10-21 they stayed near full pull at the end of every demo),
        # and none is loaded over the preload. A line may sit a little
        # slacker: here the other two at neutral already stretch it.
        pulled = wg.PULL[index] * (board.goal[index] - neutral[index])
        assert -200 <= pulled <= 0, (index, board.goal[index], neutral[index])
        assert load(board, index) <= wg.PRELOAD_MA + wg.BAND_MA, (index, board.load_ma(index))
    assert fs.side_activity({"right": (0.4, 0.7)}, "right") == 0.7
    assert fs.side_activity({"right": (0.9, -0.3)}, "right") == 0.3      # d does not count below
    assert fs.side_activity({}, "left") == 0.0


def test_neutrals_move_toward_slack_for_up_and_diag() -> None:
    n = fs.preview_neutrals()
    moved = fs.shifted_neutrals(n)
    for side in wg.SIDES.values():
        assert moved.closed[side["down"]] == n.closed[side["down"]]
        for role in ("up", "diag"):
            i = side[role]                                   # toward slack: against the pull
            offset = fs.NEUTRAL_SLACK_OFFSET_TICKS[role]
            assert wg.PULL[i] * (moved.closed[i] - n.closed[i]) == -offset
            assert wg.PULL[i] * (moved.open[i] - n.open[i]) == -offset
    none = fs.shifted_neutrals(n, {"up": 0, "diag": 0, "down": 0})
    assert none.closed == n.closed
    # Driving down (v < 0) with shifted neutrals must make up/diag actively
    # compensate (pay out UP_DIAG_DOWN_COMPENSATION x down's range) beyond their
    # shifted neutral so they never hold the lip corners up against down.
    clock, board, face = session(neutrals=moved)
    face.start()
    assert run(clock, face, ["hold=3 right=0,-1 left=0,-1"]) == "finished"
    expected_comp = -round(fs.UP_DIAG_DOWN_COMPENSATION * face.line_ranges["down"])
    for index in (11, 14, 15, 19):
        pulled = wg.PULL[index] * (board.goal[index] - moved.closed[index])
        assert pulled <= expected_comp, (index, board.goal[index], moved.closed[index], pulled)


def test_duchenne_never_pulls_down_past_neutral() -> None:
    """Down only follows in a smile: it may pay out, never be taken in past
    its neutral (the head's duchenne run at 21:59Z, made before the per-line
    cap, took line 8 in to +183)."""
    clock, board, face = session()
    face.start()
    neutral = fs.preview_neutrals().closed
    most = {8: -10**9, 9: -10**9}

    def watch(_t: float) -> None:
        for i in most:
            most[i] = max(most[i], wg.PULL[i] * (board.goal[i] - neutral[i]))
    face.on_update = watch
    script = fs.path_script(fs.duchenne_waypoints(leg_s=1.5))
    assert face.run(script, sleep=lambda _s: clock.advance(0.005)) == "finished"
    assert all(v <= 0 for v in most.values()), most


def test_angry_never_winds_up_or_diag_back() -> None:
    """2026-10-22: a fixed pay-out of up and diag while down drove wound them
    back the other way and the corners went up. They now follow by load: on
    lines with only 100 ticks of slack none ends wound back during angry."""
    base = fs.preview_neutrals()
    touching = dict(fs.PREVIEW_TAUT)                      # a camera neutral: just touching
    camera = fs.SessionNeutrals(touching, dict(touching), 4000, 3880, 35,
                                base.controller_session_id, None, None, "camera_knee")
    def short_slack(board):
        for index in (11, 14, 15, 19):
            board.tendon_slack_len[index] = 100
    clock, board, face = session(short_slack, neutrals=camera)
    face.start()
    assert face.controller.preload_ma == fs.CAMERA_FOLLOW_PRELOAD_MA
    worst = {i: 0 for i in (11, 14, 15, 19)}

    def watch(_t: float) -> None:
        for i in worst:
            worst[i] = min(worst[i], load(board, i))
    face.on_update = watch
    script = fs.path_script(fs.angry_waypoints(leg_s=1.5))
    assert face.run(script, sleep=lambda _s: clock.advance(0.005)) == "finished"
    assert all(v >= -wg.BAND_MA for v in worst.values()), worst


def test_lower_lip_and_jaw_open_override() -> None:
    frame = fs.parse_keyframe("hold=3 jaw=1 lip=1,1 brow=1,1")
    assert frame.lip == (1.0, 1.0)
    clock, board, face = session(upper_ranges=dict(fs.UPPER_SESSION_RANGES), jaw_open=3880)
    face.start()
    hold = {i: board.goal[i] for i in (3, 4)}
    run(clock, face, ["hold=1", "hold=6 jaw=1 lip=1,1"])
    assert abs(board.goal[wg.JAW] - 3880) <= 1                   # the override, not 3880 by luck
    assert board.goal[3] == hold[3] + 400 and board.goal[4] == hold[4] - 400   # 10-23 marks
    assert face.controller.clamped == {}


def test_upper_face_rests_at_the_panel_marks() -> None:
    present = {3: 3084, 4: 1816, 10: 2800, 16: 4017, 12: 1770, 17: 2476}
    marks = {3: {"neutral": {"present_tick": 3098}}, 4: {"neutral": {"present_tick": 1813}},
             10: {"neutral": {"present_tick": 2300}}}            # 500 away: moved there first
    rest, source = fs.choose_upper_neutral(present, marks)
    want = {i: mark + wg.UPPER_PULL[i] * fs.UPPER_REST_SHIFT_TICKS.get(i, 0)
            for i, mark in ((3, 3098), (4, 1813))}
    assert rest[3] == want[3] and rest[4] == want[4], (rest, want)
    assert source[3].startswith("panel mark") and source[4].startswith("panel mark")
    # A servo left at its shifted rest by an earlier run still counts as near its mark.
    left = {**present, 3: want[3]}
    assert fs.choose_upper_neutral(left, marks)[0][3] == want[3]
    assert rest[10] == 2300 and "first" in source[10] and rest[16] == 4017
    assert source[16].startswith("pose at Enable")                # no mark
    # More than one turn from its mark (a turn-count jump): refused.
    try:
        fs.choose_upper_neutral({**present, 10: 2300 + 4097}, marks)
        raise AssertionError("a mark over one turn away was accepted")
    except ValueError as error:
        assert "turn-count" in str(error) and "10" in str(error)
    # In a session the lips move to their marks at Enable and pull from there.
    def at(board):
        board.goal.update({3: 3084, 4: 1816})
    clock, board, face = session(at, upper_ranges=dict(fs.UPPER_SESSION_RANGES))
    face.panel_marks = marks
    face.start()
    run(clock, face, ["hold=1", "hold=4 lip=1,1"])
    assert board.goal[3] == want[3] + 400 and board.goal[4] == want[4] - 400   # from the shifted rests
    assert face.log["upper_neutral"]["3"]["from"].startswith("panel mark")
    # The envelope covers the rest and the pull from it.
    profile = fs.headctl.read_profile(board)
    out, _ = fs.session_envelope(profile, fs.preview_neutrals(),
                                 {i: board.present(i) for i in board.active_servos},
                                 upper_ranges=dict(fs.UPPER_SESSION_RANGES), upper_neutral=rest)
    slot = {s["index"]: s for s in out["joints"]}
    assert slot[3]["max_tick"] >= want[3] + 400 and slot[4]["min_tick"] <= want[4] - 400


def test_far_upper_servo_moves_to_its_mark_before_the_script() -> None:
    """Brow 10 sits 800 ticks from its panel mark at Enable (run
    192015_face_duchenne, 2026-11-10): the session first moves it to the mark
    at the line speed limit, and the script clock starts only once it is there."""
    def at(board):
        board.goal[10] = 2300 + 450
    clock, board, face = session(at, upper_ranges=dict(fs.UPPER_SESSION_RANGES))
    face.panel_marks = {10: {"neutral": {"present_tick": 2300}}}
    first = []
    face.on_command = lambda t, _a, _j, targets: first.append((t, targets.get(10))) if not first else None
    face.start()
    assert set(face.log["approach"]) == {"10"}
    assert run(clock, face, ["hold=1", "hold=2 brow=1,1"]) == "finished"
    assert 450 / fs.LINE_SPEED_TPS <= face.log["approach_s"] <= 450 / fs.LINE_SPEED_TPS + 1.0
    t0, target = first[0]
    assert t0 < 0.1 and abs(target - 2300) <= fs.APPROACH_DONE_TICKS + 10, first
    assert face.log["upper_neutral"]["10"]["from"].startswith("panel mark")


def test_upper_face_approach_times_out() -> None:
    """A servo that cannot reach its mark: torque off, the script never starts."""
    def at(board):
        board.goal[10] = 2300 + 450
    clock, board, face = session(at, upper_ranges=dict(fs.UPPER_SESSION_RANGES))
    face.panel_marks = {10: {"neutral": {"present_tick": 2300}}}
    face.start()
    face._approach_until = face.started_s + 0.5                 # far too short for 450 ticks
    started = []
    face.on_update = lambda t: started.append(t)
    reason = run(clock, face, ["hold=1"])
    assert reason.startswith("upper face did not reach"), reason
    assert started == []


def test_turn_jumps_are_refused() -> None:
    """2026-11-10 after a 12 V cycle: upper lip 13 sat 3413 ticks from its
    mark (refuse, do not move it there) and line 9 sat 3936 ticks 'slack'
    (refuse the envelope)."""
    marks = {13: {"neutral": {"present_tick": 4930}, "end_1": {"present_tick": 5728}},
             10: {"neutral": {"present_tick": 2064}, "end_1": {"present_tick": 1528}}}
    old = dict(wg.UPPER_PULL)
    wg.configure_upper_lips({13: 1, 18: -1}, 798)
    try:
        try:
            fs.choose_upper_neutral({13: 1517, 10: 2064, 18: 2217}, marks)
            raise AssertionError("a turn-jumped upper lip was accepted")
        except ValueError as error:
            assert "13" in str(error) and "turn-count" in str(error)
        # The brows of 2026-11-10 (327 outside their 536 span) still move first.
        rest, source = fs.choose_upper_neutral({10: 1201}, marks)
        assert rest[10] == 2064 and "first" in source[10]
    finally:
        wg.clear_upper_lips()
    assert wg.UPPER_PULL == old
    clock = Clock()
    board = fs.face_preview_controller(clock)
    neutrals = fs.preview_neutrals()
    present = {i: board.present(i) for i in board.active_servos}
    present[9] = neutrals.closed[9] - wg.PULL[9] * 3936           # far on the slack side
    try:
        fs.session_envelope(fs.headctl.read_profile(board), neutrals, present)
        raise AssertionError("a line 3936 ticks slack was accepted")
    except ValueError as error:
        assert "servo 9" in str(error) and "slack" in str(error)
    present[9] = neutrals.closed[9] - wg.PULL[9] * 300            # ordinary slack: fine
    fs.session_envelope(fs.headctl.read_profile(board), neutrals, present)


def test_span_notes_are_said_at_session_start() -> None:
    """A capped span (17 to 12) gives a note in FaceSession.__init__; it must
    be said, not crash (2026-11-10: AttributeError before the first probe)."""
    said = []
    marks = {12: {"neutral": {"present_tick": 1365}, "end_1": {"present_tick": 618}},
             17: {"neutral": {"present_tick": 2655}, "end_1": {"present_tick": 1775}}}
    clock = Clock()
    board = fs.face_preview_controller(clock)
    fs.FaceSession(fs.StreamLink(board, clock), fs.preview_neutrals(), Path(tempfile.mkdtemp()),
                   say=said.append, panel_marks=marks)
    assert any("capped" in text for text in said), said


def test_upper_face_stops_at_the_panel_end_marks() -> None:
    marks = {10: {"neutral": {"present_tick": 3121}, "end_1": {"present_tick": 2625}},
             16: {"neutral": {"present_tick": 3}, "end_1": {"present_tick": 598}},
             12: {"neutral": {"present_tick": 1738}, "end_1": {"present_tick": 2000}}}  # wrong way
    spans, notes = fs.choose_upper_spans(marks)
    assert spans == {10: 496 + fs.UPPER_SPAN_EXTRA_TICKS.get(10, 0),
                     16: 595 + fs.UPPER_SPAN_EXTRA_TICKS.get(16, 0)} and any("servo 12" in n for n in notes)
    extra = fs.choose_upper_spans({17: {"neutral": {"present_tick": 2553}, "end_1": {"present_tick": 1807}}})[0]
    assert extra[17] == 746 + fs.UPPER_SPAN_EXTRA_TICKS.get(17, 0)
    # Left oculi capped to the right oculi's span (2026-11-10 marks: 880 vs 747).
    oculi = {12: {"neutral": {"present_tick": 1365}, "end_1": {"present_tick": 618}},
             17: {"neutral": {"present_tick": 2655}, "end_1": {"present_tick": 1775}}}
    capped, notes = fs.choose_upper_spans(oculi)
    assert capped == {12: 747, 17: 747} and any("capped" in n for n in notes), (capped, notes)
    shorter = {**oculi, 17: {"neutral": {"present_tick": 2655}, "end_1": {"present_tick": 2055}}}
    assert fs.choose_upper_spans(shorter)[0][17] == 600   # a shorter left span stays
    guard = wg.WorkspaceGuard(dict(NEUTRAL_FOR_GUARD), upper_neutral={10: 3121, 16: 3},
                              upper_spans=spans)
    targets, _ = guard.upper_command({"brow": (1.0, 1.0)})
    assert targets[10] == 3121 - 496                     # the right brow's own end mark
    assert targets[16] == 3 + 520                        # the brow range (520) is shorter here


def test_envelope_sets_the_jaw_current_and_step() -> None:
    clock, board, face = session()
    face.link.connect()
    profile = fs.headctl.read_profile(board)
    present = {i: board.present(i) for i in board.active_servos}
    out, notes = fs.session_envelope(profile, fs.preview_neutrals(), present, jaw_open=3620,
                                     jaw_current_ma=750, jaw_speed_tps=800)
    jaw = {s["index"]: s for s in out["joints"]}[wg.JAW]
    assert jaw["operating_current_ma"] == 750 and jaw["min_tick"] == 3610
    rng = jaw["max_tick"] - jaw["min_tick"]
    assert int(rng * jaw["max_position_fraction_per_control_cycle"]) == 2   # 2 ticks/cycle = 1000 ticks/s
    assert jaw["max_acceleration_fraction_per_control_cycle"] <= jaw["max_position_fraction_per_control_cycle"]
    assert any("750 mA" in n for n in notes)


def test_keyframes() -> None:
    frame = fs.parse_keyframe("hold=3 right=0.5,1 left=0.2,-0.4 jaw=0.3")
    assert (frame.hold_s, frame.right, frame.left, frame.jaw) == (3.0, (0.5, 1.0), (0.2, -0.4), 0.3)
    script = fs.keyframe_script([fs.parse_keyframe("hold=1"), frame])
    assert script(0.5) == ({"right": (0.0, 0.0), "left": (0.0, 0.0)}, 0.0)
    assert script(2.0)[1] == 0.3 and script(4.5) is None
    for bad in ("hold=0", "speed=3", "right=1", "hold=nan"):
        try:
            fs.parse_keyframe(bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad!r}")


def test_upper_duchenne_and_angry_demos_run_on_simulated_board() -> None:
    for name in ("upper", "duchenne", "angry", "expressions"):
        clock, board, face = session(upper_ranges=fs.UPPER_SESSION_RANGES)
        face.start()
        # The tours' own move time: the 991-tick oculi need about 3.3 s at
        # 300 ticks/s to get back to rest; 1.5 s moves left them short.
        points = fs.DEMO_TOURS[name]()
        script = fs.path_script(points)
        assert face.run(script, sleep=lambda _s: clock.advance(0.005)) == "finished", name
        assert board.rejected_targets == 0 and not face.controller.clamped, name
        for i in wg.UPPER_PULL:
            assert board.goal[i] == board.neutral[i], (name, i, board.goal[i], board.neutral[i])


def main() -> None:
    test_q_maps_back_to_the_same_tick()
    test_rest_pretensions_every_line()
    test_up_and_diag_drive_while_down_pays_out()
    test_down_drives_alone_and_up_diag_pay_out()
    test_line_neutrals_follow_the_jaw()
    test_a_stalled_host_trips_the_watchdog()
    test_enable_is_refused_outside_the_profile_range()
    test_a_line_over_the_abort_load_turns_torque_off()
    test_the_profile_range_clamps_targets()
    test_jaw_held_high_backs_off_then_stops()
    test_neutrals_from_another_boot_are_refused()
    test_session_neutral_file_round_trip()
    test_session_envelope_numbers()
    test_session_envelope_is_uploaded_once()
    test_demo_path_is_smooth_and_inside_the_action_space()
    test_a_driver_never_lets_go_of_the_slack_it_took_in()
    test_a_less_open_jaw_limit_from_the_session_file()
    test_skipped_servos_keep_their_profile_range()
    test_rest_brings_every_line_back_to_neutral()
    test_neutrals_move_toward_slack_for_up_and_diag()
    test_duchenne_never_pulls_down_past_neutral()
    test_angry_never_winds_up_or_diag_back()
    test_lower_lip_and_jaw_open_override()
    test_upper_face_rests_at_the_panel_marks()
    test_far_upper_servo_moves_to_its_mark_before_the_script()
    test_upper_face_approach_times_out()
    test_turn_jumps_are_refused()
    test_span_notes_are_said_at_session_start()
    test_upper_face_stops_at_the_panel_end_marks()
    test_envelope_sets_the_jaw_current_and_step()
    test_demo_runs_on_the_simulated_board()
    test_upper_duchenne_and_angry_demos_run_on_simulated_board()
    test_keyframes()
    print("face stream: all tests passed")


if __name__ == "__main__":
    main()
