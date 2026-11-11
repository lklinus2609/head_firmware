"""Regression tests for the knee rule of the modiolus knee test (knee_test.find_knee)."""

from __future__ import annotations

from pathlib import Path
import random
import sys
from types import SimpleNamespace

sys.modules.setdefault("serial", SimpleNamespace(Serial=object))
FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))

import knee_test as kt  # noqa: E402


def curve(contact: int, slope: float, base: float = 12.0, noise: float = 2.0,
          length: int = 500, seed: int = 1):
    rng = random.Random(seed)
    return [(t, round(base + rng.uniform(-noise, noise) + max(0, t - contact) * slope))
            for t in range(0, length, 3)]


def test_knee_after_a_noisy_flat_start() -> None:
    result = kt.find_knee(curve(300, 0.5))
    assert result["reason"] == "knee" and result["baseline_slack"]
    assert 11 <= result["baseline_ma"] <= 13 and result["threshold_ma"] >= result["baseline_ma"] + 8
    assert 300 < result["knee_travel"] <= 300 + 30          # 8 mA over at 0.5 mA/tick: ~16 ticks


def test_noisier_baseline_raises_the_threshold() -> None:
    quiet = kt.find_knee(curve(300, 0.5, noise=1.0))
    noisy = kt.find_knee(curve(300, 0.5, noise=6.0))
    assert noisy["threshold_ma"] > quiet["threshold_ma"]
    assert noisy["knee_travel"] >= quiet["knee_travel"]


def test_single_spikes_are_not_a_knee() -> None:
    samples = curve(10_000, 0.0)
    samples[50] = (samples[50][0], 90)                        # one spike
    samples[51] = (samples[51][0], 90)
    result = kt.find_knee(samples)
    assert result["knee_travel"] is None and "never" in result["reason"]


def test_a_line_that_is_tight_from_the_start() -> None:
    result = kt.find_knee(curve(0, 0.5))
    assert not result["baseline_slack"]
    result = kt.find_knee([(0, 1), (3, 2)])
    assert result["knee_travel"] is None and "too few" in result["reason"]


def test_camera_knee_rule() -> None:
    quiet = [(t, 0.2) for t in range(0, 300, 10)]
    rising = quiet + [(300, 0.6), (310, 0.4), (320, 0.7), (330, 0.8), (340, 0.9), (350, 1.6)]
    assert kt.camera_knee(rising) == 320                  # first of three frames in a row over 0.5
    assert kt.camera_knee(quiet) is None
    # Stick-slip jump (line 19, run 155544_knee, 2026-11-09): straight past
    # the stop level in two frames; the jump's first frame is the knee.
    jump = [(1, 0.18), (8, 0.26), (99, 0.37), (110, 0.32), (117, 1.7), (124, 2.07)]
    assert kt.camera_knee(jump) == 117
    assert kt.camera_knee([(1, 0.2), (117, 1.7), (124, 0.3)]) is None   # one frame: noise
    assert kt.travel_at([[0.0, 0, 100, 0], [1.0, 0, 200, 0]], 0.25, 100, 1) == 25
    assert kt.travel_at([[0.0, 0, 100, 0], [1.0, 0, 50, 0]], 0.5, 100, -1) == 25
    assert kt.travel_at([], 1.0, 0, 1) == 0.0


def test_camera_session_neutral_loads_in_head_face() -> None:
    import face_stream as fs
    import tempfile
    import tuning_core
    import workspace_guard as wg
    log = {"jaw_tick": 3995, "source": "x_knee.json", "sides": {
        side: {"lines": {str(i): {"camera_knee": {"tick": 1000 + i}} for i in lines.values()}}
        for side, lines in wg.SIDES.items()}}
    data = kt.camera_session_neutral(log, 7, 35)
    path = Path(tempfile.mkdtemp()) / ("t" + kt.SESSION_NEUTRAL_SUFFIX)
    tuning_core.write_json_atomically(path, data)
    n = fs.load_session_neutrals(path)
    assert n.kind == "camera_knee" and n.closed == n.open and n.closed[15] == 1015
    guard = fs.guard_from_neutrals(n)                     # the jaw slices pass its checks
    assert guard.jaw_target(1.0) == kt.JAW_OPEN_TICK
    assert fs.shifted_neutrals(n).kind == "camera_knee"
    log["sides"]["left"]["lines"]["15"]["camera_knee"]["tick"] = None
    assert kt.camera_session_neutral(log, 7, 35) is None  # a line without a knee: no file


def test_take_in_stops_on_the_camera() -> None:
    """On the simulated board with a fake corner watch: the corner moves once
    line 14 is 40 ticks past its taut tick; the take-in stops on the camera
    and the camera knee is found there."""
    import tempfile
    import time as time_module
    import face_stream as fs
    import jaw_probe
    import sweep_runner
    import tuning_core
    import workspace_guard as wg
    board = fs.face_preview_controller()
    link = tuning_core.TuningLink(board)
    link.connect()
    taut = fs.PREVIEW_TAUT[14]

    class FakeWatch:
        def poll(self):
            time_module.sleep(0.02)
            return time_module.monotonic(), max(0.0, board.present(14) - (taut + 40)) * 0.05

    with tempfile.TemporaryDirectory() as directory:
        runner = sweep_runner.Runner(link, dict(board.active_servos), Path(directory), 650,
                                     dict(tuning_core.SWEEP_BOX_DEFAULTS), Path(directory) / "STOP",
                                     say=lambda _text: None)
        assert runner.start()
        probe = jaw_probe.Probe(runner, wg.JAW, dict(wg.PULL), 280, None, Path(directory))
        test = kt.KneeTest(probe, 3, 80, 900, say=lambda _text: None)
        result = test.take_in(14, FakeWatch())
        runner.disable(); runner.close()
    assert result["stopped_by"].startswith("camera"), result["stopped_by"]
    knee = result["camera_knee"]["tick"]
    assert taut + 40 <= knee <= taut + 40 + 30, (knee, taut)


def test_take_in_gives_up_when_the_corner_never_moves() -> None:
    """A detached magnet: the corner never moves, so the take-in stops after
    slack + CAM_NO_MOTION_TICKS instead of running to --max-travel."""
    import tempfile
    import time as time_module
    import face_stream as fs
    import jaw_probe
    import sweep_runner
    import tuning_core
    import workspace_guard as wg
    board = fs.face_preview_controller()
    board.tendons[14] = (1, 10**6, 0.2, 0.0)             # the line never gets tight
    link = tuning_core.TuningLink(board)
    link.connect()

    class StillWatch:
        def poll(self):
            time_module.sleep(0.01)
            return time_module.monotonic(), 0.15

    with tempfile.TemporaryDirectory() as directory:
        runner = sweep_runner.Runner(link, dict(board.active_servos), Path(directory), 650,
                                     dict(tuning_core.SWEEP_BOX_DEFAULTS), Path(directory) / "STOP",
                                     say=lambda _text: None)
        assert runner.start()
        probe = jaw_probe.Probe(runner, wg.JAW, dict(wg.PULL), 280, None, Path(directory))
        test = kt.KneeTest(probe, 10, 80, 1500, say=lambda _text: None, slack=100)
        result = test.take_in(14, StillWatch())
        runner.disable(); runner.close()
    assert "magnet off" in result["stopped_by"], result["stopped_by"]
    travel = result["curve"][-1][1] - result["curve"][0][1]
    assert 100 + kt.CAM_NO_MOTION_TICKS <= travel < 1500, travel


def test_find_slack_by_load() -> None:
    """A tight line pays out, a line wound the other way (negative load) is
    taken in, a slack line stays; none ends wound the other way."""
    import tempfile
    import face_stream as fs
    import jaw_probe
    import sweep_runner
    import tuning_core
    import workspace_guard as wg
    board = fs.face_preview_controller()
    taut = fs.PREVIEW_TAUT
    for index in (8, 11, 14):
        board.tendon_slack_len[index] = 200
    board.goal[8] = taut[8] + wg.PULL[8] * 150             # tight
    board.goal[11] = taut[11] - 450                        # wound the other way
    board.goal[14] = taut[14] - 100                        # slack already
    board.neutral = dict(board.goal)
    link = tuning_core.TuningLink(board)
    link.connect()
    with tempfile.TemporaryDirectory() as directory:
        runner = sweep_runner.Runner(link, dict(board.active_servos), Path(directory), 650,
                                     dict(tuning_core.SWEEP_BOX_DEFAULTS), Path(directory) / "STOP",
                                     say=lambda _text: None)
        assert runner.start()
        probe = jaw_probe.Probe(runner, wg.JAW, dict(wg.PULL), 280, None, Path(directory))
        test = kt.KneeTest(probe, 3, 80, 900, say=lambda _text: None, slack=50)
        reason, report = test.find_slack([8, 11, 14], 50)
        runner.disable(); runner.close()
    assert reason is None, reason
    assert all(report[str(i)]["found"] for i in (8, 11, 14)), report
    assert report["11"].get("wound_the_other_way") and not report["8"].get("wound_the_other_way")
    for index in (8, 11, 14):
        coordinate = wg.PULL[index] * (board.goal[index] - taut[index])
        assert -200 <= coordinate <= 0, (index, coordinate)   # slack, not tight, not reversed
    assert wg.PULL[8] * (report["8"]["end_goal"] - report["8"]["start_goal"]) < 0   # paid out
    assert report["11"]["end_goal"] > report["11"]["start_goal"]   # taken in


def main() -> None:
    test_knee_after_a_noisy_flat_start()
    test_noisier_baseline_raises_the_threshold()
    test_single_spikes_are_not_a_knee()
    test_a_line_that_is_tight_from_the_start()
    test_camera_knee_rule()
    test_camera_session_neutral_loads_in_head_face()
    test_take_in_stops_on_the_camera()
    test_take_in_gives_up_when_the_corner_never_moves()
    test_find_slack_by_load()
    print("knee finder: all tests passed")


if __name__ == "__main__":
    main()
