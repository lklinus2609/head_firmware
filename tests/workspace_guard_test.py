"""Regression tests for the face workspace guard (pure logic, no hardware)."""

from __future__ import annotations

from pathlib import Path
import sys

FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))

import workspace_guard as wg  # noqa: E402

NEUTRAL = {8: 2300, 11: 2700, 14: 3100, 9: 1750, 15: 450, 19: 2750}


def test_upper_half_drives_up_and_diag_down_follows() -> None:
    guard = wg.WorkspaceGuard(dict(NEUTRAL))
    cmd = guard.side_command("right", 1.0, 1.0)
    assert cmd.drivers == {14: 3100 + 700, 11: 2700 + 580} and cmd.followers == (8,)
    cmd = guard.side_command("left", 0.5, 0.25)        # 15 and 19 pull toward lower ticks
    assert cmd.drivers == {15: 450 - 290, 19: 2750 - 175} and cmd.followers == (9,)
    cmd = guard.side_command("right", 0.4, 0.0)        # diag alone: up and down follow
    assert cmd.drivers == {11: 2700 + 232} and set(cmd.followers) == {8, 14}


def test_lower_half_drives_down_only_and_never_with_up_or_diag() -> None:
    guard = wg.WorkspaceGuard(dict(NEUTRAL))
    cmd = guard.side_command("right", 1.0, -1.0)       # d is ignored below the middle
    assert cmd.drivers == {8: 2300 + wg.PULL[8] * 300} and set(cmd.followers) == {11, 14}
    for d in (0.0, 0.3, 1.0):
        for v in (-1.0, -0.5, -0.01, 0.0, 0.01, 0.5, 1.0):
            drivers = guard.side_command("left", d, v).drivers
            assert not (9 in drivers and (15 in drivers or 19 in drivers)), (d, v, drivers)


def test_actions_are_clamped_and_never_leave_the_ranges() -> None:
    guard = wg.WorkspaceGuard(dict(NEUTRAL))
    cmd = guard.side_command("right", 7.0, 9.0)
    assert cmd.drivers == {14: 3800, 11: 3280}
    cmd = guard.side_command("right", -3.0, -9.0)
    assert cmd.drivers == {8: 2300 + wg.PULL[8] * 300}
    assert guard.jaw_target(-1.0) == 4000 and guard.jaw_target(2.0) == 3880
    assert guard.jaw_target(0.5) == 3940
    for index in NEUTRAL:
        low, high = guard.line_bounds(index)
        assert high - low in (700, 580, 300)
    targets, followers = guard.command({"right": (1.0, 1.0), "left": (0.0, -1.0)}, 1.0)
    assert targets == {7: 3880, 14: 3800, 11: 3280, 9: 1750 + wg.PULL[9] * 300}
    assert set(followers) == {8, 15, 19}


def test_speed_limit() -> None:
    guard = wg.WorkspaceGuard(dict(NEUTRAL), line_speed_tps=1000.0, jaw_speed_tps=100.0)
    first = guard.limit_speed({14: 3100, 7: 4000}, 0.02)
    assert first == {14: 3100, 7: 4000}                # nothing sent before: as asked
    second = guard.limit_speed({14: 3800, 7: 3880}, 0.02)
    assert second == {14: 3120, 7: 3998}               # 1000 and 100 ticks/s x 20 ms


def test_follower_and_live_checks() -> None:
    assert wg.follow_step(0, 1) > 0                     # slack: take in
    assert wg.follow_step(80, 1) < 0                    # loaded: pay out
    assert wg.follow_step(36, 1) == 0                   # at the preload: hold
    # 15/19 tighten toward lower ticks: loaded pays out upward, slack takes in downward.
    assert wg.follow_step(-80, -1) > 0 and wg.follow_step(0, -1) < 0
    checks = wg.LiveChecks()
    assert checks.observe({14: 120, 11: 90}, (14, 11)) is None
    assert "line abort" in checks.observe({14: 285}, (14,))
    checks = wg.LiveChecks()
    for load in (100, 120, 40, 30):
        reason = checks.observe({19: -load}, (19,))
    assert reason is None                               # two low samples: not yet
    assert "magnet" in checks.observe({19: -20}, (19,))
    assert wg.LiveChecks().observe({7: 500}, (7,)) is None   # the jaw has its own guard
    checks = wg.LiveChecks()                            # let go on purpose: no stop
    for load, pulled in ((100, 500), (140, 700), (90, 600), (60, 450), (40, 300), (30, 200)):
        assert checks.observe({14: load}, (14,), {14: pulled}) is None, (load, pulled)
    checks = wg.LiveChecks()                            # 19 held, 15 pulls in: load moves, no stop
    for load, diag in ((117, 0), (100, 100), (80, 300), (64, 580), (60, 580), (60, 580)):
        drivers = (19, 15) if diag else (19,)
        assert checks.observe({19: -load, 15: -40}, drivers, {19: 700, 15: diag}) is None, (load, diag)
    checks = wg.LiveChecks()                            # held still while the load falls: stop
    reasons = [checks.observe({14: load}, (14,), {14: 700}) for load in (140, 60, 50, 40)]
    assert reasons[:3] == [None, None, None] and "magnet" in reasons[3]
    # A fast move: the current is the lag (run 200042_face_frames, line 9:
    # 85 mA at 32 ticks behind, then 23 mA once there). No stop.
    checks = wg.LiveChecks()
    for load, lag in ((60, -25), (85, -32), (70, -20), (30, -6), (23, -2), (23, -1), (23, 0), (22, 0)):
        assert checks.observe({9: -load}, (9,), {9: 150}, {9: lag}) is None, (load, lag)
    # A real drop while caught up still stops the run.
    reasons = [checks.observe({9: -load}, (9,), {9: 150}, {9: 0}) for load in (90, 30, 25, 20)]
    assert "magnet" in reasons[3], reasons
    try:
        wg.WorkspaceGuard({8: 1})
    except ValueError as error:
        assert "no session neutral" in str(error)
    else:
        raise AssertionError("missing neutrals were accepted")


def test_neutrals_follow_the_jaw() -> None:
    opened = {**NEUTRAL, 15: 450 + 160, 14: 3100 - 50}
    guard = wg.WorkspaceGuard(dict(NEUTRAL), neutral_open=opened)
    assert guard.neutral_at(15, 0.0) == 450 and guard.neutral_at(15, 1.0) == 610
    assert guard.neutral_at(15, 0.5) == 530 and guard.neutral_at(15, 9.0) == 610
    cmd = guard.side_command("left", 1.0, 0.0, 1.0)          # diag alone, jaw open
    assert cmd.drivers == {15: 610 - 580}
    targets, _ = guard.command({"right": (0.0, 1.0)}, 0.0, jaw_tick=3880)
    assert targets[14] == 3050 + 700 and targets[7] == 4000   # lines follow the jaw as it is
    targets, _ = guard.command({"right": (0.0, 1.0)}, 1.0)
    assert targets[14] == 3050 + 700 and targets[7] == 3880   # no jaw tick: its target
    assert guard.jaw_fraction(3940) == 0.5 and guard.jaw_fraction(4100) == 0.0
    assert guard.line_bounds(15) == (-130, 610) and guard.line_bounds(14) == (3050, 3800)
    assert guard.follower_bounds(8, 0.0, 600) == ((2300 - 600, 2300 + 300) if wg.PULL[8] > 0
                                                  else (2300 - 300, 2300 + 600))
    assert guard.follower_bounds(19, 0.0, 600) == (2750 - 700, 2750 + 600)
    try:
        wg.WorkspaceGuard(dict(NEUTRAL), neutral_open={8: 1})
    except ValueError as error:
        assert "open-jaw" in str(error)
    else:
        raise AssertionError("missing open-jaw neutrals were accepted")


def test_upper_face_commands_and_checks() -> None:
    upper = {10: 2944, 12: 1738, 16: 4017, 17: 2495}
    guard = wg.WorkspaceGuard(dict(NEUTRAL), upper_neutral=dict(upper),
                              upper_ranges={"brow": 350, "oculi": 550})
    targets, drivers = guard.upper_command({"brow": (1.0, 0.5), "oculi": (0.0, 1.0)})
    assert targets == {10: 2944 - 350, 16: 4017 + 175, 12: 1738, 17: 2495 - 550}
    assert set(drivers) == {10, 16, 17}
    checks = wg.LiveChecks()
    assert "line abort" in checks.observe({12: 280}, (12,))
    checks = wg.LiveChecks()
    for load in (100, 110, 40, 35, 30):
        reason = checks.observe({17: -load}, (17,), {17: 550})
    assert reason is not None and "magnet" in reason


def main() -> None:
    test_upper_half_drives_up_and_diag_down_follows()
    test_lower_half_drives_down_only_and_never_with_up_or_diag()
    test_actions_are_clamped_and_never_leave_the_ranges()
    test_speed_limit()
    test_follower_and_live_checks()
    test_neutrals_follow_the_jaw()
    test_upper_face_commands_and_checks()
    print("workspace guard: all tests passed")


if __name__ == "__main__":
    main()
