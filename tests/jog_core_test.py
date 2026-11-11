"""ROS-free tests for the jog panel's seeding, streaming and pose rules."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace


MODULE_PATH = (Path(__file__).parents[1] / "host_ros" / "src" / "head_ros" /
               "head_ros" / "jog_core.py")
SPEC = importlib.util.spec_from_file_location("jog_core_under_test", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
core = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = core
SPEC.loader.exec_module(core)

NAN = float("nan")


def servo(name: str, present_normalized: float, online: bool = True) -> SimpleNamespace:
    return SimpleNamespace(name=name, present_normalized=present_normalized, online=online)


def head(state: int, servos: list, active_servo_mask: int = 0b11,
         boot_session_id: int = 7, fault: int = 0,
         torque_enabled: bool = False) -> SimpleNamespace:
    return SimpleNamespace(state=state, fault=fault, servos=servos,
                           active_servo_mask=active_servo_mask,
                           boot_session_id=boot_session_id,
                           torque_enabled=torque_enabled)


def test_present_positions_fail_closed() -> None:
    ready = head(core.HeadLifecycle.READY, [servo("a", 0.25), servo("b", 0.75)])
    assert core.present_positions_normalized(ready) == {"a": 0.25, "b": 0.75}
    # Only active servos count: an inactive NaN slot must not block.
    masked = head(core.HeadLifecycle.READY, [servo("a", 0.25), servo("b", NAN)],
                  active_servo_mask=0b01)
    assert core.present_positions_normalized(masked) == {"a": 0.25}
    for bad in (NAN, math.inf, -0.01, 1.01):
        blocked = head(core.HeadLifecycle.READY, [servo("a", 0.5), servo("b", bad)])
        try:
            core.present_positions_normalized(blocked)
        except core.NormalizedPositionUnavailable as error:
            assert "b" in str(error)
        else:
            raise AssertionError(f"accepted present_normalized={bad}")
    offline = head(core.HeadLifecycle.READY, [servo("a", 0.5), servo("b", 0.5, online=False)])
    try:
        core.present_positions_normalized(offline)
    except core.NormalizedPositionUnavailable:
        pass
    else:
        raise AssertionError("accepted an offline servo")


def test_enable_blockers() -> None:
    ready = head(core.HeadLifecycle.READY, [servo("a", 0.5), servo("b", 0.5)])
    assert core.enable_blockers(ready, True, True) == []
    assert core.enable_blockers(None, True, True) == ["no /head/state received"]
    blockers = core.enable_blockers(
        head(core.HeadLifecycle.HOMING_REQUIRED, [servo("a", NAN), servo("b", 0.5)], fault=3),
        False, False)
    joined = "; ".join(blockers)
    for expected in ("stale", "HOMING_REQUIRED", "fault 3", "no control lease",
                     "present_normalized is unavailable for a"):
        assert expected in joined, (expected, joined)
    on = head(core.HeadLifecycle.READY, [servo("a", 0.5), servo("b", 0.5)], torque_enabled=True)
    assert "torque may still be on" in "; ".join(core.enable_blockers(on, True, True))


def test_enable_at_present_pose_blockers() -> None:
    session = core.JogSession()
    inside = head(core.HeadLifecycle.HOMING_REQUIRED, [servo("a", 0.0), servo("b", 1.0)])
    session.observe_head(inside, 0.0)
    session.grant_lease(42)
    assert session.enable_at_present_pose_blockers(0.01) == []
    # The homed Enable still wants READY.
    assert "not READY" in "; ".join(session.enable_blockers(0.01))
    # A servo outside its profile range is named with its q, not called missing.
    outside = head(core.HeadLifecycle.HOMING_REQUIRED, [servo("a", -0.02), servo("b", 0.5)])
    session.observe_head(outside, 0.02)
    joined = "; ".join(session.enable_at_present_pose_blockers(0.03))
    assert "outside the profile range: a (q -0.020)" in joined, joined
    assert "unavailable" not in joined, joined
    session.observe_head(head(core.HeadLifecycle.READY, [servo("a", 0.5), servo("b", 0.5)]), 0.04)
    assert "not HOMING_REQUIRED" in "; ".join(session.enable_at_present_pose_blockers(0.05))


def test_session_seeds_from_measured_positions() -> None:
    session = core.JogSession()
    session.observe_head(head(core.HeadLifecycle.READY, [servo("a", 0.2), servo("b", 0.8)]), 0.0)
    session.grant_lease(42)
    assert session.targets is None
    observation = session.observe_head(
        head(core.HeadLifecycle.ENABLED, [servo("a", 0.21), servo("b", 0.79)]), 0.01)
    assert observation.publish_now and not observation.request_disable
    assert session.targets.as_lists() == (["a", "b"], [0.21, 0.79])
    # Later samples never re-seed over the operator's target.
    session.set_target("a", 0.5, 0.02)
    session.observe_head(head(core.HeadLifecycle.ENABLED, [servo("a", 0.3), servo("b", 0.79)]), 0.03)
    assert session.targets.get("a") == 0.5


def test_session_enabled_without_positions_requests_disable() -> None:
    session = core.JogSession()
    session.observe_head(head(core.HeadLifecycle.READY, [servo("a", NAN), servo("b", NAN)]), 0.0)
    session.grant_lease(42)
    observation = session.observe_head(
        head(core.HeadLifecycle.ENABLED, [servo("a", NAN), servo("b", NAN)]), 0.01)
    assert observation.request_disable and not observation.publish_now
    assert session.targets is None
    assert not session.can_jog(0.02)
    # It stays locked, and does not ask again, for the rest of this ENABLED.
    again = session.observe_head(
        head(core.HeadLifecycle.ENABLED, [servo("a", 0.5), servo("b", 0.5)]), 0.02)
    assert not again.request_disable and session.targets is None


def test_session_without_lease_never_streams() -> None:
    session = core.JogSession()
    session.observe_head(head(core.HeadLifecycle.ENABLED, [servo("a", 0.5), servo("b", 0.5)]), 0.0)
    assert session.targets is None and not session.can_jog(0.0)


def test_session_stale_state_stops_streaming() -> None:
    session = core.JogSession()
    session.observe_head(head(core.HeadLifecycle.READY, [servo("a", 0.5), servo("b", 0.5)]), 0.0)
    session.grant_lease(42)
    session.observe_head(head(core.HeadLifecycle.ENABLED, [servo("a", 0.5), servo("b", 0.5)]), 1.0)
    assert session.check_stale(1.0 + core.STATE_FRESH_S / 2) is None
    assert session.targets is not None
    assert session.check_stale(1.0 + core.STATE_FRESH_S) is not None
    assert session.targets is None
    assert session.set_target("a", 0.9, 1.0 + core.STATE_FRESH_S) is None


def test_session_leaving_enabled_and_reboot() -> None:
    session = core.JogSession()
    session.observe_head(head(core.HeadLifecycle.READY, [servo("a", 0.5), servo("b", 0.5)]), 0.0)
    session.grant_lease(42)
    session.observe_head(head(core.HeadLifecycle.ENABLED, [servo("a", 0.5), servo("b", 0.5)]), 0.01)
    session.freeze_motion()
    assert session.targets is not None, "a frozen target keeps streaming until ENABLED ends"
    assert session.set_target("a", 0.9, 0.02) is None
    observation = session.observe_head(
        head(core.HeadLifecycle.FAULT, [servo("a", 0.5), servo("b", 0.5)]), 0.03)
    assert session.targets is None and not session.is_motion_frozen
    assert any("FAULT" in notice for notice in observation.notices)
    assert session.has_lease, "the lease survives a fault so it can be cleared"
    rebooted = session.observe_head(
        head(core.HeadLifecycle.BOOT, [servo("a", 0.5), servo("b", 0.5)], boot_session_id=8), 0.04)
    assert not session.has_lease
    assert any("boot session changed" in notice for notice in rebooted.notices)
    assert not session.can_renew(0.05)


def test_target_vector_and_recall() -> None:
    targets = core.TargetVector({"a": 0.5, "b": 0.5})
    assert targets.step("a", 0.7) == 1.0
    assert targets.step("b", -0.7) == 0.0
    try:
        targets.set("a", NAN)
    except ValueError:
        pass
    else:
        raise AssertionError("accepted a NaN target")
    try:
        targets.set("c", 0.5)
    except KeyError:
        pass
    else:
        raise AssertionError("accepted an unknown joint")
    pose = core.JogPose("smile", {"a": 0.3, "z": 0.9})
    applied, skipped = core.recall_pose(pose, targets)
    assert applied == {"a": 0.3} and skipped == ["z"]
    assert targets.as_lists() == (["a", "b"], [0.3, 0.0])


def test_pose_library_round_trip_and_validation() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / "poses" / "jog.json"
        library = core.JogPoseLibrary(path)
        library.load()
        assert library.poses == []
        library.store("neutral", {"a": 0.5, "b": 0.5}, "rest")
        library.store("smile", {"a": 0.7})
        library.store("neutral", {"a": 0.4, "b": 0.6})
        assert library.names() == ["neutral", "smile"]
        reloaded = core.JogPoseLibrary(path)
        reloaded.load()
        assert reloaded.get("neutral").positions_normalized == {"a": 0.4, "b": 0.6}
        assert not list(path.parent.glob("*.tmp"))
        try:
            library.store("bad", {"a": 1.5})
        except ValueError:
            pass
        else:
            raise AssertionError("stored an out-of-range pose")
        assert reloaded.names() == library.names()
        reloaded.delete("smile")
        assert json.loads(path.read_text())["poses"][0]["name"] == "neutral"
        path.write_text(json.dumps({"version": 99, "poses": []}))
        try:
            core.JogPoseLibrary(path).load()
        except ValueError:
            pass
        else:
            raise AssertionError("loaded an unknown pose file version")


def main() -> None:
    test_present_positions_fail_closed()
    test_enable_blockers()
    test_enable_at_present_pose_blockers()
    test_session_seeds_from_measured_positions()
    test_session_enabled_without_positions_requests_disable()
    test_session_without_lease_never_streams()
    test_session_stale_state_stops_streaming()
    test_session_leaving_enabled_and_reboot()
    test_target_vector_and_recall()
    test_pose_library_round_trip_and_validation()
    print("jog_core_test: all tests passed")


if __name__ == "__main__":
    main()
