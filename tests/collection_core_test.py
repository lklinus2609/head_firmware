"""ROS-free tests for the proprioception collection state and storage."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile


MODULE_PATH = (Path(__file__).parents[1] / "host_ros" / "src" / "head_ros" /
               "head_ros" / "collection_core.py")
SPEC = importlib.util.spec_from_file_location("collection_core_under_test", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
core = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = core
SPEC.loader.exec_module(core)


def main() -> None:
    state = core.CollectionState()
    for phase in (
        core.SessionPhase.ACQUIRING,
        core.SessionPhase.HOMING,
        core.SessionPhase.ENTERING_HOLD,
        core.SessionPhase.WARMING_UP,
        core.SessionPhase.ARMED,
    ):
        state.transition(phase, phase.name.lower())
    assert state.can_start_recording
    state.transition(core.SessionPhase.RECORDING, "recording")
    assert state.can_stop_recording
    state.transition(core.SessionPhase.STOPPING, "stopping")
    state.transition(core.SessionPhase.SAVED, "saved")

    try:
        state.transition(core.SessionPhase.RECORDING, "invalid")
    except ValueError:
        pass
    else:
        raise AssertionError("invalid transition was accepted")

    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary) / "raw"
        writer = core.SessionWriter(root, "session_test", {"operator_id": "tester"})
        writer.append_telemetry({"mcu_uptime_ms": 10, "servos": []}, True)
        writer.append_event({"kind": "RECORDING_START", "mcu_uptime_ms": 10})
        result = writer.finalize("complete", validation={"valid": True})
        assert result == root / "session_test"
        assert not (root / ".partial" / "session_test").exists()
        manifest = json.loads((result / "session.json").read_text())
        assert manifest["status"] == "complete"
        assert manifest["samples_received"] == 1
        assert (result / "checksums.json").is_file()


if __name__ == "__main__":
    main()
