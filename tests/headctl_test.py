"""ROS-free regression tests for calibration-profile compatibility helpers."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import struct
import sys
import time
from types import SimpleNamespace


sys.modules.setdefault("serial", SimpleNamespace(Serial=object))
MODULE_PATH = (Path(__file__).parents[1] / "host_ros" / "src" / "head_ros" /
               "head_ros" / "headctl.py")
SPEC = importlib.util.spec_from_file_location("headctl_under_test", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
headctl = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(headctl)


def valid_slot() -> dict:
    return {
        "index": 0,
        "branch": 0,
        "id": 0,
        "min_tick": 0,
        "max_tick": 4095,
        "home_tick": 2048,
        "max_position_fraction_per_control_cycle": 0.005,
        "max_acceleration_fraction_per_control_cycle": 0.001,
        "homing_direction": 1.0,
        "reserved_homing_start_tick": 0,
        "homing_max_travel_ticks": 100,
        "homing_timeout_ms": 1000,
        "operating_current_ma": 100,
        "homing_current_ma": 50,
        "homing_following_error_ticks": 0,
        "homing_persistence_ms": 10,
        "homing_current_limit_ma": 75,
        "homing_speed_ticks_per_second": 500,
        "homing_backoff_ticks": 5,
    }


def main() -> None:
    assert headctl.START_ROUTING == 34
    assert headctl.STATE_NAMES[10] == "ROUTING"
    current = valid_slot()
    legacy = valid_slot()
    legacy["max_step_per_tick"] = legacy.pop(
        "max_position_fraction_per_control_cycle")
    legacy["max_accel_step_per_tick"] = legacy.pop(
        "max_acceleration_fraction_per_control_cycle")
    legacy["homing_start_tick"] = legacy.pop("reserved_homing_start_tick")
    assert headctl.encode_slot(current) == headctl.encode_slot(legacy)

    invalid = valid_slot()
    invalid["max_position_fraction_per_control_cycle"] = math.nan
    try:
        headctl.encode_slot(invalid)
    except ValueError:
        pass
    else:
        raise AssertionError("NaN profile value was accepted")

    # A continuous telemetry stream must not extend a transaction's deadline.
    device = object.__new__(headctl.Device)
    device.session_id = 0x12345678
    device.transaction_id = 0
    device.send_confirmed = lambda kind, payload=b"": 7
    receive_timeouts = []

    def telemetry_only(timeout_s):
        receive_timeouts.append(timeout_s)
        time.sleep(min(timeout_s, 0.001))
        return (headctl.STATE, b"\0" * 4)

    device.recv = telemetry_only
    assert device.request(headctl.RENEW_LEASE, timeout_s=0.01) == (False, b"")
    assert len(receive_timeouts) > 1
    assert receive_timeouts[-1] < receive_timeouts[0]

    ack = bytes((headctl.RENEW_LEASE, 0)) + struct.pack(
        "<III", 0, 7, device.session_id)
    assert device.ack_matches(headctl.RENEW_LEASE, ack, 7)
    wrong_session_ack = ack[:-4] + struct.pack("<I", 0x87654321)
    assert not device.ack_matches(headctl.RENEW_LEASE, wrong_session_ack, 7)
    assert not device.ack_matches(headctl.RENEW_LEASE, ack[:10], 7)

    class FrameSerial:
        def __init__(self, frame):
            self.input = bytearray(frame)

        @property
        def in_waiting(self):
            return len(self.input)

        def read(self, length):
            output = bytes(self.input[:length])
            del self.input[:length]
            return output

    state_payload = bytearray(32)
    state_payload[0] = 5  # HEAD_READY
    struct.pack_into("<I", state_payload, headctl.STATE_SESSION_ID_OFFSET,
                     device.session_id)
    device.buffer = bytearray()
    device.last_byte_monotonic_s = 0.0
    device.recv = headctl.Device.recv.__get__(device, headctl.Device)
    device.serial = FrameSerial(headctl.encode(headctl.STATE, bytes(state_payload)))
    assert device.recv(0.1)[0] == headctl.STATE

    rebooted_payload = bytearray(state_payload)
    struct.pack_into("<I", rebooted_payload, headctl.STATE_SESSION_ID_OFFSET,
                     0x87654321)
    device.serial = FrameSerial(headctl.encode(headctl.STATE, bytes(rebooted_payload)))
    try:
        device.recv(0.1)
    except RuntimeError as error:
        assert "rebooted" in str(error)
    else:
        raise AssertionError("changed controller session was accepted")
    assert device.session_id is None

    class PartialSerial:
        def __init__(self):
            self.output = bytearray()

        def write(self, data):
            chunk = bytes(data[:2])
            self.output.extend(chunk)
            return len(chunk)

    device.serial = PartialSerial()
    expected = b"partial frame output"
    device.write(expected, timeout_s=0.1)
    assert bytes(device.serial.output) == expected

    class StalledSerial:
        def write(self, data):
            return 0

    device.serial = StalledSerial()
    try:
        device.write(b"stalled", timeout_s=0.003)
    except TimeoutError:
        pass
    else:
        raise AssertionError("stalled serial output was not bounded")

    active_profile = {
        "version": 4,
        "expected_servo_count": 1,
        "allow_partial_inventory": 1,
        "thermal_start_c": 45,
        "thermal_full_c": 60,
        "active_servo_mask": 1,
        "joints": [valid_slot()],
    }
    stored_profile = dict(active_profile)
    stored_profile["joints"] = []
    for index in range(20):
        slot = valid_slot()
        slot["index"] = index
        slot["branch"] = index // 5
        slot["id"] = index
        stored_profile["joints"].append(slot)
    assert headctl.profile_matches(active_profile, stored_profile)
    duplicate_profile = dict(active_profile)
    duplicate_profile["joints"] = [valid_slot(), valid_slot()]
    assert not headctl.profile_matches(duplicate_profile, stored_profile)

    diagnostics = bytearray(headctl.DIAGNOSTICS_PAYLOAD_LENGTH)
    diagnostics[headctl.DIAGNOSTICS_STORAGE_STATE_OFFSET] = headctl.STORAGE_SUCCEEDED
    struct.pack_into("<iI", diagnostics, headctl.DIAGNOSTICS_STORAGE_RESULT_OFFSET, 0, 42)
    assert headctl.decode_diagnostics(bytes(diagnostics)) == {
        "storage_state": headctl.STORAGE_SUCCEEDED,
        "storage_result": 0,
        "generation": 42,
    }


if __name__ == "__main__":
    main()
