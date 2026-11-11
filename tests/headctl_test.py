"""ROS-free regression tests for calibration-profile compatibility helpers."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import struct
import sys
import tempfile
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
    # Branch 2 (J3) carries one expected servo whose reply timed out twice:
    # the shape of a delivery failure with a healthy servo.
    struct.pack_into("<BBBBIIIIIIII", diagnostics,
                     headctl.DIAGNOSTICS_BRANCH_OFFSET + 2 * headctl.BRANCH_RECORD_LENGTH,
                     2, 0, 0x01, 0x00, 1000, 900, 2, 0, 0, 17, 1, 0)
    report = headctl.decode_diagnostics(bytes(diagnostics))
    assert report["storage_state"] == headctl.STORAGE_SUCCEEDED
    assert report["storage_result"] == 0
    assert report["generation"] == 42
    assert report["discovery_reasons"] == [0] * 20
    assert report["last_error_address"] == 0
    assert report["last_error_status"] == 0
    assert len(report["branches"]) == 4
    assert report["branches"][2] == {
        "index": 2, "active": 0, "expected_mask": 0x01, "received_mask": 0x00,
        "requested_ms": 1000, "completed_ms": 900, "timeouts": 2,
        "protocol_errors": 0, "bus_errors": 0, "transmissions": 17,
        "transmission_errors": 1, "deferred": 0,
    }
    assert report["branches"][0]["expected_mask"] == 0
    assert report["branches"][0]["timeouts"] == 0

    # Header, USB counters and boot cause, at the offsets send_diagnostics()
    # in src/main.c writes them.
    struct.pack_into("<BBIII", diagnostics, 0, 7, 9, 3, 4567, 2500)
    struct.pack_into("<III", diagnostics, 158, 11, 222, 33)
    struct.pack_into("<I", diagnostics, 182, 0x10)
    report = headctl.decode_diagnostics(bytes(diagnostics))
    assert (report["state"], report["fault"]) == (7, 9)
    assert report["control_deadline_misses"] == 3
    assert report["last_control_cycle_ms"] == 4567
    assert report["control_max_period_us"] == 2500
    assert (report["usb_dropped_frames"], report["usb_dropped_bytes"],
            report["usb_coalesced_frames"]) == (11, 222, 33)
    assert report["reset_cause"] == 0x10
    assert report["generation"] == 42
    assert headctl.describe_reset_cause(0x10) == "watchdog (0x00000010)"
    assert headctl.describe_reset_cause(0) == "unknown (0x00000000)"

    # NACK results are (int8_t) negative picolibc errno values; magnitudes
    # above 128 wrap to a positive byte and must still decode.
    def nack(result: int) -> bytes:
        return bytes((headctl.CLEAR_FAULT, result & 0xFF)) + bytes(12)
    assert headctl.describe_firmware_error(nack(-1)) == "-EPERM (-1)"
    assert headctl.describe_firmware_error(nack(-22)) == "-EINVAL (-22)"
    assert headctl.describe_firmware_error(nack(-116)) == "-ETIMEDOUT (-116)"
    assert headctl.describe_firmware_error(nack(-140)) == "-ECANCELED (-140)"
    assert headctl.describe_firmware_error(nack(0)) == "accepted"
    assert headctl.describe_firmware_error(b"") == "no reply from the controller"

    # BUS_STATS: transaction ID, 4 branches x 26 counts, 20 missed-reply counts,
    # in the order send_bus_stats() (src/main.c) writes them.
    values = [77]
    for branch in range(4):
        values += [branch * 100 + field for field in range(26)]
    values += [1000 + servo for servo in range(20)]
    values += [2345, 6]
    bus_stats = headctl.decode_bus_stats(struct.pack(f"<{len(values)}I", *values))
    assert bus_stats["branches"][2]["overrun"] == 200
    assert bus_stats["branches"][2]["noise_errors"] == 202
    assert bus_stats["branches"][2]["requests"] == 205
    assert bus_stats["branches"][3]["rx_restart_failures"] == 317
    assert bus_stats["branches"][3]["rx_gaps"] == 318
    assert bus_stats["branches"][3]["dma_rx_errors"] == 320
    assert bus_stats["branches"][3]["bad_shifted"] == 323
    assert bus_stats["missed_replies"][19] == 1019
    assert bus_stats["window_max_period_us"] == 2345
    assert bus_stats["control_deadline_misses"] == 6
    try:
        headctl.decode_bus_stats(bytes(10))
    except RuntimeError:
        pass
    else:
        raise AssertionError("a short BUS_STATS payload was accepted")

    # CONTROL_TIMING: transaction ID, cycle count, last cycle start (u64 us),
    # schedule restarts, 5 period bins, busy-over-half-period cycles, then the
    # window maxima, in the order send_control_timing() (src/main.c) writes them.
    def control_timing(cycles, start_us, restarts, bins, busy_over, maxima):
        return struct.pack("<IIQI5IIIII", 88, cycles, start_us, restarts, *bins,
                           busy_over, *maxima)
    assert len(control_timing(0, 0, 0, [0] * 5, 0, [0] * 3)) == \
        headctl.CONTROL_TIMING_PAYLOAD_LENGTH
    timing = headctl.decode_control_timing(
        control_timing(7, (1 << 33) + 5, 2, [10, 11, 12, 13, 14], 3, [2050, 900, 40]))
    assert timing["control_cycle_count"] == 7
    assert timing["last_control_cycle_start_us"] == (1 << 33) + 5
    assert timing["schedule_restart_count"] == 2
    assert timing["period_bin_count"] == [10, 11, 12, 13, 14]
    assert timing["busy_over_half_period_count"] == 3
    assert timing["window_max_period_us"] == 2050
    assert timing["window_max_busy_us"] == 900
    assert timing["window_max_lock_wait_us"] == 40
    try:
        headctl.decode_control_timing(bytes(10))
    except RuntimeError:
        pass
    else:
        raise AssertionError("a short CONTROL_TIMING payload was accepted")

    # Pass rule: 60 s at exactly 500 Hz with jitter only passes; the cycle
    # counter may wrap between the two readings.
    before = headctl.decode_control_timing(
        control_timing(0xFFFFFF00, 1_000_000, 0, [0, 5, 0, 0, 0], 0, [0] * 3))
    good = headctl.decode_control_timing(
        control_timing((0xFFFFFF00 + 30_000) & 0xFFFFFFFF, 61_000_000, 0,
                       [100, 29_000, 905, 0, 0], 0, [2060, 700, 30]))
    result = headctl.evaluate_control_timing(before, good, 500)
    assert result["cycle_count"] == 30_000
    assert abs(result["rate_hz"] - 500.0) < 1e-9
    assert result["passed"], result
    # One lost cycle fails, even with a correct mean rate.
    lost = headctl.decode_control_timing(
        control_timing((0xFFFFFF00 + 30_000) & 0xFFFFFFFF, 61_000_000, 1,
                       [100, 29_000, 904, 0, 1], 0, [4010, 700, 30]))
    result = headctl.evaluate_control_timing(before, lost, 500)
    assert not result["checks"]["no_lost_cycle"] and result["checks"]["rate"]
    # A rate 0.2 % low fails; busy time of half the period fails.
    slow = headctl.decode_control_timing(
        control_timing((0xFFFFFF00 + 29_940) & 0xFFFFFFFF, 61_000_000, 0,
                       [0, 29_945, 0, 0, 0], 0, [2010, 1000, 30]))
    result = headctl.evaluate_control_timing(before, slow, 500)
    assert not result["checks"]["rate"] and not result["checks"]["busy"]

    # Port selection: an explicit path passes through unchanged; AUTO_PORT
    # needs exactly one CDC1 link (interface 02) and never takes the console.
    assert headctl.resolve_port("/dev/ttyACM7") == "/dev/ttyACM7"
    default_pattern = headctl.HEAD_PORT_PATTERN
    with tempfile.TemporaryDirectory() as directory:
        by_id = Path(directory)
        headctl.HEAD_PORT_PATTERN = str(by_id / "usb-ZEPHYR_Head*-if02")
        (by_id / "usb-ZEPHYR_Head_Teensy_Controller_500100DD041F39D7-if00").touch()
        for expected_count in (0, 2):
            if expected_count == 2:
                (by_id / "usb-ZEPHYR_Head_Teensy_Controller_500100DD041F39D7-if02").touch()
                (by_id / "usb-ZEPHYR_Head_Controller_NO-12V_Bench_00000001-if02").touch()
            try:
                headctl.resolve_port(headctl.AUTO_PORT)
            except OSError:
                pass
            else:
                raise AssertionError(f"AUTO_PORT accepted {expected_count} candidates")
        (by_id / "usb-ZEPHYR_Head_Controller_NO-12V_Bench_00000001-if02").unlink()
        assert headctl.resolve_port(headctl.AUTO_PORT) == str(
            by_id / "usb-ZEPHYR_Head_Teensy_Controller_500100DD041F39D7-if02")
    headctl.HEAD_PORT_PATTERN = default_pattern


if __name__ == "__main__":
    main()
