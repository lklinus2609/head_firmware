"""ROS-free lab commissioning client for the Teensy head controller."""

from __future__ import annotations

import argparse
import json
import math
import struct
import time
import zlib

import serial

SOF, VERSION, MAX_PAYLOAD = 0xA5, 2, 512
HELLO, HELLO_REPLY = 1, 2
HELLO_SCHEMA_VERSION = 1
CAP_TRANSACTION_IDS = 1 << 0
ACQUIRE, RELEASE, HOME, ENABLE, DISABLE, CLEAR_FAULT = 3, 4, 6, 8, 9, 10
STATE, DIAGNOSTICS, ACK, NACK = 12, 13, 14, 15
STATE_SESSION_ID_OFFSET = 28
GET_INFO, INFO, GET_SLOT, SLOT, STAGE_INFO, STAGE_SLOT, COMMIT = range(17, 24)
PROBE_BRANCH, PROBE_RESULT, LINE_MODE_TEST, UART_TX_METER_TEST, RX_LINE_TEST, RX_LINE_TEST_RESULT, DEBUG_PING, DEBUG_PING_RESULT = 24, 25, 26, 27, 28, 29, 30, 31
RENEW_LEASE = 32
START_PROPRIOCEPTION = 33
START_ROUTING = 34
DEBUG_READ, DEBUG_READ_RESULT = 35, 36
ZERO_HOME = 37

SERIAL_READ_POLL_S = 0.001
SERIAL_INTERFRAME_TIMEOUT_S = 0.05
SERIAL_WRITE_TIMEOUT_S = 0.5
ACK_PAYLOAD_LENGTH = 14
HELLO_REPLY_LENGTH = 52
# Layout: 14-byte prefix, 4 branches x 36, 12 bytes USB counters, then storage,
# generation, boot cause, and one discovery-reason byte per servo.
DIAGNOSTICS_PAYLOAD_LENGTH = 209
# Per-branch health: 4 records of 36 bytes starting after the fixed header.
DIAGNOSTICS_BRANCH_OFFSET = 14
BRANCH_RECORD_LENGTH = 36
DIAGNOSTICS_STORAGE_STATE_OFFSET = 170
# Reserved bytes after storage_state now carry the retried-transfer total.
DIAGNOSTICS_READ_RETRIES_OFFSET = 171
DIAGNOSTICS_STORAGE_RESULT_OFFSET = 174
DIAGNOSTICS_GENERATION_OFFSET = 178
DIAGNOSTICS_DISCOVERY_REASON_OFFSET = 186
DIAGNOSTICS_LAST_ERROR_OFFSET = 206
DISCOVERY_REASON_NAMES = (
    "OK", "INACTIVE", "NO_REPLY", "STATUS_ERROR", "MODEL", "FIRMWARE", "COMMUNICATION_READ",
    "BAUD", "DRIVE_MODE", "PROTOCOL", "TORQUE_ON", "STATUS_LEVEL",
    "SECONDARY_ID", "STARTUP", "RETURN_DELAY", "OPERATING_MODE",
    "VERIFY_READBACK", "LIMITS", "GAINS", "PROFILES", "WATCHDOG",
    "CURRENT_LIMIT")
STORAGE_SUCCEEDED = 3
STORAGE_FAILED = 4

STATE_NAMES = ("BOOT", "RESERVED", "HOMING_REQUIRED", "HOMING",
               "MAINTENANCE_CALIBRATION", "READY", "ENABLED", "FAULT",
               "PROPRIOCEPTION_SETTLING", "PROPRIOCEPTION_HOLD", "ROUTING")
FAULT_NAMES = ("NONE", "CONFIGURATION", "DISCOVERY", "HOMING", "WATCHDOG",
               "BUS", "SERVO", "FAN", "CONTROL_DEADLINE", "TELEMETRY",
               "LEGACY_CURRENT_BUDGET_RESERVED", "BRANCH_CURRENT_BUDGET")
# enum head_torque_state. Reporting this as a boolean conflates OFF_VERIFIED
# with the two shutdown states, which is exactly the distinction needed to tell
# a safe controller from one whose torque-off sweep never confirmed.
TORQUE_NAMES = ("unknown", "off_verified", "on_verified", "shutdown_pending",
                "shutdown_failed")
HEAD_SERVO_COUNT = 20
# STATE payload: the fixed controller header, then one record per servo of
# goal/present/velocity (i32 x3), current + voltage (u16 x2), temperature,
# moving status, hardware error, online (u8 x4) and feedback age (u32).
SERVO_BLOCK_OFFSET = 32
SERVO_RECORD_LENGTH = 24


def encode(message_type: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds Teensy protocol limit")
    body = bytes((VERSION, message_type)) + struct.pack("<H", len(payload)) + payload
    return bytes((SOF,)) + body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


class Device:
    def __init__(self, port: str) -> None:
        self.serial = serial.Serial(port, 115200, timeout=0,
                                    write_timeout=SERIAL_WRITE_TIMEOUT_S)
        self.buffer = bytearray()
        self.last_byte_monotonic_s = 0.0
        self.transaction_id = 0
        self.session_id: int | None = None
        self.configuration_generation: int | None = None

    def close(self) -> None:
        self.serial.close()

    def write(self, data: bytes, timeout_s: float = SERIAL_WRITE_TIMEOUT_S) -> None:
        """Write a complete frame before its absolute output deadline."""
        deadline_monotonic_s = time.monotonic() + timeout_s
        offset = 0
        while offset < len(data):
            remaining_s = deadline_monotonic_s - time.monotonic()
            if remaining_s <= 0:
                raise TimeoutError("serial write deadline exceeded")
            try:
                # pySerial enforces this per call; lowering it to the
                # remaining transaction budget also bounds a blocked call
                # when a caller supplied a shorter deadline.
                if hasattr(self.serial, "write_timeout"):
                    self.serial.write_timeout = min(SERIAL_WRITE_TIMEOUT_S, remaining_s)
                written_length = self.serial.write(data[offset:])
            except Exception as error:
                serial_timeout_error = getattr(serial, "SerialTimeoutException", None)
                if serial_timeout_error is not None and isinstance(error, serial_timeout_error):
                    raise TimeoutError("serial write stalled before the deadline") from error
                raise
            if not isinstance(written_length, int) or written_length < 0:
                raise RuntimeError("serial write returned an invalid byte count")
            if written_length > len(data) - offset:
                raise RuntimeError("serial write exceeded the requested byte count")
            if written_length == 0:
                time.sleep(min(SERIAL_READ_POLL_S, remaining_s))
                continue
            offset += written_length

    def send(self, kind: int, payload: bytes = b"") -> None:
        self.write(encode(kind, payload))

    def send_confirmed(self, kind: int, payload: bytes = b"") -> int:
        self.transaction_id = (self.transaction_id + 1) & 0xFFFFFFFF
        if self.transaction_id == 0:
            self.transaction_id = 1
        self.send(kind, payload + struct.pack("<I", self.transaction_id))
        return self.transaction_id

    def _observe_session(self, observed_session_id: int) -> None:
        if observed_session_id == 0:
            raise RuntimeError("controller reported an invalid zero boot session")
        if self.session_id is None:
            self.session_id = observed_session_id
            return
        if observed_session_id != self.session_id:
            self.session_id = None
            self.configuration_generation = None
            raise RuntimeError("controller rebooted; session/configuration must be reacquired")

    def _observe_frame_session(self, kind: int, payload: bytes) -> None:
        if kind == STATE and len(payload) >= STATE_SESSION_ID_OFFSET + 4:
            self._observe_session(struct.unpack_from("<I", payload, STATE_SESSION_ID_OFFSET)[0])
        elif kind == HELLO_REPLY and len(payload) >= 36:
            self._observe_session(struct.unpack_from("<I", payload, 32)[0])
        elif kind in (ACK, NACK) and len(payload) >= ACK_PAYLOAD_LENGTH:
            # A stale ACK from a previous boot must never become the newly
            # trusted session when the client has not completed HELLO.
            if self.session_id is not None:
                self._observe_session(struct.unpack_from("<I", payload, 10)[0])

    def recv(self, timeout_s: float = 1.0) -> tuple[int, bytes] | None:
        deadline_monotonic_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_monotonic_s:
            data = self.serial.read(self.serial.in_waiting or 1)
            now = time.monotonic()
            if self.buffer and now - self.last_byte_monotonic_s > SERIAL_INTERFRAME_TIMEOUT_S:
                self.buffer.clear()
            if data:
                self.last_byte_monotonic_s = now
                self.buffer.extend(data)
            while self.buffer and self.buffer[0] != SOF:
                del self.buffer[0]
            if len(self.buffer) >= 5:
                length = struct.unpack_from("<H", self.buffer, 3)[0]
                total = length + 9
                if length > MAX_PAYLOAD:
                    del self.buffer[0]
                elif len(self.buffer) >= total:
                    raw, self.buffer = self.buffer[:total], self.buffer[total:]
                    if raw[1] == VERSION and zlib.crc32(raw[1:-4]) & 0xFFFFFFFF == struct.unpack_from("<I", raw, total - 4)[0]:
                        kind, payload = raw[2], bytes(raw[5:-4])
                        self._observe_frame_session(kind, payload)
                        return kind, payload
                    # Reconsider every suffix after the corrupt SOF. This lets
                    # a valid frame embedded after a truncated one survive.
                    self.buffer = bytearray(raw[1:]) + self.buffer
            time.sleep(min(SERIAL_READ_POLL_S,
                           max(0.0, deadline_monotonic_s - time.monotonic())))
        return None

    def ack_matches(self, kind: int, payload: bytes, transaction_id: int) -> bool:
        return (self.session_id is not None and len(payload) >= ACK_PAYLOAD_LENGTH and
                payload[0] == kind and
                struct.unpack_from("<I", payload, 6)[0] == transaction_id and
                struct.unpack_from("<I", payload, 10)[0] == self.session_id)

    def hello(self, timeout_s: float = 1.0) -> dict:
        transaction_id = self.send_confirmed(HELLO)
        deadline_monotonic_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_monotonic_s:
            remaining_s = deadline_monotonic_s - time.monotonic()
            frame = self.recv(remaining_s)
            if frame is None:
                break
            kind, payload = frame
            if kind != HELLO_REPLY or len(payload) != HELLO_REPLY_LENGTH:
                continue
            if struct.unpack_from("<I", payload, 0)[0] != transaction_id:
                continue
            (reply_transaction_id, hello_schema, protocol_version, fw_major,
             fw_minor, fw_patch, hardware_revision, schema_hash, capabilities,
             build_id, hardware_id, session_id, reset_cause, generation,
             control_hz, telemetry_hz, usb_vid, usb_pid) = struct.unpack(
                 "<IHHBBBBIQIIIIIHHHH", payload)
            if (hello_schema != HELLO_SCHEMA_VERSION or
                    protocol_version != VERSION or
                    not (capabilities & CAP_TRANSACTION_IDS)):
                raise RuntimeError("controller HELLO is incompatible with protocol v2")
            if session_id != self.session_id:
                raise RuntimeError("HELLO session changed during negotiation")
            self.configuration_generation = generation
            return {"transaction_id": reply_transaction_id,
                    "hello_schema": hello_schema,
                    "protocol_version": protocol_version,
                    "firmware": (fw_major, fw_minor, fw_patch),
                    "hardware_revision": hardware_revision,
                    "schema_hash": schema_hash,
                    "capabilities": capabilities,
                    "build_id": build_id,
                    "hardware_id": hardware_id,
                    "session_id": session_id,
                    "reset_cause": reset_cause,
                    "generation": generation,
                    "control_hz": control_hz,
                    "telemetry_hz": telemetry_hz,
                    "usb_vid": usb_vid,
                    "usb_pid": usb_pid}
        raise RuntimeError("controller did not return a valid HELLO reply")

    def request(self, kind: int, payload: bytes = b"", timeout_s: float = 1.0) -> tuple[bool, bytes]:
        transaction_id = self.send_confirmed(kind, payload)
        deadline_monotonic_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_monotonic_s:
            frame = self.recv(deadline_monotonic_s - time.monotonic())
            if frame is None:
                break
            response_kind, response = frame
            if response_kind in (ACK, NACK) and self.ack_matches(kind, response, transaction_id):
                return response_kind == ACK and response[1] == 0, response
        return False, b""

    def get(self, request_kind: int, reply_kind: int, payload: bytes = b"", timeout_s: float = 1.0) -> bytes:
        self.send_confirmed(request_kind, payload)
        deadline_monotonic_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_monotonic_s:
            frame = self.recv(deadline_monotonic_s - time.monotonic())
            if frame and frame[0] == reply_kind:
                return frame[1]
        raise RuntimeError("controller did not return requested configuration")


def probe_branch(device: Device, branch: int, baudrate: int) -> list[tuple[int, int, int]]:
    transaction_id = device.send_confirmed(PROBE_BRANCH, struct.pack("<BI", branch, baudrate))
    deadline_monotonic_s = time.monotonic() + 3.0
    accepted = False
    result: bytes | None = None
    while time.monotonic() < deadline_monotonic_s:
        frame = device.recv(deadline_monotonic_s - time.monotonic())
        if frame is None:
            continue
        kind, payload = frame
        if kind == PROBE_RESULT and len(payload) >= 2 and payload[0] == branch:
            result = payload
        elif kind in (ACK, NACK) and device.ack_matches(PROBE_BRANCH, payload, transaction_id):
            if kind != ACK or len(payload) < 2 or payload[1] != 0:
                error, = struct.unpack_from("<b", payload, 1)
                raise RuntimeError(f"probe rejected by firmware (errno {error}); torque may be on, it may be busy, or bus initialization failed")
            accepted = True
        if accepted and result is not None:
            reply_count = result[1]
            if len(result) != 2 + 4 * reply_count:
                raise RuntimeError("malformed probe response")
            return [struct.unpack_from("<BHB", result, 2 + 4 * index)
                    for index in range(reply_count)]
    raise RuntimeError("controller did not return a probe result")

def line_mode_test(device: Device, branch: int, tx: bool) -> None:
    mode = "TX" if tx else "RX"
    print(f"Holding J{branch + 1} in {mode} mode for 10 seconds; measure now.", flush=True)
    transaction_id = device.send_confirmed(LINE_MODE_TEST, bytes((branch, int(tx))))
    deadline_monotonic_s = time.monotonic() + 12.0
    while time.monotonic() < deadline_monotonic_s:
        frame = device.recv(deadline_monotonic_s - time.monotonic())
        if (frame and frame[0] in (ACK, NACK) and
                device.ack_matches(LINE_MODE_TEST, frame[1], transaction_id)):
            if frame[0] != ACK or len(frame[1]) < 2 or frame[1][1] != 0:
                raise RuntimeError("line-mode test rejected: ensure torque is off")
            return
    raise RuntimeError("controller did not complete line-mode test")

def uart_tx_meter_test(device: Device, branch: int) -> None:
    print(f"Sending a torque-safe 1 Mbps zero-byte stream on J{branch + 1} for 10 seconds; measure J{branch + 1} DATA now.", flush=True)
    transaction_id = device.send_confirmed(UART_TX_METER_TEST, bytes((branch,)))
    deadline_monotonic_s = time.monotonic() + 12.0
    while time.monotonic() < deadline_monotonic_s:
        frame = device.recv(deadline_monotonic_s - time.monotonic())
        if (frame and frame[0] in (ACK, NACK) and
                device.ack_matches(UART_TX_METER_TEST, frame[1], transaction_id)):
            if frame[0] != ACK or len(frame[1]) < 2 or frame[1][1] != 0:
                raise RuntimeError("UART meter test rejected: ensure torque is off")
            return
    raise RuntimeError("controller did not complete UART meter test")

def uart_rx_line_test(device: Device, branch: int) -> tuple[int, int]:
    print(f"Listening on J{branch + 1} for 10 seconds. With the servo unplugged, briefly bridge DATA (pin 3) to GND (pin 1) now.", flush=True)
    transaction_id = device.send_confirmed(RX_LINE_TEST, bytes((branch,)))
    deadline_monotonic_s = time.monotonic() + 12.0
    result: tuple[int, int] | None = None
    accepted = False
    while time.monotonic() < deadline_monotonic_s:
        frame = device.recv(deadline_monotonic_s - time.monotonic())
        if not frame:
            continue
        kind, payload = frame
        if kind == RX_LINE_TEST_RESULT and len(payload) == 9 and payload[0] == branch:
            result = struct.unpack_from("<II", payload, 1)
        elif kind in (ACK, NACK) and device.ack_matches(RX_LINE_TEST, payload, transaction_id):
            if kind != ACK or len(payload) < 2 or payload[1] != 0:
                raise RuntimeError("RX line test rejected: ensure torque is off")
            accepted = True
        if accepted and result is not None:
            return result
    raise RuntimeError("controller did not complete RX line test")


def debug_ping(device: Device, branch: int, servo_id: int, baudrate: int) -> dict:
    transaction_id = device.send_confirmed(
        DEBUG_PING, struct.pack("<BBI", branch, servo_id, baudrate))
    deadline_monotonic_s = time.monotonic() + 3.0
    accepted = False
    result: dict | None = None
    while time.monotonic() < deadline_monotonic_s:
        frame = device.recv(deadline_monotonic_s - time.monotonic())
        if frame is None:
            continue
        kind, payload = frame
        if kind == DEBUG_PING_RESULT and len(payload) >= 13 and payload[0] == branch and payload[1] == servo_id:
            request_length, received_length = payload[3], payload[4]
            body_end = 13 + request_length + received_length
            # Firmware may append an 8-byte RX forensics trail after the captured bytes.
            if len(payload) not in (body_end, body_end + 8):
                raise RuntimeError("malformed debug-ping response")
            delay_us, uart_errors = struct.unpack_from("<II", payload, 5)
            result = {"flags": payload[2], "request": payload[13:13 + request_length],
                      "received": payload[13 + request_length:body_end],
                      "first_byte_delay_us": delay_us,
                      "uart_error_flags": uart_errors}
            if len(payload) == body_end + 8:
                stale, rx_disabled, rx_restarts, rx_overflows = struct.unpack_from(
                    "<HHHH", payload, body_end)
                result.update({"stale_bytes": stale, "rx_disabled": rx_disabled,
                               "rx_restart_failures": rx_restarts,
                               "rx_overflows": rx_overflows})
        elif kind in (ACK, NACK) and device.ack_matches(DEBUG_PING, payload, transaction_id):
            if kind != ACK or len(payload) < 2 or payload[1] != 0:
                error, = struct.unpack_from("<b", payload, 1)
                raise RuntimeError(f"debug ping rejected by firmware (errno {error})")
            accepted = True
        if accepted and result is not None:
            return result
    raise RuntimeError("controller did not return a debug-ping result")


def debug_read(device: Device, branch: int, servo_id: int, address: int,
               length: int) -> bytes:
    transaction_id = device.send_confirmed(
        DEBUG_READ, struct.pack("<BBHB", branch, servo_id, address, length))
    deadline_monotonic_s = time.monotonic() + 3.0
    accepted = False
    data: bytes | None = None
    while time.monotonic() < deadline_monotonic_s:
        frame = device.recv(deadline_monotonic_s - time.monotonic())
        if frame is None:
            continue
        kind, payload = frame
        if (kind == DEBUG_READ_RESULT and len(payload) >= 5
                and payload[0] == branch and payload[1] == servo_id):
            got = payload[4]
            data = payload[5:5 + got]
        elif kind in (ACK, NACK) and device.ack_matches(DEBUG_READ, payload, transaction_id):
            if kind != ACK or len(payload) < 2 or payload[1] != 0:
                error, = struct.unpack_from("<b", payload, 1)
                raise RuntimeError(f"debug read rejected by firmware (errno {error})")
            accepted = True
        if accepted and data is not None:
            return data
    raise RuntimeError("controller did not return a debug-read result")


def decode_info(raw: bytes) -> dict:
    version, = struct.unpack_from("<I", raw, 0)
    expected, partial, thermal_start, thermal_full = struct.unpack_from("<BBBB", raw, 4)
    active_mask, = struct.unpack_from("<I", raw, 8)
    return {"version": version, "expected_servo_count": expected,
            "allow_partial_inventory": partial, "thermal_start_c": thermal_start,
            "thermal_full_c": thermal_full, "active_servo_mask": active_mask}


def decode_diagnostics(raw: bytes) -> dict:
    """Decode the storage fields in the existing fixed diagnostics payload."""
    if len(raw) != DIAGNOSTICS_PAYLOAD_LENGTH:
        raise RuntimeError("malformed diagnostics response")
    reasons = raw[DIAGNOSTICS_DISCOVERY_REASON_OFFSET:
                  DIAGNOSTICS_DISCOVERY_REASON_OFFSET + 20]
    branches = []
    for branch in range(4):
        offset = DIAGNOSTICS_BRANCH_OFFSET + branch * BRANCH_RECORD_LENGTH
        (index, active, expected, received, requested_ms, completed_ms, timeouts,
         protocol_errors, bus_errors, transmissions, transmission_errors,
         deferred) = struct.unpack_from("<BBBBIIIIIIII", raw, offset)
        branches.append({
            "index": index, "active": active, "expected_mask": expected,
            "received_mask": received, "requested_ms": requested_ms,
            "completed_ms": completed_ms, "timeouts": timeouts,
            "protocol_errors": protocol_errors, "bus_errors": bus_errors,
            "transmissions": transmissions,
            "transmission_errors": transmission_errors, "deferred": deferred})
    return {
        "branches": branches,
        "storage_state": raw[DIAGNOSTICS_STORAGE_STATE_OFFSET],
        "read_retries": struct.unpack_from("<H", raw, DIAGNOSTICS_READ_RETRIES_OFFSET)[0],
        "storage_result": struct.unpack_from("<i", raw, DIAGNOSTICS_STORAGE_RESULT_OFFSET)[0],
        "generation": struct.unpack_from("<I", raw, DIAGNOSTICS_GENERATION_OFFSET)[0],
        "discovery_reasons": list(reasons),
        "last_error_address": struct.unpack_from("<H", raw, DIAGNOSTICS_LAST_ERROR_OFFSET)[0],
        "last_error_status": raw[DIAGNOSTICS_LAST_ERROR_OFFSET + 2],
    }


def read_profile(device: Device) -> dict:
    profile = decode_info(device.get(GET_INFO, INFO))
    profile["joints"] = [decode_slot(device.get(GET_SLOT, SLOT, bytes((index,))))
                         for index in range(20)]
    return profile


def profile_matches(expected: dict, observed: dict) -> bool:
    if encode_info(expected) != encode_info(observed):
        return False

    def slots_by_index(profile: dict) -> dict[int, dict] | None:
        slots: dict[int, dict] = {}
        for slot in profile.get("joints", []):
            index = int(slot["index"])
            if index < 0 or index >= 20 or index in slots:
                return None
            slots[index] = slot
        return slots

    expected_slots = slots_by_index(expected)
    observed_slots = slots_by_index(observed)
    if expected_slots is None or observed_slots is None:
        return False
    for index in range(20):
        if not (int(expected["active_servo_mask"]) & (1 << index)):
            continue
        if index not in expected_slots or index not in observed_slots:
            return False
        if encode_slot(expected_slots[index]) != encode_slot(observed_slots[index]):
            return False
    return True


def wait_for_profile_persistence(device: Device, expected_generation: int,
                                 timeout_s: float = 30.0) -> None:
    deadline_monotonic_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_monotonic_s:
        frame = device.recv(deadline_monotonic_s - time.monotonic())
        if frame is None:
            break
        if frame[0] != DIAGNOSTICS:
            continue
        diagnostic = decode_diagnostics(frame[1])
        if diagnostic["storage_state"] == STORAGE_FAILED:
            raise RuntimeError(
                f"profile persistence failed (errno {diagnostic['storage_result']})")
        if (diagnostic["storage_state"] == STORAGE_SUCCEEDED and
                diagnostic["generation"] == expected_generation):
            return
    raise RuntimeError(
        f"profile persistence did not complete for generation {expected_generation}")


def encode_info(profile: dict) -> bytes:
    return struct.pack("<IBBBBI", int(profile["version"]), int(profile["expected_servo_count"]),
                       int(profile["allow_partial_inventory"]), int(profile["thermal_start_c"]),
                       int(profile["thermal_full_c"]), int(profile["active_servo_mask"]))


SLOT_FORMAT = "<BBBBiii3fiiIhhiHhHi"
SLOT_FIELDS = ("index", "branch", "id", "reserved", "min_tick", "max_tick", "home_tick",
               "max_position_fraction_per_control_cycle",
               "max_acceleration_fraction_per_control_cycle", "homing_direction",
               "reserved_homing_start_tick", "homing_max_travel_ticks", "homing_timeout_ms",
               "operating_current_ma", "homing_current_ma",
               "homing_following_error_ticks", "homing_persistence_ms")
SLOT_FIELDS = SLOT_FIELDS + ("homing_current_limit_ma",
                             "homing_speed_ticks_per_second",
                             "homing_backoff_ticks")
LEGACY_SLOT_ALIASES = {
    "max_position_fraction_per_control_cycle": "max_step_per_tick",
    "max_acceleration_fraction_per_control_cycle": "max_accel_step_per_tick",
    "reserved_homing_start_tick": "homing_start_tick",
}


def decode_slot(raw: bytes) -> dict:
    return dict(zip(SLOT_FIELDS, struct.unpack(SLOT_FORMAT, raw)))


def encode_slot(slot: dict) -> bytes:
    def slot_value(name: str):
        if name in slot:
            return slot[name]
        alias = LEGACY_SLOT_ALIASES.get(name)
        return slot.get(alias, 0) if alias is not None else 0

    values = [slot_value(name) for name in SLOT_FIELDS]
    values[3] = 0
    for index in (7, 8, 9):
        if not math.isfinite(float(values[index])):
            raise ValueError(f"{SLOT_FIELDS[index]} must be finite")
    return struct.pack(SLOT_FORMAT, *values)


def acquire(device: Device) -> int:
    ok, payload = device.request(ACQUIRE, b"headctl")
    if not ok or len(payload) < 6:
        raise RuntimeError("could not acquire control lease")
    return struct.unpack_from("<I", payload, 2)[0]


def wait_for_zero_home_hold(device: Device, token: int, timeout_s: float = 120.0) -> None:
    """Zero homing settles into PROPRIOCEPTION_HOLD rather than torque-off READY."""
    deadline_monotonic_s = time.monotonic() + timeout_s
    next_lease_renewal_monotonic_s = time.monotonic() + 0.5
    while time.monotonic() < deadline_monotonic_s:
        frame = device.recv(min(0.1, deadline_monotonic_s - time.monotonic()))
        if frame and frame[0] == STATE and frame[1]:
            state = frame[1][0]
            if state == 9:  # HEAD_PROPRIOCEPTION_HOLD
                return
            if state == 7:  # HEAD_FAULT
                fault = frame[1][1] if len(frame[1]) > 1 else 0xFF
                raise RuntimeError(f"zero homing faulted (fault {fault})")
        if time.monotonic() >= next_lease_renewal_monotonic_s:
            if not device.request(RENEW_LEASE, struct.pack("<I", token))[0]:
                raise RuntimeError("control lease expired or was replaced during zero homing")
            next_lease_renewal_monotonic_s = time.monotonic() + 0.5
    raise RuntimeError("controller did not reach the zero-home hold")


def wait_for_home_ready(device: Device, token: int, timeout_s: float = 120.0) -> None:
    deadline_monotonic_s = time.monotonic() + timeout_s
    next_lease_renewal_monotonic_s = time.monotonic() + 0.5
    while time.monotonic() < deadline_monotonic_s:
        frame = device.recv(min(0.1, deadline_monotonic_s - time.monotonic()))
        if frame and frame[0] == STATE and frame[1]:
            state = frame[1][0]
            if state == 5:  # HEAD_READY
                return
            if state == 7:  # HEAD_FAULT
                fault = frame[1][1] if len(frame[1]) > 1 else 0xFF
                raise RuntimeError(f"homing faulted (fault {fault})")
        if time.monotonic() >= next_lease_renewal_monotonic_s:
            if not device.request(RENEW_LEASE, struct.pack("<I", token))[0]:
                raise RuntimeError("control lease expired or was replaced during homing")
            next_lease_renewal_monotonic_s = time.monotonic() + 0.5
    raise RuntimeError("homing did not reach READY before timeout")


def main() -> None:
    parser = argparse.ArgumentParser(description="Teensy tendon-head lab tool")
    parser.add_argument("--port", default="/dev/ttyACM1",
                        help="dedicated head protocol CDC port (default: /dev/ttyACM1)")
    parser.add_argument("command", choices=(
        "status", "diagnostics", "probe", "ping-debug", "read-register", "line-test", "uart-tx-test",
        "uart-rx-test", "export-profile", "upload-profile", "home", "enable",
        "proprioception", "routing", "disable", "clear-fault", "release",
        "zero-home"))
    parser.add_argument("--branch", type=int, choices=range(4), default=0,
                        help="physical branch: J1=0, J2=1, J3=2, J4=3")
    parser.add_argument("--baud", type=int, default=57600,
                        help="temporary read-only probe baud (factory XC330 default: 57600)")
    parser.add_argument("--id", type=int, choices=range(253), default=2,
                        help="target servo ID for ping-debug (default: 2)")
    parser.add_argument("--mode", choices=("rx", "tx"), default="tx",
                        help="line-test direction; TX is held for 10 seconds, then safely released")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0,
                        help="control-table address for read-register")
    parser.add_argument("--length", type=int, default=1,
                        help="byte count for read-register")
    parser.add_argument("--file")
    parser.add_argument("--lease", type=lambda value: int(value, 0), default=0)
    args = parser.parse_args()
    if args.command == "enable" and args.lease == 0:
        parser.error("enable requires --lease from the controller that will stream targets")
    device = Device(args.port)
    acquired_here = False
    token = 0
    try:
        device.hello()
        if args.command == "status":
            while (frame := device.recv(1.0)) is not None:
                if frame[0] == STATE:
                    state, fault, torque, fan_stalled, fan_rpm, uptime_ms = struct.unpack_from(
                        "<BBBBHI", frame[1], 0)
                    state_name = STATE_NAMES[state] if state < len(STATE_NAMES) else f"UNKNOWN({state})"
                    fault_name = FAULT_NAMES[fault] if fault < len(FAULT_NAMES) else f"UNKNOWN({fault})"
                    # Byte 2 is head_state_torque_may_be_on(), a boolean -- NOT
                    # the head_torque_state enum, which sits at offset 16 after
                    # applied_sequence and control_hz. Decoding the boolean
                    # through the enum names inverts the safety reading.
                    payload = frame[1]
                    torque_state = payload[16] if len(payload) > 16 else None
                    torque_name = ("unreported" if torque_state is None else
                                   TORQUE_NAMES[torque_state] if torque_state < len(TORQUE_NAMES)
                                   else f"UNKNOWN({torque_state})")
                    print(f"state={state_name} fault={fault_name} "
                          f"torque_may_be_on={'yes' if torque else 'no'} "
                          f"torque_state={torque_name} "
                          f"fan={'stalled' if fan_stalled else 'ok'} rpm={fan_rpm} "
                          f"uptime_s={uptime_ms / 1000:.3f}")
                    if len(payload) >= 28:
                        confirmations, requested = payload[17], payload[18]
                        attempts, failures = struct.unpack_from("<II", payload, 20)
                        print(f"shutdown requested={'yes' if requested else 'no'} "
                              f"confirmations={confirmations} attempts={attempts} "
                              f"failures={failures}")
                    # Per-servo telemetry. These are the exact fields
                    # active_feedback_fresh() gates home/enable on, so a
                    # rejected motion command explains itself here.
                    try:
                        active_mask = decode_info(device.get(GET_INFO, INFO))["active_servo_mask"]
                    except RuntimeError:
                        active_mask = None
                    for index in range(HEAD_SERVO_COUNT):
                        offset = SERVO_BLOCK_OFFSET + index * SERVO_RECORD_LENGTH
                        if len(payload) < offset + SERVO_RECORD_LENGTH:
                            break
                        if active_mask is not None and not (active_mask >> index) & 1:
                            continue
                        (goal, present, _velocity, current_ma, voltage_mv, temperature,
                         _moving, hardware_error, online, age_ms) = struct.unpack_from(
                            "<iiihHBBBBI", payload, offset)
                        age = "never" if age_ms == 0xFFFFFFFF else f"{age_ms}ms"
                        print(f"  servo {index:2d} (J{index // 5 + 1} id {index}): "
                              f"online={'yes' if online else 'no'} "
                              f"hw_error=0x{hardware_error:02x} feedback={age} "
                              f"pos={present} goal={goal} {current_ma}mA "
                              f"{voltage_mv / 1000:.1f}V {temperature}C")
                    return
            raise RuntimeError("no state frame")
        if args.command == "diagnostics":
            # Broadcast every 100 ms alongside STATE, so no request is needed.
            raw = None
            deadline_monotonic_s = time.monotonic() + 2.0
            while raw is None and time.monotonic() < deadline_monotonic_s:
                frame = device.recv(deadline_monotonic_s - time.monotonic())
                if frame is not None and frame[0] == DIAGNOSTICS:
                    raw = frame[1]
            if raw is None:
                raise RuntimeError("no diagnostics frame")
            report = decode_diagnostics(raw)
            print(f"read_retries={report['read_retries']}")
            print(f"storage_state={report['storage_state']} "
                  f"storage_result={report['storage_result']} "
                  f"generation={report['generation']}")
            # The register whose transfer was rejected last, with the raw
            # Protocol 2.0 error byte: this names the control-table entry a
            # discovery reason only categorizes.
            print(f"last_error address={report['last_error_address']} "
                  f"status=0x{report['last_error_status']:02x}")
            # Branch health. telemetry_timeouts rising while a servo reports
            # hw_error=0x00 is a delivery problem, not a servo problem.
            for branch in report["branches"]:
                if branch["expected_mask"] == 0 and branch["transmissions"] == 0:
                    continue
                print(f"  J{branch['index'] + 1}: expected=0x{branch['expected_mask']:02x} "
                      f"received=0x{branch['received_mask']:02x} "
                      f"timeouts={branch['timeouts']} "
                      f"protocol_errors={branch['protocol_errors']} "
                      f"bus_errors={branch['bus_errors']} "
                      f"tx={branch['transmissions']}/{branch['transmission_errors']}err")
            try:
                active_mask = decode_info(device.get(GET_INFO, INFO))["active_servo_mask"]
            except RuntimeError:
                active_mask = None
            print("discovery reasons:")
            for index, reason in enumerate(report["discovery_reasons"]):
                if active_mask is not None and not (active_mask >> index) & 1:
                    continue
                name = (DISCOVERY_REASON_NAMES[reason] if reason < len(DISCOVERY_REASON_NAMES)
                        else f"UNKNOWN({reason})")
                print(f"  servo {index:2d} (J{index // 5 + 1} id {index}): {name}")
            if active_mask == 0:
                print("  (no servo is active in the provisioned calibration)")
            return
        if args.command == "probe":
            replies = probe_branch(device, args.branch, args.baud)
            connector = f"J{args.branch + 1}"
            if not replies:
                print(f"{connector}: no Dynamixel Protocol 2.0 servo replied")
            for servo_id, model_number, firmware_version in replies:
                print(f"{connector}: id={servo_id} model={model_number} firmware={firmware_version}")
            return
        if args.command == "line-test":
            line_mode_test(device, args.branch, args.mode == "tx")
            print("complete; branch returned to RX mode")
            return
        if args.command == "uart-tx-test":
            uart_tx_meter_test(device, args.branch)
            print("complete; branch returned to RX mode")
            return
        if args.command == "uart-rx-test":
            bytes_seen, error_flags = uart_rx_line_test(device, args.branch)
            print(f"received_bytes={bytes_seen} uart_error_flags=0x{error_flags:08x}")
            return
        if args.command == "ping-debug":
            diagnostic = debug_ping(device, args.branch, args.id, args.baud)
            flag_names = ("RX_ANY", "HEADER_OK", "LENGTH_OK", "STATUS_OK", "ID_OK", "CRC_OK")
            flags = ",".join(name for bit, name in enumerate(flag_names) if diagnostic["flags"] & (1 << bit)) or "none"
            delay = "none" if diagnostic["first_byte_delay_us"] == 0xFFFFFFFF else str(diagnostic["first_byte_delay_us"]) + " us"
            print("request=" + diagnostic["request"].hex(" "))
            print("received=" + (diagnostic["received"].hex(" ") or "(none)"))
            print("flags={} first_byte_delay={} uart_error_flags=0x{:08x}".format(flags, delay, diagnostic["uart_error_flags"]))
            if "stale_bytes" in diagnostic:
                print("stale_bytes={} rx_disabled={} rx_restart_failures={} rx_overflows={}".format(
                    diagnostic["stale_bytes"], diagnostic["rx_disabled"],
                    diagnostic["rx_restart_failures"], diagnostic["rx_overflows"]))
            return
        if args.command == "read-register":
            raw = debug_read(device, args.branch, args.id, args.address, args.length)
            value = int.from_bytes(raw, "little")
            print(f"branch={args.branch} id={args.id} address={args.address} "
                  f"length={args.length}")
            print(f"raw={raw.hex(' ')} value={value} (0x{value:x}) bits={value:#b}")
            return
        if args.command == "export-profile":
            if not args.file:
                parser.error("--file is required")
            profile = read_profile(device)
            with open(args.file, "w", encoding="utf-8") as output:
                json.dump(profile, output, indent=2, sort_keys=True)
            return
        token = args.lease or acquire(device)
        acquired_here = args.lease == 0
        if args.command == "upload-profile":
            if not args.file:
                parser.error("--file is required")
            with open(args.file, encoding="utf-8") as source:
                profile = json.load(source)
            if not device.request(STAGE_INFO, struct.pack("<I", token) + encode_info(profile))[0]:
                raise RuntimeError("profile header rejected")
            active = int(profile["active_servo_mask"])
            for slot in profile["joints"]:
                index = int(slot["index"])
                if not active & (1 << index):
                    continue
                # The lease lasts HEAD_LEASE_DURATION_MS (1000 ms) from each
                # renewal and token_matches() rejects a staged slot the moment
                # it lapses. Renewing every fifth *array position* let up to
                # five staging round trips share one lease -- a 200 ms budget
                # per request that any slower link blows partway through, so
                # the upload failed on a middle slot with no hint that timing
                # rather than slot content was the cause. Renew before each
                # staged slot: one extra round trip per slot costs nothing
                # next to a rejected upload.
                if not device.request(RENEW_LEASE, struct.pack("<I", token))[0]:
                    raise RuntimeError("lease renewal failed during profile upload")
                if not device.request(STAGE_SLOT, struct.pack("<I", token) + encode_slot(slot))[0]:
                    raise RuntimeError(f"slot {index} rejected")
            if not device.request(RENEW_LEASE, struct.pack("<I", token))[0]:
                raise RuntimeError("lease renewal failed before profile commit")
            if not device.request(COMMIT, struct.pack("<I", token))[0]:
                raise RuntimeError("profile commit rejected (NVS may be unavailable)")
            print("profile accepted and queued for persistence", flush=True)
            if device.configuration_generation is None:
                raise RuntimeError("cannot verify profile persistence without a session generation")
            expected_generation = (device.configuration_generation + 1) & 0xFFFFFFFF
            wait_for_profile_persistence(device, expected_generation)
            stored_profile = read_profile(device)
            if not profile_matches(profile, stored_profile):
                raise RuntimeError("profile persistence completed but readback did not match")
            print(f"profile committed at generation {expected_generation}")
            return
        message = {"home": HOME, "enable": ENABLE,
                   "proprioception": START_PROPRIOCEPTION,
                   "routing": START_ROUTING,
                   "disable": DISABLE, "clear-fault": CLEAR_FAULT,
                   "release": RELEASE,
                   "zero-home": ZERO_HOME}[args.command]
        if not device.request(message, struct.pack("<I", token))[0]:
            raise RuntimeError(f"{args.command} rejected")
        if args.command == "home":
            wait_for_home_ready(device, token)
            acquired_here = False
            print(f"homing completed; lease 0x{token:08x} will expire unless renewed")
        elif args.command == "zero-home":
            # Ends holding at the datum with torque on, so the lease has to be
            # renewed for as long as the hold lasts.
            wait_for_zero_home_hold(device, token)
            print("holding at encoder zero; press Ctrl-C to disable")
            try:
                while True:
                    if not device.request(RENEW_LEASE, struct.pack("<I", token))[0]:
                        raise RuntimeError("lease renewal failed during zero-home hold")
                    time.sleep(0.4)
            except KeyboardInterrupt:
                if not device.request(DISABLE, struct.pack("<I", token))[0]:
                    raise RuntimeError("zero-home stop/disable was rejected")
                print("zero-home hold stopped; torque-disable requested")
        elif args.command in ("proprioception", "routing"):
            label = ("proprioception hold" if args.command == "proprioception"
                     else "routing hold at 0 degrees")
            print(f"{label} started; press Ctrl-C to disable")
            try:
                while True:
                    if not device.request(RENEW_LEASE, struct.pack("<I", token))[0]:
                        raise RuntimeError(f"lease renewal failed during {label}")
                    time.sleep(0.4)
            except KeyboardInterrupt:
                if not device.request(DISABLE, struct.pack("<I", token))[0]:
                    raise RuntimeError(f"{label} stop/disable was rejected")
                print(f"{label} stopped; torque-disable requested")
        else:
            print("accepted")
    finally:
        if acquired_here and token:
            try:
                device.request(RELEASE, struct.pack("<I", token))
            except (RuntimeError, TimeoutError):
                pass
        device.close()


if __name__ == "__main__":
    main()
