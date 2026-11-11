"""ROS-free lab commissioning client for the Teensy head controller."""

from __future__ import annotations

import argparse
import fcntl
import glob
import json
import math
import struct
import termios
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
START_TUNING, TUNING_STEP = 38, 39
CAP_TUNING = 1 << 10
GET_BLACK_BOX, BLACK_BOX = 40, 41
CAP_BLACK_BOX = 1 << 11
GET_BUS_STATS, BUS_STATS = 42, 43
CAP_BUS_STATS = 1 << 12
ENABLE_AT_PRESENT_POSE = 44
CAP_ENABLE_AT_PRESENT_POSE = 1 << 13
GET_POSITION_RANGE, POSITION_RANGE = 45, 46
CAP_POSITION_RANGE = 1 << 14
# STATE byte that send_state() in src/main.c fills with
# runtime.position_range_generation (a reserved zero byte before 2026-10-08).
STATE_POSITION_RANGE_GENERATION_OFFSET = 19
POSITION_RANGE_PAYLOAD_LENGTH = 16 + 20 * 8
# send_state() in src/main.c sends PRESENT_PWM right after every STATE: the
# STATE's MCU time (u32 ms) and each servo's signed Present PWM(124) register.
PRESENT_PWM = 47
CAP_PRESENT_PWM = 1 << 15
PRESENT_PWM_PAYLOAD_LENGTH = 4 + 20 * 2
# Present PWM(124) unit, XC330-T181 datasheet (hardware/datasheets/xc330-t181.pdf).
PRESENT_PWM_PERCENT_PER_UNIT = 0.113
# STATE u32 that send_state() fills with k_uptime_get_32() when it builds the sample.
STATE_MCU_TIME_OFFSET = 6
# send_control_timing() in src/main.c: transaction ID, cycle count, last cycle
# start (u64 us), schedule restarts, five period bins, busy-over-half-period
# cycles, then the window maxima of period, busy time and head_lock wait.
GET_CONTROL_TIMING, CONTROL_TIMING = 48, 49
CAP_CONTROL_TIMING = 1 << 16
CONTROL_PERIOD_BIN_UPPER_US = (1900, 2100, 2500, 4000)
CONTROL_TIMING_PAYLOAD_LENGTH = 56
# Pass rule for a validated control rate (operator decision 2026-10-29), over
# one steady-state window: mean rate within 0.1 % of the advertised rate, no
# lost cycle (no schedule restart, no period of 4000 us or more), and busy
# time under half the period.
CONTROL_RATE_TOLERANCE_FRACTION = 0.001
CONTROL_LOST_CYCLE_PERIOD_US = 4000
# send_bus_stats() in src/main.c: per branch, struct head_board_error_counts
# (board.h) then struct head_dxl_bus_stats (dxl.h), in declaration order.
BUS_STATS_BRANCH_FIELDS = (
    "overrun", "framing_errors", "noise_errors", "ring_overflows", "dma_starvations",
    "requests", "complete", "silent_timeouts", "partial_timeouts",
    "received_bytes", "drained_bytes", "resync_bytes", "bad_lengths", "bad_packets",
    "unknown_ids", "servo_errors", "write_failures", "rx_restart_failures",
    "rx_gaps", "rx_overlaps", "dma_rx_errors", "dma_tx_errors",
    "bad_in_place", "bad_shifted", "dma_halt_samples", "dma_rx_last_error_status")
# enum head_black_box_phase in src/main.c: where the control thread was.
BLACK_BOX_PHASE_NAMES = ("idle (sleeping between cycles)", "locked (top of cycle)",
                         "torque-on preparation", "reading telemetry",
                         "homing step", "control step", "writing goals",
                         "starting telemetry", "torque-off sweep", "feeding watchdog")
# eDMA ES fields (DMA_ES_*_MASK in MIMXRT1062.h); ERRCHN is bits 8-12.
DMA_ERROR_STATUS_BITS = ((0, "destination bus error"), (1, "source bus error"),
                         (2, "scatter/gather configuration error"),
                         (3, "NBYTES/CITER configuration error"),
                         (4, "destination offset error"), (5, "destination address error"),
                         (6, "source offset error"), (7, "source address error"),
                         (14, "channel priority error"), (15, "group priority error"),
                         (16, "transfer cancelled"), (31, "an error is latched (VLD)"))
# eDMA channels from the LPUART dmas properties in the board devicetree.
DMA_CHANNEL_NAMES = {1: "J2 TX", 2: "J2 RX", 5: "J3 TX", 6: "J3 RX",
                     9: "J4 TX", 10: "J4 RX", 11: "J1 TX", 12: "J1 RX"}
# Zephyr hwinfo RESET_* bits (include/zephyr/drivers/hwinfo.h).
RESET_CAUSE_NAMES = {0: "pin", 1: "software", 2: "brownout", 3: "power-on",
                     4: "watchdog", 5: "debug", 6: "security", 7: "low-power wake",
                     8: "CPU lockup", 9: "parity", 10: "PLL", 11: "clock",
                     12: "hardware", 13: "user", 14: "temperature"}

SERIAL_READ_POLL_S = 0.001
SERIAL_INTERFRAME_TIMEOUT_S = 0.05
SERIAL_WRITE_TIMEOUT_S = 0.5
# A NACK carries a negative picolibc errno as int8_t, so magnitudes above 128
# wrap (-ECANCELED, 140, arrives as 0x74); 256 - byte recovers both forms.
# Values: modules/lib/picolibc/newlib/libc/include/sys/errno.h.
FIRMWARE_ERRNO_NAMES = {1: "EPERM", 2: "ENOENT", 5: "EIO", 11: "EAGAIN", 16: "EBUSY",
                        19: "ENODEV", 22: "EINVAL", 28: "ENOSPC", 34: "ERANGE",
                        71: "EPROTO", 116: "ETIMEDOUT", 119: "EINPROGRESS",
                        134: "ENOTSUP", 139: "EOVERFLOW", 140: "ECANCELED"}
# CDC1, the head protocol, is USB interface 2 of every head image (CDC0, the
# console, is interface 0). udev 60-serial.rules links it as
# /dev/serial/by-id/usb-<manufacturer>_<product>_<serial>-if02, and the link
# follows the controller when a reset moves it to a new ttyACM number.
AUTO_PORT = "auto"
HEAD_PORT_PATTERN = "/dev/serial/by-id/usb-ZEPHYR_Head*-if02"
# HEAD_BRANCH_DEFAULT_BAUD in include/board.h. A 254-ID probe sweep at 57600
# holds the firmware lock past its 1 s watchdog (docs/PROBE_WATCHDOG_LIMIT.md).
BRANCH_DEFAULT_BAUD = 1_000_000
ACK_PAYLOAD_LENGTH = 14
HELLO_REPLY_LENGTH = 52
# Layout: 14-byte prefix, 4 branches x 36, 12 bytes USB counters, then storage,
# generation, boot cause, and one discovery-reason byte per servo.
DIAGNOSTICS_PAYLOAD_LENGTH = 209
# Per-branch health: 4 records of 36 bytes starting after the fixed header.
DIAGNOSTICS_BRANCH_OFFSET = 14
BRANCH_RECORD_LENGTH = 36
DIAGNOSTICS_USB_COUNTERS_OFFSET = 158
DIAGNOSTICS_STORAGE_STATE_OFFSET = 170
# Reserved bytes after storage_state now carry the retried-transfer total.
DIAGNOSTICS_READ_RETRIES_OFFSET = 171
# The third reserved byte: the last preparation step that failed for good,
# (phase << 5) | servo index, servo 31 = none (firmware since 2026-11-10; 0 =
# none since boot, and always 0 from older images).
DIAGNOSTICS_PREPARATION_FAILURE_OFFSET = 173
PREPARATION_PHASE_NAMES = ("torque-off sweep", "torque-off check", "bus watchdog off",
                           "voltage check", "profile accel/velocity", "current limit",
                           "bus watchdog", "seed goal + torque on")
DIAGNOSTICS_STORAGE_RESULT_OFFSET = 174
DIAGNOSTICS_GENERATION_OFFSET = 178
DIAGNOSTICS_RESET_CAUSE_OFFSET = 182
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
               "PROPRIOCEPTION_SETTLING", "PROPRIOCEPTION_HOLD", "ROUTING",
               "TUNING")
FAULT_NAMES = ("NONE", "CONFIGURATION", "DISCOVERY", "HOMING", "WATCHDOG",
               "BUS", "SERVO", "FAN", "CONTROL_DEADLINE", "TELEMETRY",
               "LEGACY_CURRENT_BUDGET_RESERVED", "BRANCH_CURRENT_BUDGET")
# enum head_torque_state. Reporting this as a boolean conflates OFF_VERIFIED
# with the two shutdown states, which is exactly the distinction needed to tell
# a safe controller from one whose torque-off sweep never confirmed.
TORQUE_NAMES = ("unknown", "off_verified", "on_verified", "shutdown_pending",
                "shutdown_failed")
HEAD_SERVO_COUNT = 20
# The CLEAR_FAULT reply waits for a new discovery pass (discovery_thread in
# src/main.c), which allows each active servo 500 ms.
CLEAR_FAULT_REPLY_TIMEOUT_S = HEAD_SERVO_COUNT * 0.5 + 0.5
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


def resolve_port(port: str) -> str:
    """Return the CDC1 path; AUTO_PORT finds the one connected head controller."""
    if port != AUTO_PORT:
        return port
    matches = sorted(glob.glob(HEAD_PORT_PATTERN))
    if len(matches) != 1:
        raise OSError(f"expected one head controller at {HEAD_PORT_PATTERN}, found "
                      f"{', '.join(matches) or 'none'}; pass --port explicitly")
    return matches[0]


def open_head_serial(port: str, baudrate: int, **settings) -> serial.Serial:
    """Open CDC1 for this process only.

    exclusive=True takes pySerial's flock; TIOCEXCL also refuses programs that
    do not use flock. Best effort, as in bench/dxl_bench/bus.py: the kernel
    drops the flag when this descriptor closes.
    """
    device = serial.Serial(resolve_port(port), baudrate, exclusive=True, **settings)
    try:
        fcntl.ioctl(device.fileno(), termios.TIOCEXCL)
    except OSError:
        pass
    return device


def describe_reset_cause(reset_cause: int) -> str:
    names = [name for bit, name in RESET_CAUSE_NAMES.items() if reset_cause & (1 << bit)]
    return f"{', '.join(names) or 'unknown'} (0x{reset_cause:08x})"


def describe_firmware_error(payload: bytes) -> str:
    """Name the result in an ACK/NACK payload, or say that none arrived."""
    if len(payload) < 2:
        return "no reply from the controller"
    if payload[1] == 0:
        return "accepted"
    error_number = 256 - payload[1]
    return f"-{FIRMWARE_ERRNO_NAMES.get(error_number, 'errno')} (-{error_number})"


class Device:
    def __init__(self, port: str) -> None:
        self.serial = open_head_serial(port, 115200, timeout=0,
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
                raise RuntimeError(f"probe rejected by firmware ({describe_firmware_error(payload)}); "
                                   "torque may be on, it may be busy, or bus initialization failed")
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
                raise RuntimeError(f"debug ping rejected by firmware ({describe_firmware_error(payload)})")
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
                raise RuntimeError(f"debug read rejected by firmware ({describe_firmware_error(payload)})")
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


def decode_position_range(raw: bytes) -> dict:
    """Decode POSITION_RANGE (send_position_range() in src/main.c).

    `ranges` maps each servo index in `valid_mask` to the (q = 0, q = 1) tick
    pair. `generation` matches the STATE byte at
    STATE_POSITION_RANGE_GENERATION_OFFSET while these ranges are current.
    """
    if len(raw) != POSITION_RANGE_PAYLOAD_LENGTH:
        raise RuntimeError("malformed position range response")
    transaction_id, generation, valid_mask, homed_mask = struct.unpack_from("<IB3xII", raw, 0)
    ticks = struct.unpack_from("<40i", raw, 16)
    ranges = {index: (ticks[2 * index], ticks[2 * index + 1])
              for index in range(20) if valid_mask & (1 << index)}
    if any(minimum >= maximum for minimum, maximum in ranges.values()):
        raise RuntimeError("position range response has an empty range")
    return {"transaction_id": transaction_id, "generation": generation,
            "valid_mask": valid_mask, "homed_mask": homed_mask, "ranges": ranges}


def normalized_position(tick: int, tick_range: tuple[int, int]) -> float:
    """q for a raw tick. Not clamped: a value outside 0..1 means the servo is
    outside its range, and callers must see that rather than a clipped value."""
    minimum, maximum = tick_range
    return (tick - minimum) / (maximum - minimum)


def decode_diagnostics(raw: bytes) -> dict:
    """Decode the fixed DIAGNOSTICS payload (send_diagnostics() in src/main.c)."""
    if len(raw) != DIAGNOSTICS_PAYLOAD_LENGTH:
        raise RuntimeError("malformed diagnostics response")
    (state, fault, control_deadline_misses, last_control_cycle_ms,
     control_max_period_us) = struct.unpack_from("<BBIII", raw, 0)
    usb_dropped_frames, usb_dropped_bytes, usb_coalesced_frames = struct.unpack_from(
        "<III", raw, DIAGNOSTICS_USB_COUNTERS_OFFSET)
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
        "state": state,
        "fault": fault,
        "control_deadline_misses": control_deadline_misses,
        "last_control_cycle_ms": last_control_cycle_ms,
        "control_max_period_us": control_max_period_us,
        "branches": branches,
        "usb_dropped_frames": usb_dropped_frames,
        "usb_dropped_bytes": usb_dropped_bytes,
        "usb_coalesced_frames": usb_coalesced_frames,
        "reset_cause": struct.unpack_from("<I", raw, DIAGNOSTICS_RESET_CAUSE_OFFSET)[0],
        "storage_state": raw[DIAGNOSTICS_STORAGE_STATE_OFFSET],
        "read_retries": struct.unpack_from("<H", raw, DIAGNOSTICS_READ_RETRIES_OFFSET)[0],
        "last_preparation_failure": raw[DIAGNOSTICS_PREPARATION_FAILURE_OFFSET],
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


def decode_bus_stats(payload: bytes) -> dict:
    field_count = len(BUS_STATS_BRANCH_FIELDS)
    expected_length = 4 + 4 * 4 * field_count + 4 * HEAD_SERVO_COUNT + 8
    if len(payload) != expected_length:
        raise RuntimeError(f"malformed bus statistics ({len(payload)} bytes)")
    values = struct.unpack(f"<I{4 * field_count}I{HEAD_SERVO_COUNT}III", payload)
    branches = [dict(zip(BUS_STATS_BRANCH_FIELDS,
                         values[1 + branch * field_count:1 + (branch + 1) * field_count]))
                for branch in range(4)]
    missed_end = 1 + 4 * field_count + HEAD_SERVO_COUNT
    return {"branches": branches, "missed_replies": list(values[1 + 4 * field_count:missed_end]),
            "window_max_period_us": values[missed_end],
            "control_deadline_misses": values[missed_end + 1]}


def decode_control_timing(payload: bytes) -> dict:
    if len(payload) != CONTROL_TIMING_PAYLOAD_LENGTH:
        raise RuntimeError(f"malformed control timing ({len(payload)} bytes)")
    values = struct.unpack("<IIQI5IIIII", payload)
    return {"control_cycle_count": values[1],
            "last_control_cycle_start_us": values[2],
            "schedule_restart_count": values[3],
            "period_bin_count": list(values[4:9]),
            "busy_over_half_period_count": values[9],
            "window_max_period_us": values[10],
            "window_max_busy_us": values[11],
            "window_max_lock_wait_us": values[12]}


def evaluate_control_timing(before: dict, after: dict, control_hz: int) -> dict:
    """Differences between two readings, and the pass rule applied to them."""
    def delta(name: str) -> int:
        return (after[name] - before[name]) & 0xFFFFFFFF
    cycle_count = delta("control_cycle_count")
    elapsed_us = after["last_control_cycle_start_us"] - before["last_control_cycle_start_us"]
    rate_hz = cycle_count * 1e6 / elapsed_us if elapsed_us > 0 else 0.0
    period_bins = [(last - first) & 0xFFFFFFFF for first, last in
                   zip(before["period_bin_count"], after["period_bin_count"])]
    period_us = 1_000_000 // control_hz
    checks = {
        "rate": cycle_count > 0 and
                abs(rate_hz - control_hz) <= CONTROL_RATE_TOLERANCE_FRACTION * control_hz,
        "no_lost_cycle": delta("schedule_restart_count") == 0 and period_bins[-1] == 0,
        "busy": after["window_max_busy_us"] < period_us // 2,
    }
    return {"cycle_count": cycle_count, "elapsed_us": elapsed_us, "rate_hz": rate_hz,
            "period_bins": period_bins,
            "schedule_restart_count": delta("schedule_restart_count"),
            "busy_over_half_period_count": delta("busy_over_half_period_count"),
            "period_us": period_us, "checks": checks, "passed": all(checks.values())}


def print_control_timing(result: dict, after: dict, control_hz: int,
                         host_elapsed_s: float, states: set[str]) -> None:
    print(f"over {result['elapsed_us'] / 1e6:.1f} s of controller time "
          f"({host_elapsed_s:.1f} s on this computer):")
    print(f"  cycles {result['cycle_count']}, mean rate {result['rate_hz']:.3f} Hz "
          f"(target {control_hz} Hz)")
    if host_elapsed_s > 0:
        print(f"  cycles per host second: {result['cycle_count'] / host_elapsed_s:.2f} "
              "(cross-check of the controller clock)")
    edges = (0,) + CONTROL_PERIOD_BIN_UPPER_US
    labels = [f"{low}-{high} us" for low, high in zip(edges, edges[1:])]
    labels.append(f">= {CONTROL_PERIOD_BIN_UPPER_US[-1]} us")
    print("  period between cycle starts:")
    total = sum(result["period_bins"])
    for label, count in zip(labels, result["period_bins"]):
        share = 100.0 * count / total if total else 0.0
        print(f"    {label:>14}: {count} ({share:.2f} %)")
    print(f"  longest period {after['window_max_period_us']} us, "
          f"schedule restarts (lost cycles) {result['schedule_restart_count']}")
    print(f"  longest busy time {after['window_max_busy_us']} us "
          f"(limit {result['period_us'] // 2} us), "
          f"cycles busy >= {result['period_us'] // 2} us: "
          f"{result['busy_over_half_period_count']}")
    print(f"  longest head_lock wait {after['window_max_lock_wait_us']} us")
    print(f"  controller states seen: {', '.join(sorted(states)) or 'none'}")
    if len(states) > 1:
        print("  WARNING: the state changed during the window, so this is not one "
              "steady-state measurement")
    for name, passed in result["checks"].items():
        print(f"  {name}: {'PASS' if passed else 'FAIL'}")
    print(f"RESULT: {'PASS' if result['passed'] else 'FAIL'}")


def print_bus_stats(before: dict, after: dict, seconds: float,
                    servo_branches: dict[int, int]) -> None:
    """Print what changed between two readings, per branch and per servo."""
    print(f"over {seconds:.1f} s:")
    print(f"  control loop: longest cycle {after['window_max_period_us']} us "
          f"(target 2000 us), missed deadlines "
          f"{after['control_deadline_misses'] - before['control_deadline_misses']}")
    for branch, (first, last) in enumerate(zip(before["branches"], after["branches"])):
        delta = {name: last[name] - first[name] for name in BUS_STATS_BRANCH_FIELDS}
        if delta["requests"] == 0:
            continue
        print(f"  J{branch + 1}: requests={delta['requests']} complete={delta['complete']} "
              f"({100.0 * delta['complete'] / delta['requests']:.1f}%) "
              f"timeouts: silent={delta['silent_timeouts']} "
              f"partial={delta['partial_timeouts']}")
        print(f"      UART: overrun={delta['overrun']} framing={delta['framing_errors']} "
              f"noise={delta['noise_errors']} ring_overflow={delta['ring_overflows']} "
              f"dma_starvation={delta['dma_starvations']}")
        print(f"      packets: resync_bytes={delta['resync_bytes']} "
              f"bad_length={delta['bad_lengths']} bad_packet={delta['bad_packets']} "
              f"unknown_id={delta['unknown_ids']} servo_error={delta['servo_errors']}")
        print(f"      bytes: received={delta['received_bytes']} "
              f"drained_stale={delta['drained_bytes']} "
              f"write_failures={delta['write_failures']} "
              f"rx_restart_failures={delta['rx_restart_failures']}")
        print(f"      driver: rx_gaps={delta['rx_gaps']} rx_overlaps={delta['rx_overlaps']} "
              f"dma_errors: rx={delta['dma_rx_errors']} tx={delta['dma_tx_errors']}")
        status = last["dma_rx_last_error_status"]
        errors = [name for bit, name in DMA_ERROR_STATUS_BITS if status >> bit & 1 and bit != 31]
        print(f"      eDMA: halted_samples={delta['dma_halt_samples']} "
              f"last_rx_error=0x{status:08x} ({', '.join(errors) or 'none'})")
        print(f"      bad packets: changed_in_place={delta['bad_in_place']} "
              f"bytes_lost_or_added={delta['bad_shifted']} "
              f"(the rest ended a reply, so they cannot be told apart)")
    print("missed replies per servo:")
    for index in sorted(servo_branches):
        missed = after["missed_replies"][index] - before["missed_replies"][index]
        print(f"  {servo_label(index, servo_branches)}: {missed}")


def read_servo_branches(device: Device, active_mask: int) -> dict[int, int]:
    """Branch index each active servo is provisioned on, read from the profile.

    The branch is a per-servo profile field, not a function of the ID."""
    return {index: decode_slot(device.get(GET_SLOT, SLOT, bytes((index,))))["branch"]
            for index in range(HEAD_SERVO_COUNT) if (active_mask >> index) & 1}


def servo_label(index: int, servo_branches: dict[int, int]) -> str:
    branch = servo_branches.get(index)
    connector = f"J{branch + 1}" if branch is not None else "J?"
    return f"servo {index:2d} ({connector} id {index})"


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


def upload_profile(device: Device, token: int, profile: dict,
                   say=lambda _text: None) -> int:
    """Stage, commit and verify a profile (upload-profile). Returns the new
    generation once it is stored and reads back the same."""
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
    say("profile accepted and queued for persistence")
    if device.configuration_generation is None:
        raise RuntimeError("cannot verify profile persistence without a session generation")
    expected_generation = (device.configuration_generation + 1) & 0xFFFFFFFF
    wait_for_profile_persistence(device, expected_generation)
    stored_profile = read_profile(device)
    if not profile_matches(profile, stored_profile):
        raise RuntimeError("profile persistence completed but readback did not match")
    device.configuration_generation = expected_generation
    return expected_generation


def acquire(device: Device) -> int:
    ok, payload = device.request(ACQUIRE, b"headctl")
    if not ok or len(payload) < 6:
        raise RuntimeError("could not acquire control lease")
    return struct.unpack_from("<I", payload, 2)[0]


def start_tuning(device: Device, token: int, current_limit_ma: int) -> None:
    """Torque on at the present pose with every Goal Current capped, then hold."""
    if not 0 < current_limit_ma <= 0xFFFF:
        raise ValueError("tuning current limit must be a positive mA value")
    ok, _ = device.request(START_TUNING, struct.pack("<IH", token, current_limit_ma))
    if not ok:
        raise RuntimeError("tuning rejected: needs HOMING_REQUIRED, torque off, fresh "
                           "telemetry, and a limit no higher than every operating_current_ma")


TUNING_MAX_STEP_TICKS = 50  # HEAD_TUNING_MAX_STEP_TICKS in include/head.h


def tuning_step(device: Device, token: int, servo_index: int, step_ticks: int) -> int:
    """Request one tuning step; returns the firmware result (0 = moved)."""
    if step_ticks == 0 or abs(step_ticks) > TUNING_MAX_STEP_TICKS:
        raise ValueError(f"step must be 1..{TUNING_MAX_STEP_TICKS} ticks either way")
    transaction_id = device.send_confirmed(
        TUNING_STEP, struct.pack("<IBb", token, servo_index, step_ticks))
    deadline_monotonic_s = time.monotonic() + 1.0
    while time.monotonic() < deadline_monotonic_s:
        frame = device.recv(deadline_monotonic_s - time.monotonic())
        if frame is None:
            break
        kind, payload = frame
        if kind in (ACK, NACK) and device.ack_matches(TUNING_STEP, payload, transaction_id):
            return struct.unpack_from("<b", payload, 1)[0]
    raise RuntimeError("controller did not answer the tuning step")


def decode_black_box(payload: bytes) -> dict:
    (_transaction_id, kept, boot_count, reset_cause, holder_line, holder_since_ms,
     last_feed_ms, phase, phase_ms) = struct.unpack_from("<IBIIIIIII", payload, 0)
    trail = [struct.unpack_from("<II", payload, 33 + 8 * slot) for slot in range(8)]
    telemetry = (struct.unpack_from("<IIII", payload, 97) if len(payload) >= 113
                 else None)
    timer_ms = None
    uart = []
    if len(payload) >= 113 + 4 + 4 * 15 * 4:
        timer_ms = struct.unpack_from("<I", payload, 113)[0]
        for branch in range(4):
            values = struct.unpack_from("<15I", payload, 117 + branch * 60)
            uart.append({"now": values[0:7], "at_feed": values[7:14],
                         "last_ms": values[14]})
    running = None
    if len(payload) >= 357 + 68:
        thread, = struct.unpack_from("<I", payload, 357)
        name = payload[361:377].split(b"\0", 1)[0].decode("ascii", "replace")
        pc, lr = struct.unpack_from("<II", payload, 377)
        active = struct.unpack_from("<5I", payload, 385)
        pending = struct.unpack_from("<5I", payload, 405)
        running = {"thread": thread, "name": name, "pc": pc, "lr": lr,
                   "active": [32 * word + bit for word in range(5) for bit in range(32)
                              if active[word] >> bit & 1],
                   "pending": [32 * word + bit for word in range(5) for bit in range(32)
                               if pending[word] >> bit & 1]}
    consume = struct.unpack_from("<4I", payload, 425) if len(payload) >= 441 else None
    registers = None
    if len(payload) >= 489:
        values = struct.unpack_from("<12I", payload, 441)
        registers = {"dma_error_status": values[0], "dma_error_channels": values[1],
                     "dma_interrupt_channels": values[2],
                     "enabled": [32 * word + bit for word in range(5) for bit in range(32)
                                 if values[3 + word] >> bit & 1],
                     "uart_status": values[8:12]}
    return {"registers": registers, "consume": consume, "running": running, "timer_ms": timer_ms, "uart": uart, "telemetry": telemetry,"kept": bool(kept), "boot_count": boot_count, "reset_cause": reset_cause,
            "holder_line": holder_line, "holder_since_ms": holder_since_ms,
            "last_feed_ms": last_feed_ms, "phase": phase, "phase_ms": phase_ms,
            "trail": [(line, ms) for line, ms in trail if line]}


def print_black_box(report: dict) -> None:
    print(f"last restart cause: {describe_reset_cause(report['reset_cause'])}")
    if not report["kept"]:
        print("no black box from before the restart: the memory was not kept "
              "(normal after power-on or a new flash)")
        return
    print(f"boots with the black box kept: {report['boot_count']}")
    feed = report["last_feed_ms"]
    print(f"last watchdog feed: {feed} ms after that boot")
    phase = report["phase"]
    phase_name = (BLACK_BOX_PHASE_NAMES[phase] if phase < len(BLACK_BOX_PHASE_NAMES)
                  else f"unknown ({phase})")
    print(f"control thread was in: {phase_name}, since {report['phase_ms']} ms "
          f"({report['phase_ms'] - feed:+d} ms from the last feed)")
    if report["holder_line"]:
        print(f"lock was held by main.c line {report['holder_line']}, since "
              f"{report['holder_since_ms']} ms ({report['holder_since_ms'] - feed:+d} ms "
              f"from the last feed)")
    else:
        print("lock was free")
    print("last lock takers, oldest first (main.c line @ ms):")
    for line, ms in report["trail"]:
        print(f"  line {line} @ {ms} ms ({ms - feed:+d})")
    if report["telemetry"] is not None:
        point, branch, drain_count, point_ms = report["telemetry"]
        names = ("none", "reading replies", "building request", "emptying old input",
                 "sending request", "request sent")
        name = names[point] if point < len(names) else f"unknown ({point})"
        print(f"telemetry was at: {name}, branch J{branch + 1}, since {point_ms} ms "
              f"({point_ms - feed:+d} ms from the last feed); emptying loop ran "
              f"{drain_count} times")
    if report["timer_ms"] is not None:
        timer_ms = report["timer_ms"]
        verdict = ("interrupts KEPT RUNNING after the last feed"
                   if timer_ms - feed > 20 else
                   "interrupts STOPPED near the last feed")
        print(f"10 ms timer interrupt last ran at {timer_ms} ms "
              f"({timer_ms - feed:+d} ms from the last feed): {verdict}")
        event_names = ("tx_done", "tx_aborted", "rx_ready", "rx_buf_request",
                       "rx_buf_released", "rx_disabled", "rx_stopped(error)")
        print("serial-port events after the last feed, per branch:")
        for branch, data in enumerate(report["uart"]):
            after = {name: now - then for name, now, then
                     in zip(event_names, data["now"], data["at_feed"]) if now != then}
            total = ", ".join(f"{name}={count}" for name, count in after.items()) or "none"
            print(f"  J{branch + 1}: {total}; last event at {data['last_ms']} ms "
                  f"({data['last_ms'] - feed:+d})")
    running = report.get("running")
    if running is not None:
        print(f"last timer tick found running: task '{running['name'] or '?'}' "
              f"(0x{running['thread']:08x}) at pc=0x{running['pc']:08x} "
              f"lr=0x{running['lr']:08x}")
        print(f"  interrupts active: {running['active'] or 'none'}; "
              f"waiting: {running['pending'] or 'none'}")
    registers = report.get("registers")
    if registers is not None:
        status = registers["dma_error_status"]
        errors = [name for bit, name in DMA_ERROR_STATUS_BITS if status >> bit & 1]
        channel = (status >> 8) & 0x1F
        print(f"  eDMA error status 0x{status:08x}: {', '.join(errors) or 'no error'}"
              + (f" on channel {channel} ({DMA_CHANNEL_NAMES.get(channel, '?')})"
                 if errors else ""))
        for label, mask in (("channels with an error flag", registers["dma_error_channels"]),
                            ("channels with an interrupt flag",
                             registers["dma_interrupt_channels"])):
            channels = [f"{bit} ({DMA_CHANNEL_NAMES.get(bit, '?')})"
                        for bit in range(32) if mask >> bit & 1]
            print(f"  {label}: {', '.join(channels) or 'none'}")
        waiting = set(running["pending"]) if running is not None else set()
        print(f"  waiting interrupts that are enabled: "
              f"{sorted(waiting & set(registers['enabled'])) or 'none'}")
        print("  LPUART STAT: " + ", ".join(
            f"J{branch + 1}=0x{value:08x}" for branch, value in enumerate(registers["uart_status"])))
    consume = report.get("consume")
    if consume is not None:
        used, loops, bad_count, bad_value = consume
        print(f"telemetry sorting: {used} bytes buffered at start, {loops} passes; "
              f"out-of-range byte counts reset {bad_count} times"
              + (f" (last value {bad_value})" if bad_count else ""))


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
    parser.add_argument("--port", default=AUTO_PORT,
                        help=f"head protocol port (CDC1); '{AUTO_PORT}' opens the one "
                             f"{HEAD_PORT_PATTERN} link (default: {AUTO_PORT})")
    parser.add_argument("command", choices=(
        "status", "diagnostics", "probe", "ping-debug", "read-register", "line-test", "uart-tx-test",
        "uart-rx-test", "export-profile", "upload-profile", "home", "enable",
        "proprioception", "routing", "disable", "clear-fault", "release",
        "zero-home", "blackbox", "bus-stats", "control-timing"))
    parser.add_argument("--branch", type=int, choices=range(4), default=0,
                        help="physical branch: J1=0, J2=1, J3=2, J4=3")
    parser.add_argument("--baud", type=int, default=BRANCH_DEFAULT_BAUD,
                        help="temporary read-only baud for probe and ping-debug "
                             f"(default: {BRANCH_DEFAULT_BAUD}; factory XC330: 57600, "
                             "which only ping-debug may use: a 57600 probe resets the board)")
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
    parser.add_argument("--seconds", type=float, default=10.0,
                        help="bus-stats and control-timing measuring window (default: 10)")
    args = parser.parse_args()
    if args.command == "enable" and args.lease == 0:
        parser.error("enable requires --lease from the controller that will stream targets")
    device = Device(args.port)
    acquired_here = False
    token = 0
    try:
        hello = device.hello()
        if args.command == "blackbox":
            print_black_box(decode_black_box(device.get(GET_BLACK_BOX, BLACK_BOX)))
            return
        if args.command == "bus-stats":
            if not hello["capabilities"] & CAP_BUS_STATS:
                raise RuntimeError("this firmware image has no bus statistics")
            info = decode_info(device.get(GET_INFO, INFO))
            servo_branches = read_servo_branches(device, info["active_servo_mask"])
            before = decode_bus_stats(device.get(GET_BUS_STATS, BUS_STATS))
            # Keep reading while waiting: STATE streams at 100 Hz, and recv()
            # reports a controller reboot during the window.
            deadline_monotonic_s = time.monotonic() + args.seconds
            while time.monotonic() < deadline_monotonic_s:
                device.recv(deadline_monotonic_s - time.monotonic())
            after = decode_bus_stats(device.get(GET_BUS_STATS, BUS_STATS))
            print_bus_stats(before, after, args.seconds, servo_branches)
            return
        if args.command == "control-timing":
            if not hello["capabilities"] & CAP_CONTROL_TIMING:
                raise RuntimeError("this firmware image has no control timing")
            # The first reading also restarts the window maxima.
            before = decode_control_timing(device.get(GET_CONTROL_TIMING, CONTROL_TIMING))
            started_monotonic_s = time.monotonic()
            deadline_monotonic_s = started_monotonic_s + args.seconds
            states: set[str] = set()
            while time.monotonic() < deadline_monotonic_s:
                frame = device.recv(deadline_monotonic_s - time.monotonic())
                if frame is not None and frame[0] == STATE and frame[1]:
                    state = frame[1][0]
                    states.add(STATE_NAMES[state] if state < len(STATE_NAMES)
                               else f"UNKNOWN({state})")
            after = decode_control_timing(device.get(GET_CONTROL_TIMING, CONTROL_TIMING))
            host_elapsed_s = time.monotonic() - started_monotonic_s
            result = evaluate_control_timing(before, after, hello["control_hz"])
            print_control_timing(result, after, hello["control_hz"], host_elapsed_s, states)
            if not result["passed"]:
                raise SystemExit(1)
            return
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
                        servo_branches = read_servo_branches(device, active_mask)
                    except RuntimeError:
                        active_mask = None
                        servo_branches = {}
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
                        print(f"  {servo_label(index, servo_branches)}: "
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
            failure = report["last_preparation_failure"]
            if failure:
                phase, servo = failure >> 5, failure & 0x1F
                print(f"last_preparation_failure phase={phase} "
                      f"({PREPARATION_PHASE_NAMES[phase]}) servo="
                      f"{'none' if servo == 31 else servo}")
            else:
                print("last_preparation_failure none")
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
                servo_branches = read_servo_branches(device, active_mask)
            except RuntimeError:
                active_mask = None
                servo_branches = {}
            print("discovery reasons:")
            for index, reason in enumerate(report["discovery_reasons"]):
                if active_mask is not None and not (active_mask >> index) & 1:
                    continue
                name = (DISCOVERY_REASON_NAMES[reason] if reason < len(DISCOVERY_REASON_NAMES)
                        else f"UNKNOWN({reason})")
                print(f"  {servo_label(index, servo_branches)}: {name}")
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
            expected_generation = upload_profile(device, token, profile,
                                                 say=lambda text: print(text, flush=True))
            print(f"profile committed at generation {expected_generation}")
            return
        message = {"home": HOME, "enable": ENABLE,
                   "proprioception": START_PROPRIOCEPTION,
                   "routing": START_ROUTING,
                   "disable": DISABLE, "clear-fault": CLEAR_FAULT,
                   "release": RELEASE,
                   "zero-home": ZERO_HOME}[args.command]
        ok, response = device.request(
            message, struct.pack("<I", token),
            CLEAR_FAULT_REPLY_TIMEOUT_S if message == CLEAR_FAULT else 1.0)
        if not ok:
            raise RuntimeError(f"{args.command} rejected: {describe_firmware_error(response)}")
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
            except (RuntimeError, TimeoutError, OSError):
                pass
        device.close()


if __name__ == "__main__":
    main()
