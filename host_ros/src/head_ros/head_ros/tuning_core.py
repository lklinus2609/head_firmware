"""ROS-independent logic for the tuning panel.

Tuning is the commissioning step before any limits are known: the firmware
holds every active servo where it was at torque-on, with Goal Current capped at
an operator-chosen limit, and accepts single-tick steps for one servo at a
time (HEAD_TUNING in firmware/src/state_machine.c). This module talks to the
controller through headctl's serial Device, keeps the lease alive, and keeps a
record of every step and mark so a profile can be proposed from it later.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import struct
import tempfile
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

try:
    from . import headctl, records_layout
except ImportError:  # run as a plain script from this directory
    import headctl  # type: ignore[no-redef]
    import records_layout  # type: ignore[no-redef]

TUNING_STATE = headctl.STATE_NAMES.index("TUNING")
FAULT_STATE = headctl.STATE_NAMES.index("FAULT")
HOMING_REQUIRED_STATE = headctl.STATE_NAMES.index("HOMING_REQUIRED")

# Same renewal period as the jog and routing controls; the lease lasts 1 s.
LEASE_RENEW_PERIOD_S = 0.4
# A renewal or step with no answer in this long means control is lost.
ANSWER_TIMEOUT_S = 1.0
# STATE frames arrive every 10 ms (99.9 Hz measured at the PC, 2026-10-20).
# Older than this, the display is stale and the step buttons are locked.
STATE_FRESH_S = 0.35

# Firmware result codes for a refused step. The values are picolibc's
# (firmware/modules/lib/picolibc/newlib/libc/include/sys/errno.h), which the
# Zephyr build links, not the host's errno module.
STEP_RESULT_TEXT = {
    0: "Moved one step.",
    -1: "Not moved. The board is not in tuning.",              # EPERM
    -11: "Not moved. No fresh data from this servo. Try again.",  # EAGAIN
    -16: "Not moved. The servo is still moving, or it reached the load limit. "
         "Wait, or move back.",  # EBUSY
    -22: "Not moved. Bad request.",                             # EINVAL
    -34: "Not moved. One full turn from the start. You can only move back.",  # ERANGE
}

MARK_NAMES = ("neutral", "end_1", "end_2")


def step_result_text(code: int) -> str:
    return STEP_RESULT_TEXT.get(code, f"Not moved. Board answer {code}.")


@dataclass
class ServoSample:
    index: int
    goal_tick: int
    present_tick: int
    current_ma: int
    online: bool
    feedback_age_ms: Optional[int]
    hardware_error: int = 0
    temperature_c: int = 0
    voltage_mv: int = 0


# Hardware Error Status(70) bits, XC330-T181 datasheet
# (hardware/datasheets/xc330-t181.pdf). The firmware's Shutdown(63) mask 0x34
# makes the servo switch its own torque off on Overload, Electrical Shock and
# Overheating.
HARDWARE_ERROR_BITS = ((0x20, "Overload"), (0x10, "Electrical Shock / insufficient power"),
                       (0x08, "Motor Encoder"), (0x04, "Overheating"),
                       (0x01, "Input Voltage"))


def describe_hardware_error(value: int) -> str:
    names = [name for bit, name in HARDWARE_ERROR_BITS if value & bit]
    return f"0x{value:02x} ({', '.join(names) or 'unknown bit'})"


def hardware_error_report(state: "ControllerState") -> str:
    """One line per servo reporting a hardware error, with what it read."""
    return "; ".join(
        f"servo {servo.index}: {describe_hardware_error(servo.hardware_error)} at "
        f"{servo.current_ma} mA, {servo.temperature_c} °C, {servo.voltage_mv / 1000:.1f} V"
        for servo in state.servos if servo.hardware_error)


@dataclass
class ControllerState:
    state: int
    fault: int
    torque_state: Optional[int]
    servos: List[ServoSample]
    received_monotonic_s: float
    # Controller uptime when the sample was built (k_uptime_get_32 in
    # send_state); PRESENT_PWM carries the same value.
    mcu_time_ms: int = 0

    @property
    def state_name(self) -> str:
        names = headctl.STATE_NAMES
        return names[self.state] if self.state < len(names) else f"UNKNOWN({self.state})"

    @property
    def fault_name(self) -> str:
        names = headctl.FAULT_NAMES
        return names[self.fault] if self.fault < len(names) else f"UNKNOWN({self.fault})"


def decode_state(payload: bytes, received_monotonic_s: float) -> ControllerState:
    """Decode the STATE payload with headctl's documented layout."""
    if len(payload) < headctl.SERVO_BLOCK_OFFSET + headctl.SERVO_RECORD_LENGTH * headctl.HEAD_SERVO_COUNT:
        raise ValueError("STATE payload is too short")
    servos = []
    for index in range(headctl.HEAD_SERVO_COUNT):
        offset = headctl.SERVO_BLOCK_OFFSET + index * headctl.SERVO_RECORD_LENGTH
        (goal, present, _velocity, current_ma, voltage_mv, temperature_c,
         _moving, hardware_error, online, age_ms) = struct.unpack_from(
             "<iiihHBBBBI", payload, offset)
        servos.append(ServoSample(index, goal, present, current_ma, bool(online),
                                  None if age_ms == 0xFFFFFFFF else age_ms,
                                  hardware_error, temperature_c, voltage_mv))
    torque_state = payload[16] if len(payload) > 16 else None
    mcu_time_ms = struct.unpack_from("<I", payload, headctl.STATE_MCU_TIME_OFFSET)[0]
    return ControllerState(payload[0], payload[1], torque_state, servos, received_monotonic_s,
                           mcu_time_ms)


def decode_present_pwm(payload: bytes) -> tuple[int, tuple[int, ...]]:
    """(MCU time ms, signed Present PWM register value per servo slot)."""
    if len(payload) < headctl.PRESENT_PWM_PAYLOAD_LENGTH:
        raise ValueError("PRESENT_PWM payload is too short")
    mcu_time_ms = struct.unpack_from("<I", payload, 0)[0]
    return mcu_time_ms, struct.unpack_from(f"<{headctl.HEAD_SERVO_COUNT}h", payload, 4)


@dataclass
class TuningRecord:
    """Everything one tuning session observed, saved after every change."""

    path: Path
    current_limit_ma: int
    servo_branches: Dict[int, int]
    started_utc: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    neutral_ticks: Dict[int, int] = field(default_factory=dict)
    peak_current_ma: Dict[int, int] = field(default_factory=dict)
    marks: Dict[int, Dict[str, Dict[str, Any]]] = field(default_factory=dict)
    steps: List[Dict[str, Any]] = field(default_factory=list)
    # Notes typed during this session; the latest text per servo also lives in
    # NOTES_FILE so it shows in every later session.
    notes: Dict[int, str] = field(default_factory=dict)
    # Board faults seen during the session, with every servo's reading then.
    faults: List[Dict[str, Any]] = field(default_factory=list)

    @property
    def telemetry_path(self) -> Path:
        """The session's TelemetryLog file, next to the record."""
        return self.path.with_suffix(TELEMETRY_SUFFIX)

    def add_fault(self, state: "ControllerState") -> None:
        self.faults.append({
            "utc": datetime.now(timezone.utc).isoformat(), "fault": state.fault_name,
            # Finds the fault in the telemetry file's mcu_time_ms column.
            "mcu_time_ms": state.mcu_time_ms,
            "servos": [{"servo": servo.index, "hardware_error": servo.hardware_error,
                        "goal_tick": servo.goal_tick, "present_tick": servo.present_tick,
                        "current_ma": servo.current_ma, "temperature_c": servo.temperature_c,
                        "voltage_mv": servo.voltage_mv}
                       for servo in state.servos if index_is_reported(servo)]})

    def observe(self, state: ControllerState) -> None:
        """Take neutral from the first tuning sample; track peak load."""
        if state.state != TUNING_STATE:
            return
        for servo in state.servos:
            if servo.index not in self.servo_branches:
                continue
            # Until the first step the goal is exactly the pose preparation
            # read at torque-on, which is the firmware's own neutral.
            self.neutral_ticks.setdefault(servo.index, servo.goal_tick)
            peak = self.peak_current_ma.get(servo.index, 0)
            if abs(servo.current_ma) > peak:
                self.peak_current_ma[servo.index] = abs(servo.current_ma)

    def add_step(self, servo: ServoSample, direction: int, result: int) -> None:
        self.steps.append({
            "utc": datetime.now(timezone.utc).isoformat(), "servo": servo.index,
            "step_ticks": direction, "result": result, "goal_tick": servo.goal_tick,
            "present_tick": servo.present_tick, "current_ma": servo.current_ma})

    def add_mark(self, servo: ServoSample, name: str) -> None:
        if name not in MARK_NAMES:
            raise ValueError(f"unknown mark {name!r}")
        self.marks.setdefault(servo.index, {})[name] = {
            "utc": datetime.now(timezone.utc).isoformat(),
            "present_tick": servo.present_tick, "goal_tick": servo.goal_tick,
            "current_ma": servo.current_ma}

    def to_json(self) -> Dict[str, Any]:
        return {
            "format": "head-tuning-record-1",
            "started_utc": self.started_utc,
            "current_limit_ma": self.current_limit_ma,
            "servos": {
                str(index): {
                    "branch": branch,
                    "neutral_tick": self.neutral_ticks.get(index),
                    "peak_current_ma": self.peak_current_ma.get(index, 0),
                    "marks": self.marks.get(index, {}),
                    "note": self.notes.get(index, ""),
                } for index, branch in sorted(self.servo_branches.items())},
            "steps": self.steps,
            "faults": self.faults,
            "telemetry_file": self.telemetry_path.name,
        }

    def save(self) -> None:
        """Write atomically so a crash never leaves a half-written record."""
        write_json_atomically(self.path, self.to_json())


# "<start>_tuning.json" -> "<start>_tuning.telemetry.csv". It does not match
# the "*_tuning.json" pattern that load_earlier_marks reads.
TELEMETRY_SUFFIX = ".telemetry.csv"
# STATE comes at up to 100 Hz. One row per servo every 100 ms of controller
# time is enough to see load, PWM and temperature build up over a hold that
# lasts minutes, and keeps a 20-servo file near 65 MB per hour.
TELEMETRY_LOG_PERIOD_MS = 100
TELEMETRY_COLUMNS = ("host_utc", "mcu_time_ms", "board_state", "fault", "servo",
                     "goal_tick", "present_tick", "current_ma", "pwm_percent",
                     "temperature_c", "voltage_mv", "hardware_error", "online",
                     "feedback_age_ms")


class TelemetryLog:
    """Every tuned servo's telemetry for the whole session, in a CSV file.

    A board fault snapshot is taken after the servo has already switched itself
    off, so it cannot show what led up to the fault. This file can: one row per
    servo every TELEMETRY_LOG_PERIOD_MS of controller time, plus a row set at
    once whenever the board state, the fault or a servo's hardware error
    changes, so the sample that shows a trip is never skipped. Rows are written
    as they arrive, line by line, so the file is complete up to a crash.
    pwm_percent is empty when the firmware has no PRESENT_PWM.

    A write error stops the log and is kept in `error`; it never stops tuning.
    """

    def __init__(self, path: Path, servo_indexes: Sequence[int]) -> None:
        self.path = Path(path)
        self.servo_indexes = sorted(servo_indexes)
        self.error: Optional[str] = None
        self.row_count = 0
        self._last_logged_mcu_ms: Optional[int] = None
        self._last_condition: Optional[tuple] = None
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.path, "w", encoding="utf-8", newline="", buffering=1)
        self._writer = csv.writer(self._file, lineterminator="\n")
        self._writer.writerow(TELEMETRY_COLUMNS)

    def observe(self, state: ControllerState,
                present_pwm_raw: Optional[Sequence[int]]) -> bool:
        """Log this sample if it is due or something changed; True if logged."""
        if self._file.closed:
            return False
        condition = (state.state, state.fault,
                     tuple(state.servos[index].hardware_error for index in self.servo_indexes))
        due = (self._last_logged_mcu_ms is None or
               (state.mcu_time_ms - self._last_logged_mcu_ms) & 0xFFFFFFFF >=
               TELEMETRY_LOG_PERIOD_MS)
        if not due and condition == self._last_condition:
            return False
        host_utc = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        try:
            for index in self.servo_indexes:
                servo = state.servos[index]
                pwm_percent = ("" if present_pwm_raw is None else
                               f"{present_pwm_raw[index] * headctl.PRESENT_PWM_PERCENT_PER_UNIT:.2f}")
                self._writer.writerow((
                    host_utc, state.mcu_time_ms, state.state_name, state.fault_name, index,
                    servo.goal_tick, servo.present_tick, servo.current_ma, pwm_percent,
                    servo.temperature_c, servo.voltage_mv, servo.hardware_error,
                    int(servo.online),
                    "" if servo.feedback_age_ms is None else servo.feedback_age_ms))
                self.row_count += 1
        except OSError as error:
            self.error = f"Telemetry log stopped: {error}"
            self.close()
            return False
        self._last_logged_mcu_ms = state.mcu_time_ms
        self._last_condition = condition
        return True

    def close(self) -> None:
        try:
            self._file.close()
        except OSError:
            pass


# At saturation the servo reads about the limit, a few mA either side.
BACKOFF_LOAD_FRACTION = 0.95
BACKOFF_HOLD_S = 0.3
# Same window as HOME_SETTLE_TICKS and the firmware's smallest lag limit.
BACKOFF_LEAD_TICKS = 20
# There is deliberately no "stalled" trigger. With Position I = 0 a servo
# holding a load of X mA must trail its goal by about X / 3 ticks for as long
# as it holds, so "goal ahead and not moving" is normal holding, not a fault;
# a stall rule released servo 7 holding 454 mA on 2026-10-12. The operator
# wants a servo to hold at any load below the limit.


class BackOff:
    """Releases a servo that is pushing at the load limit against something
    that does not move. Below the limit it is left holding.

    A refused step does not reduce the push: the goal stays ahead and the
    servo keeps pulling at the limit, which on 2026-10-12 held servo 7 at
    ~600 mA (its Current Limit) until its own Overload protection shut it off.
    So when a servo has stayed at BACKOFF_LOAD_FRACTION of the limit for
    BACKOFF_HOLD_S with its goal BACKOFF_LEAD_TICKS or more ahead, or a step
    was refused with the goal that far ahead, the goal is stepped back toward
    where the servo is until it leads by less than BACKOFF_LEAD_TICKS. The
    hold time keeps the short peak after an ordinary step from triggering it.
    It does not wait for the servo to catch up (a blocked servo never does),
    only for a STATE newer than its last answer, so it never steps from a stale
    goal."""

    def __init__(self) -> None:
        self._loaded_since_s: Dict[int, float] = {}

        self._last_answer_s: Dict[int, float] = {}
        self.active: set[int] = set()

    @staticmethod
    def _lead(servo: ServoSample) -> int:
        return servo.goal_tick - servo.present_tick

    def observe(self, state: ControllerState, limit_ma,
                watched: List[int]) -> List[int]:
        """Update from a STATE sample; returns servos that just started backing
        off. limit_ma is one limit for all, or servo -> its own tuning cap."""
        started = []
        now = state.received_monotonic_s
        for index in watched:
            servo = state.servos[index]
            limit = limit_ma[index] if isinstance(limit_ma, dict) else limit_ma
            leading = abs(self._lead(servo)) >= BACKOFF_LEAD_TICKS
            loaded = False
            if leading and abs(servo.current_ma) >= BACKOFF_LOAD_FRACTION * limit:
                loaded = now - self._loaded_since_s.setdefault(index, now) >= BACKOFF_HOLD_S
            else:
                self._loaded_since_s.pop(index, None)
            if loaded and index not in self.active:
                self.active.add(index)
                started.append(index)
        return started

    def on_refused(self, index: int, state: Optional[ControllerState]) -> bool:
        """A step was refused; back off if the goal is left far ahead."""
        if state is None or index in self.active:
            return False
        if abs(self._lead(state.servos[index])) >= BACKOFF_LEAD_TICKS:
            self.active.add(index)
            return True
        return False

    def on_result(self, index: int, answered_s: float) -> None:
        self._last_answer_s[index] = answered_s

    def next_step(self, state: ControllerState) -> Optional[tuple[int, int]]:
        for index in sorted(self.active):
            servo = state.servos[index]
            if state.received_monotonic_s <= self._last_answer_s.get(index, float("-inf")):
                continue
            lead = self._lead(servo)
            if abs(lead) < BACKOFF_LEAD_TICKS:
                self.active.discard(index)
                self._loaded_since_s.pop(index, None)
                continue
            limit = headctl.TUNING_MAX_STEP_TICKS
            # Move the goal toward the servo, stopping inside the lead window.
            distance = abs(lead) - (BACKOFF_LEAD_TICKS - 1)
            step = min(limit, distance)
            return index, -step if lead > 0 else step
        return None

    def give_up(self, index: int) -> None:
        self.active.discard(index)


def index_is_reported(servo: ServoSample) -> bool:
    return servo.online or servo.hardware_error != 0 or servo.feedback_age_ms is not None


def write_json_atomically(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".tuning-", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            json.dump(data, output, indent=2)
            output.write("\n")
        os.replace(temporary, path)
    except BaseException:
        Path(temporary).unlink(missing_ok=True)
        raise


NOTES_FILE = "servo_notes.json"
NOTES_FORMAT = "head-servo-notes-1"


def load_notes(directory: Path) -> Dict[int, str]:
    """Latest note per servo from NOTES_FILE in the records' config/; empty if
    there is none yet."""
    path = records_layout.config_dir(directory) / NOTES_FILE
    if not path.exists():
        return {}
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("format") != NOTES_FORMAT:
        raise ValueError(f"{path} is not a servo notes file")
    return {int(key): str(entry["text"]) for key, entry in data["servos"].items()
            if str(entry.get("text", ""))}


def save_note(directory: Path, index: int, text: str) -> None:
    """Set one servo's note (empty text removes it), keeping the others."""
    notes = load_notes(directory)
    path = records_layout.config_dir(directory) / NOTES_FILE
    existing = (json.loads(path.read_text(encoding="utf-8"))["servos"]
                if path.exists() else {})
    servos = {key: entry for key, entry in existing.items() if int(key) in notes}
    if text:
        servos[str(index)] = {"text": text,
                              "updated_utc": datetime.now(timezone.utc).isoformat()}
    else:
        servos.pop(str(index), None)
    write_json_atomically(path, {"format": NOTES_FORMAT,
                                 "servos": dict(sorted(servos.items(), key=lambda item: int(item[0])))})


def is_ready_for_profile(marks: Dict[str, Any]) -> bool:
    """A start and at least one end: what propose_profile.py needs."""
    return "neutral" in marks and ("end_1" in marks or "end_2" in marks)


def new_record_path(directory: Path, name: Optional[str] = None) -> Path:
    """A new record file: `name` in `directory` (a run folder), or
    <local date>_<time>_tuning.json (the panel's records in panel/)."""
    return Path(directory) / (name or f"{records_layout.local_stamp()}_tuning.json")


RECORD_FORMAT = "head-tuning-record-1"


# The first panel record of the new head prototype (2026-11-09, local time).
# Marks from earlier records belong to the old head (other skin, other
# anchors, 15/16 swapped) and must not set a rest or a span on this one.
HEAD_MARKS_SINCE = "2026-11-09_141836"


def record_stamp_digits(name: str) -> str:
    """The first 14 digits of a record file name (YYYYMMDDHHMMSS), for both
    the old 20261009T000000Z_ and the present 2026-11-09_141836_ names."""
    return "".join(character for character in name if character.isdigit())[:14]


def load_earlier_marks(directory: Path, since: Optional[str] = None
                       ) -> tuple[Dict[int, Dict[str, Dict[str, Any]]], List[str]]:
    """Marks from every panel record (the records' panel/ folder), merged the
    way scripts/propose_profile.py merges them: per servo and mark name, the
    latest file wins. File names start with the start time, so name order is
    time order. With `since` (a record name stamp such as HEAD_MARKS_SINCE),
    records older than it are ignored. Returns the marks and the names of
    files that could not be read."""
    merged: Dict[int, Dict[str, Dict[str, Any]]] = {}
    skipped: List[str] = []
    cutoff = record_stamp_digits(since) if since else ""
    for path in sorted(records_layout.panel_dir(directory).glob("*_tuning.json")):
        if cutoff and record_stamp_digits(path.name) < cutoff:
            continue
        try:
            record = json.loads(path.read_text(encoding="utf-8"))
            if record.get("format") != RECORD_FORMAT:
                raise ValueError("not a tuning record")
            for key, data in record["servos"].items():
                for name, mark in dict(data.get("marks", {})).items():
                    if name in MARK_NAMES:
                        merged.setdefault(int(key), {})[name] = dict(mark)
        except (OSError, ValueError, KeyError, TypeError, AttributeError):
            skipped.append(path.name)
    return merged, skipped


def home_tick(index: int, record: Optional["TuningRecord"],
              earlier_marks: Dict[int, Dict[str, Dict[str, Any]]]) -> Optional[int]:
    """Where Home takes a servo: the latest clicked start mark (this session
    first, then earlier sessions), else this session's start pose. This is the
    value propose_profile.py writes as home_tick."""
    if record is not None and "neutral" in record.marks.get(index, {}):
        return int(record.marks[index]["neutral"]["present_tick"])
    if "neutral" in earlier_marks.get(index, {}):
        return int(earlier_marks[index]["neutral"]["present_tick"])
    if record is not None and index in record.neutral_ticks:
        return int(record.neutral_ticks[index])
    return None


# Home never moves these servos. The jaw (7, servo_notes.json) cannot move
# while the modiolus tendons are tight, and Home pushed it into them until it
# tripped Overload (FAULT SERVO 0x20 at -540 mA, 2026-10-15). Move the jaw with
# the jaw sweep, which gives it room, or by single steps.
HOME_SKIPPED_SERVOS = (7,)

# Servos no tool moves or uses. A skipped servo stays in the profile, so
# TUNING and ENABLED still switch its torque on and hold it where it is
# (every active servo is held); the tools only never step or command it.
# Empty since 2026-11-08: the eyelids (5, 6) were fixed and are tuned again.
SKIPPED_SERVOS: Tuple[int, ...] = ()


def home_moves_over_one_turn(targets: Dict[int, int],
                             state: ControllerState) -> Dict[int, int]:
    """Servo -> distance (ticks) for every Home target more than one turn
    from where the servo is. No Home may move that far: one horn turn is
    about 51 mm of line (r = 8.1 mm), and a distance that large has so far
    meant the servo's turn count changed (after a servo power cycle,
    2026-10-15), not that the horn really turned."""
    one_turn = headctl_one_turn_ticks()
    return {index: tick - state.servos[index].present_tick
            for index, tick in sorted(targets.items())
            if abs(tick - state.servos[index].present_tick) > one_turn}


# Home waits until the servo is this close to its goal before the next step,
# the same window as the firmware's smallest tuning lag limit.
HOME_SETTLE_TICKS = 20
# A servo whose steps are refused this many times in a row is left where it
# is: it is at the load limit or the one-turn limit, and retrying cannot help.
HOME_MAX_REFUSALS = 25
EAGAIN_RESULT, EBUSY_RESULT = -11, -16


# Home has arrived when the servo itself, not just its goal, is this close.
HOME_ARRIVE_TICKS = 5
# Drive steps refused by the push cap this many fresh samples in a row: the
# servo cannot reach home within the load limit.
HOME_MAX_CAPPED_SAMPLES = 10


def tuning_max_lead_ticks(current_limit_ma: int) -> int:
    """The firmware's tuning lag limit (head_state_tuning_step): the limit
    divided by HEAD_TUNING_LAG_CURRENT_MA_PER_TICK, at least
    HEAD_TUNING_MIN_LAG_TICKS (include/head.h)."""
    return max(20, current_limit_ma // 3)


class HomeMover:
    """Drives servos onto their home ticks with ordinary TUNING_STEPs.

    It closes the loop on where the servo is, not on its goal: with Position
    I = 0 a servo holding a load sits short of its goal by about load / 3
    ticks, so a goal at home leaves a loaded servo short of home, and waiting
    for it to "catch up" never ends (2026-10-12, the jaw). After each fresh
    sample the goal moves by the servo's remaining error, at most
    HEAD_TUNING_MAX_STEP_TICKS, until the servo is within HOME_ARRIVE_TICKS of
    home; it then holds there with whatever load that takes. The goal never
    leads the servo by more than the firmware's tuning lag limit, which also
    paces long moves; if that is not enough, the servo is given up and the
    load it needed is reported. Back-off at the load limit (BackOff) still
    applies on top.

    With several servos each gets one step in turn, so co-actuated tendons come
    back roughly together. Every firmware check still applies."""

    def __init__(self, targets: Dict[int, int], max_lead_ticks: int = 20) -> None:
        self.targets = dict(targets)
        self.order = sorted(self.targets)
        self.max_lead_ticks = max_lead_ticks
        self.refusals: Dict[int, int] = {index: 0 for index in self.order}
        self.given_up: Dict[int, int] = {}
        self.arrived: set[int] = set()
        self.notices: List[str] = []
        self._capped: Dict[int, int] = {}
        self._next = 0
        # STATE comes about every 10 ms. A sample older than a servo's last
        # step answer still shows the goal before that step, and stepping from
        # it would overshoot, so that servo waits for a newer one. Other
        # servos can step meanwhile.
        self._last_answer_s: Dict[int, float] = {}

    @property
    def remaining(self) -> List[int]:
        return [index for index in self.order if index not in self.given_up]

    def is_done(self, state: Optional[ControllerState]) -> bool:
        return state is not None and all(index in self.arrived for index in self.remaining)

    def _drive_step(self, servo: ServoSample, target: int) -> Optional[int]:
        error = target - servo.present_tick
        if abs(error) <= HOME_ARRIVE_TICKS:
            self.arrived.add(servo.index)
            return None
        limit = headctl.TUNING_MAX_STEP_TICKS
        step = max(-limit, min(limit, error))
        lead_after = servo.goal_tick + step - servo.present_tick
        if abs(lead_after) > self.max_lead_ticks:
            capped_goal = servo.present_tick + (self.max_lead_ticks if lead_after > 0
                                                else -self.max_lead_ticks)
            step = capped_goal - servo.goal_tick
            if step == 0 or (step > 0) != (error > 0):
                self._capped[servo.index] = self._capped.get(servo.index, 0) + 1
                if self._capped[servo.index] >= HOME_MAX_CAPPED_SAMPLES:
                    self.given_up[servo.index] = EBUSY_RESULT
                    self.notices.append(
                        f"Servo {servo.index} cannot reach home within the load limit: it "
                        f"stops {abs(error)} ticks short while pulling {abs(servo.current_ma)} mA. "
                        "It is left holding there.")
                return None
        self._capped[servo.index] = 0
        return step

    def next_step(self, state: ControllerState) -> Optional[tuple[int, int]]:
        """(servo, step_ticks) to send now, or None to wait."""
        candidates = [index for index in self.remaining if index not in self.arrived]
        for offset in range(len(candidates)):
            index = candidates[(self._next + offset) % len(candidates)]
            if state.received_monotonic_s <= self._last_answer_s.get(index, float("-inf")):
                continue
            step = self._drive_step(state.servos[index], self.targets[index])
            if not step:
                continue
            self._next = (self._next + offset + 1) % max(1, len(candidates))
            return index, step
        return None

    def on_result(self, index: int, result: int, answered_s: float) -> Optional[str]:
        """Record a step answer (with the link clock time it was handled);
        returns a message when a servo is given up."""
        self._last_answer_s[index] = answered_s
        if index not in self.refusals or index in self.given_up:
            return None
        if result == 0:
            self.refusals[index] = 0
            return None
        if result in (EAGAIN_RESULT, EBUSY_RESULT):
            self.refusals[index] += 1
            if self.refusals[index] < HOME_MAX_REFUSALS:
                return None
        self.given_up[index] = result
        return f"Servo {index} stopped before home: {step_result_text(result)}"


def read_operating_currents(device, active_mask: int) -> Dict[int, int]:
    """Servo -> operating_current_ma from the board's profile."""
    return {index: int(headctl.decode_slot(device.get(headctl.GET_SLOT, headctl.SLOT,
                                                      bytes((index,))))["operating_current_ma"])
            for index in range(headctl.HEAD_SERVO_COUNT) if (active_mask >> index) & 1}


def tuning_caps(limit_ma: int, operating_ma: Dict[int, int]) -> Dict[int, int]:
    """Each servo's tuning Goal Current: the requested limit, but never more
    than its own operating_current_ma (head_state_tuning_limit_ma in
    src/state_machine.c, from 2026-10-20)."""
    return {index: min(limit_ma, current) for index, current in operating_ma.items()}


# The jaw may peak high (servo 7 got 650 mA on 2026-10-20) but must not stay
# there: on 2026-10-13 it tripped Overload within 6-12 s at 450-517 mA and
# held about 79 C at a steady 320 mA. Limits chosen with the operator.
JAW_HOLD_MA, JAW_HOLD_S = 400, 2.0
JAW_PEAK_MA, JAW_PEAK_S = 600, 0.5


class JawGuard:
    """Watches the jaw's load over time: "back off" once it has stayed over
    hold_ma for hold_s, "torque off" once over peak_ma for peak_s."""

    def __init__(self, jaw: int, hold_ma: int = JAW_HOLD_MA, hold_s: float = JAW_HOLD_S,
                 peak_ma: int = JAW_PEAK_MA, peak_s: float = JAW_PEAK_S) -> None:
        self.jaw, self.hold_ma, self.hold_s, self.peak_ma, self.peak_s = \
            jaw, hold_ma, hold_s, peak_ma, peak_s
        self._over_hold_since: Optional[float] = None
        self._over_peak_since: Optional[float] = None

    def observe(self, state: ControllerState) -> Optional[str]:
        now = state.received_monotonic_s
        load = abs(state.servos[self.jaw].current_ma)
        if load <= self.peak_ma:
            self._over_peak_since = None
        elif self._over_peak_since is None:
            self._over_peak_since = now
        if load <= self.hold_ma:
            self._over_hold_since = None
        elif self._over_hold_since is None:
            self._over_hold_since = now
        if self._over_peak_since is not None and now - self._over_peak_since >= self.peak_s:
            return "torque off"
        if self._over_hold_since is not None and now - self._over_hold_since >= self.hold_s:
            return "back off"
        return None

    def reset(self) -> None:
        self._over_hold_since = self._over_peak_since = None


# Jaw sweep -----------------------------------------------------------------
# The jaw (7) cannot open unless the tendons that lift the modiolus (right
# 11/14, left 16/19) pay out to give it room, and they must reel in again when
# it closes. The tendon servos do not feel the jaw push: on 2026-10-15 the jaw
# load rose from 32 to 600 mA while their loads changed by 10 mA at most
# (tuning_runs/20261015T221931Z). So the jaw's own load is the sensor: while
# it moves, the jaw pushes only a little more than it held at the last stage,
# and while it is held there the release tendons pay out. At each stage the jaw holds and every tendon is pulled back
# to a preload, which a servo does feel when it pulls its own line. Each
# settled stage is one row of the sweep file.

# HEAD_TUNING_LAG_CURRENT_MA_PER_TICK (include/head.h): load per tick of lag.
LAG_MA_PER_TICK = 3
# Part of the load error corrected per step. Below 1 so that a stiff tendon,
# where the load rises by about LAG_MA_PER_TICK per goal tick, does not
# overshoot; a soft one just takes more steps.
TENSION_GAIN = 0.5
TENSION_MAX_STEP_TICKS = 10
# The jaw moves this much at most per step.
JAW_MAX_STEP_TICKS = 5
# A release tendon pays out this much per sample while the jaw is held at
# its push limit.
RELEASE_STEP_TICKS = 2
# The move stops (and holds) when every release tendon has paid out this much
# since the jaw last gained a tick toward its target: the release set is not
# what blocks the jaw. On 2026-10-18 (20261018T205016Z) 11/14/16/19 paid out
# about 200 ticks each in 9 s with the jaw still, while servo 8 took in the
# slack. Raised from 60 to 150 ticks (about 1.9 mm of line on the 8.1 mm horn)
# on 2026-10-20 with the operator: a horn only starts to turn once its goal
# leads it by about 50 ticks (friction), so 60 stopped the jaw -40 slice of
# the range finder about half the time.
NO_PROGRESS_RELEASE_TICKS = 150
# In a hold every tendon reaches the preload from below: it pays out until
# it reads no more than preload - band, then only pulls in. Friction holds a
# horn's reading anywhere in a band of about 20-50 mA (first hardware sweep,
# 20261015T231809Z: 20 mA reached with no horn motion; hanging-mass test
# 2026-09-22), so a reading reached from one side is repeatable where one
# reached from either side is not. A line that went slack reads low and is
# pulled in.
# At the pretension (stage 0) a tendon that started under the band but whose
# horn took in less than this was not shown to move its line: its load may be
# friction (servo 9 moved 5 ticks to "56 mA" on 2026-10-15, the others
# 120-430). A tendon already in band at the start has nothing to take in.
PRETENSION_MOTION_MIN_TICKS = 20
# Fresh samples in a row (STATE comes every 10 ms, so 5 samples is about 50 ms;
# the comment said 100 ms until 2026-10-20) with the jaw arrived and
# every tendon in band before a stage is recorded.
SWEEP_SETTLE_SAMPLES = 5
SWEEP_STAGE_TIMEOUT_S = 30.0
SWEEP_FORMAT = "head-jaw-sweep-4"


def parse_pull_directions(text: str) -> Dict[int, int]:
    """'8+ 11- 14+' -> {8: 1, 11: -1, 14: 1}: the tick direction that tightens
    each tendon. Commas or spaces separate the entries."""
    pull: Dict[int, int] = {}
    for token in text.replace(",", " ").split():
        if len(token) < 2 or token[-1] not in "+-" or not token[:-1].isdigit():
            raise ValueError(f"'{token}' is not a servo and a direction, like 8+ or 11-")
        index = int(token[:-1])
        if index in pull:
            raise ValueError(f"servo {index} is given twice")
        pull[index] = 1 if token[-1] == "+" else -1
    if not pull:
        raise ValueError("no tendon servos given")
    return pull


def parse_servo_list(text: str) -> List[int]:
    """'11 14, 16 19' -> [11, 14, 16, 19]."""
    servos: List[int] = []
    for token in text.replace(",", " ").split():
        if not token.isdigit():
            raise ValueError(f"'{token}' is not a servo number")
        if int(token) in servos:
            raise ValueError(f"servo {token} is given twice")
        servos.append(int(token))
    return servos


@dataclass
class SweepSettings:
    jaw: int
    jaw_open_tick: int
    stages: int
    # Tendon servo -> +1 or -1, the tick direction that tightens it.
    pull: Dict[int, int]
    # Tendons that pay out to give the jaw room when it opens. The others are
    # only held at the preload.
    release: List[int]
    preload_ma: int
    band_ma: int
    # While moving, the jaw pushes toward its target with at most about this
    # much more load than it held at the last recorded stage.
    jaw_push_ma: int
    abort_ma: int
    max_travel_ticks: int
    # Tendons left slack on the way open: in its holds they are not pulled
    # back to the preload (they only pay out if loaded over the band), so
    # they do not pull the jaw shut. At the pretension and on the way back
    # they are re-tensioned like the others. 9 holds the jaw (2026-10-18).
    defer_on_open: List[int] = field(default_factory=list)
    # The jaw's own abort load; None means abort_ma, like the lines. With
    # per-servo caps (2026-10-20) the jaw may be allowed more than the lines.
    jaw_abort_ma: Optional[int] = None

    @property
    def jaw_abort(self) -> int:
        return self.abort_ma if self.jaw_abort_ma is None else self.jaw_abort_ma


def check_sweep_settings(settings: SweepSettings, active: Sequence[int],
                         limit_ma: int, caps: Optional[Dict[int, int]] = None) -> List[str]:
    """Reasons the sweep must not start; empty when it may. caps (servo ->
    tuning cap) lets the jaw and the lines have different limits."""
    problems = []
    line_limit = min((caps or {}).get(index, limit_ma) for index in settings.pull) \
        if settings.pull else limit_ma
    jaw_limit = (caps or {}).get(settings.jaw, limit_ma)
    for index in [settings.jaw, *settings.pull]:
        if index not in active:
            problems.append(f"Servo {index} is not active on this board.")
        if index in SKIPPED_SERVOS:
            problems.append(f"Servo {index} is skipped (faulty eyelid); leave it out.")
    if settings.jaw in settings.pull:
        problems.append(f"Servo {settings.jaw} is the jaw; it cannot also be a tendon.")
    for index in settings.release:
        if index not in settings.pull:
            problems.append(f"Release servo {index} is not in the tendon list.")
    for index in settings.defer_on_open:
        if index not in settings.pull:
            problems.append(f"Slack-while-opening servo {index} is not in the tendon list.")
    if settings.stages < 1:
        problems.append("Use at least 1 stage.")
    if settings.band_ma < 1:
        problems.append("The band must be at least 1 mA.")
    if settings.preload_ma - settings.band_ma <= 0:
        # Otherwise a slack tendon (about 0 mA) would count as in band.
        problems.append("The preload must be larger than the band.")
    if settings.jaw_push_ma < LAG_MA_PER_TICK:
        problems.append(f"The jaw push limit must be at least {LAG_MA_PER_TICK} mA.")
    if settings.abort_ma <= settings.preload_ma + settings.band_ma or \
            settings.jaw_abort <= settings.jaw_push_ma:
        problems.append("The abort load must be above preload + band and above the jaw "
                        "push limit.")
    if settings.abort_ma >= BACKOFF_LOAD_FRACTION * line_limit:
        problems.append(f"The abort load must be below {BACKOFF_LOAD_FRACTION:.0%} of the "
                        f"load limit ({line_limit} mA), so the sweep stops before back-off.")
    if settings.jaw_abort_ma is not None and (
            settings.jaw_abort_ma >= BACKOFF_LOAD_FRACTION * jaw_limit or
            settings.jaw_abort_ma <= settings.jaw_push_ma):
        problems.append(f"The jaw abort load must be above the jaw push limit and below "
                        f"{BACKOFF_LOAD_FRACTION:.0%} of the jaw's limit ({jaw_limit} mA).")
    if not 1 <= settings.max_travel_ticks <= headctl_one_turn_ticks():
        problems.append(f"The tendon travel limit must be 1 to {headctl_one_turn_ticks()} "
                        "ticks (no more than one turn).")
    return problems


def sweep_jaw_targets(start_tick: int, open_tick: int, stages: int) -> List[int]:
    """Jaw tick per stage: the start (pretension only), `stages` equal steps
    to open, and the same steps back to the start."""
    out = [start_tick + round((open_tick - start_tick) * stage / stages)
           for stage in range(stages + 1)]
    return out + out[-2::-1]


# The sweep boxes as last used, so the panel opens with them filled. The jaw
# open tick is not kept: it is filled from where the jaw is at each Start.
SWEEP_BOXES_FILE = "jaw_sweep_settings.json"
SWEEP_BOXES_FORMAT = "head-jaw-sweep-boxes-1"
# Used until the first sweep is started: the values confirmed on the head on
# 2026-10-15 (pull directions from the operator, preload above the friction
# band of the first run, 20261015T231809Z_jawsweep.json). Servo 11 was given
# as "-" until 2026-10-18; the operator then found it tightens toward "+".
SWEEP_BOX_DEFAULTS = {"jaw": "7", "stages": "1", "pull": "8- 11+ 14+ 9- 15- 19-",
                      "release": "8 9 11 14 15 19", "preload_ma": "60", "band_ma": "10",
                      "jaw_push_ma": "60", "abort_ma": "280", "max_travel_ticks": "600",
                      "defer_on_open": "8 9"}


def load_sweep_boxes(directory: Path) -> Dict[str, str]:
    """The sweep box texts last used (the records' config/), else
    SWEEP_BOX_DEFAULTS."""
    path = records_layout.config_dir(directory) / SWEEP_BOXES_FILE
    boxes = dict(SWEEP_BOX_DEFAULTS)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("format") == SWEEP_BOXES_FORMAT:
            boxes.update({key: str(value) for key, value in data["boxes"].items()
                          if key in SWEEP_BOX_DEFAULTS})
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        pass
    return boxes


def save_sweep_boxes(directory: Path, boxes: Dict[str, str]) -> None:
    write_json_atomically(records_layout.config_dir(directory) / SWEEP_BOXES_FILE,
                          {"format": SWEEP_BOXES_FORMAT,
                           "boxes": {key: boxes[key] for key in SWEEP_BOX_DEFAULTS}})


def new_sweep_path(directory: Path) -> Path:
    # Not "*_tuning.json", so load_earlier_marks and propose_profile.py skip it.
    return Path(directory) / f"{records_layout.local_stamp()}_jawsweep.json"


class JawSweep:
    """Opens the jaw in stages and closes it again, with the tendons following.

    Each stage has two phases.

    move: the jaw steps toward the stage target by at most JAW_MAX_STEP_TICKS.
    Toward the target, its goal never leads it by more than
    jaw_push_ma / LAG_MA_PER_TICK ticks beyond the lead it had toward the
    target at the last recorded stage, and its load toward the target never
    rises more than jaw_push_ma above the load it had then toward the target
    (none, when it was held the other way): it carries the preload, plus at
    most about jaw_push_ma.
    When it is held there on the way open, each release tendon pays out
    RELEASE_STEP_TICKS per fresh sample until the jaw moves again. If every
    release tendon has paid out NO_PROGRESS_RELEASE_TICKS since the jaw last
    gained a tick, the sweep stops and holds: something else blocks it. Release
    tendons are never reeled in during a move; the other tendons are held at
    the preload.

    hold: the jaw holds the target (its goal may lead it by up to
    abort_ma / LAG_MA_PER_TICK ticks, as the preload needs). Every tendon
    approaches the preload from below: it pays out (by its excess over
    preload - band, / LAG_MA_PER_TICK ticks) until it reads no more than
    preload - band, then only pulls in (by TENSION_GAIN x error /
    LAG_MA_PER_TICK ticks) until it reads at least preload - band / 2. If it
    later reads over preload + band it unloads and comes up again; under
    preload - band, it pulls in again. Tendon load is Present Current in the
    pull direction; its sign follows the lag (goal minus present), as the
    tuning records show. A line can only pull, so a reading against the pull
    direction is the servo pushing its horn out against friction (servo 14,
    -11 mA, horn still, 2026-10-15): it counts as no line load, never as an
    error. The rows keep the signed readings. The stage is recorded when the jaw is still within
    HOME_ARRIVE_TICKS and every tendon has finished its approach and been in
    band for SWEEP_SETTLE_SAMPLES samples. The row keeps how far each horn
    took in line during the hold; at stage 0 the tendons that started under
    the band and took in less than PRETENSION_MOTION_MIN_TICKS are listed as
    unverified.

    Stage 0 is a hold at the start: pretension only.

    On the way open, the defer_on_open tendons are left slack: in holds and
    moves they only pay out (when over preload + half the band), never pull
    in, and a stage does not wait for them to be in band. Their rows list
    them under "slack". At the pretension and on the way back they are
    treated like the others.

    It stops and holds on a refused step, a stage timeout, or a tendon that
    would travel more than max_travel_ticks from where the sweep started
    (which is also what a wrong pull direction does: the horn unwinds and
    never takes load). It stops and asks for torque off when the jaw or a
    tendon reaches abort_ma in either direction. Every firmware check still applies, and
    BackOff still acts at the load limit."""

    def __init__(self, path: Path, settings: SweepSettings, state: ControllerState,
                 now_s: float) -> None:
        self.path = Path(path)
        self.settings = settings
        self.jaw_push_ticks = max(1, settings.jaw_push_ma // LAG_MA_PER_TICK)
        self.jaw_hold_lead_ticks = max(1, settings.jaw_abort // LAG_MA_PER_TICK)
        jaw = state.servos[settings.jaw]
        # The jaw's lead and load at the last recorded stage: what it needs to
        # carry the preload there. A move may push only a little beyond it.
        self._jaw_lead_ref = jaw.goal_tick - jaw.present_tick
        self._jaw_current_ref = jaw.current_ma
        self.started_utc = datetime.now(timezone.utc).isoformat()
        self.start_ticks = {index: state.servos[index].present_tick
                            for index in [settings.jaw, *settings.pull]}
        self.targets = sweep_jaw_targets(self.start_ticks[settings.jaw],
                                         settings.jaw_open_tick, settings.stages)
        self.open_sign = 1 if settings.jaw_open_tick >= self.start_ticks[settings.jaw] else -1
        self.stage = 0
        self.phase = "hold"
        self.rows: List[Dict[str, Any]] = []
        self.done = False
        self.stopped: Optional[str] = None
        self.torque_off = False
        self.refusals: Dict[int, int] = {}
        self.jaw_blocked = False
        self._approach: Dict[int, str] = {}
        self._hold_start: Dict[int, int] = {}
        # Tendons that start under the band: only these must take in line at
        # the pretension.
        self._started_slack = {index for index in settings.pull
                               if self.tension_ma(state.servos[index])
                               < settings.preload_ma - settings.band_ma}
        # Jaw progress toward the move target, and what each release tendon
        # has paid out since the jaw last gained a tick.
        self._jaw_best_progress = 0
        self._released_since_progress: Dict[int, int] = {}
        self._move_start_jaw = jaw.present_tick
        self._start_hold(state)
        self._settled = 0
        self._stage_started_s = now_s
        self._last_observed_s = float("-inf")
        self._last_answer_s: Dict[int, float] = {}
        self._next = 0

    @property
    def active(self) -> bool:
        return not self.done and self.stopped is None

    @property
    def jaw_target(self) -> int:
        return self.targets[self.stage]

    @property
    def leg(self) -> str:
        if self.stage == 0:
            return "start"
        return "open" if self.stage <= self.settings.stages else "close"

    def tension_ma(self, servo: ServoSample) -> int:
        """Line load: the reading in the pull direction, 0 when against it."""
        return max(0, servo.current_ma * self.settings.pull[servo.index])

    def _in_band(self, servo: ServoSample) -> bool:
        return abs(self.tension_ma(servo) - self.settings.preload_ma) <= self.settings.band_ma

    def _start_hold(self, state: ControllerState) -> None:
        """Every tendon starts its approach to the preload from below."""
        self._approach = {index: "unload" for index in self.settings.pull}
        self._hold_start = {index: state.servos[index].present_tick
                            for index in self.settings.pull}

    def _slack_now(self, index: int) -> bool:
        """Left slack in this stage: a defer_on_open tendon on the way open."""
        return self.leg == "open" and index in self.settings.defer_on_open

    def _advance_approach(self, state: ControllerState) -> None:
        settings = self.settings
        low = settings.preload_ma - settings.band_ma
        for index in settings.pull:
            tension = self.tension_ma(state.servos[index])
            phase = self._approach[index]
            if phase == "unload" and tension <= low:
                self._approach[index] = "load"
            elif phase == "load" and tension >= settings.preload_ma - settings.band_ma // 2:
                self._approach[index] = "done"
            elif phase == "done" and tension > settings.preload_ma + settings.band_ma:
                self._approach[index] = "unload"
            elif phase == "done" and tension < low:
                self._approach[index] = "load"

    def _jaw_arrived(self, state: ControllerState) -> bool:
        return abs(state.servos[self.settings.jaw].present_tick - self.jaw_target) <= \
            HOME_ARRIVE_TICKS

    def _stop(self, reason: str, torque_off: bool = False) -> None:
        if self.stopped is None:
            self.stopped = reason
            self.torque_off = torque_off

    def observe(self, state: ControllerState) -> Optional[Dict[str, Any]]:
        """Check one STATE sample; returns the row when a stage is recorded."""
        if not self.active or state.received_monotonic_s <= self._last_observed_s:
            return None
        self._last_observed_s = state.received_monotonic_s
        settings = self.settings
        for index in [settings.jaw, *settings.pull]:
            servo = state.servos[index]
            abort = settings.jaw_abort if index == settings.jaw else settings.abort_ma
            if abs(servo.current_ma) >= abort:
                self._stop(f"Servo {index} reached {abs(servo.current_ma)} mA, at or over the "
                           f"abort load of {abort} mA.", torque_off=True)
                return None
        self.jaw_blocked = self.phase == "move" and self._jaw_step(state) == 0 and \
            not self._jaw_arrived(state)
        if state.received_monotonic_s - self._stage_started_s > SWEEP_STAGE_TIMEOUT_S:
            blocked = (f" The jaw was held at its push limit ({settings.jaw_push_ma} mA)"
                       + ("; no release tendon could give it more room."
                          if self.leg == "open" else " while closing.")
                       if self.jaw_blocked else "")
            self._stop(f"Stage {self.stage} ({self.phase}) did not finish in "
                       f"{SWEEP_STAGE_TIMEOUT_S:.0f} s.{blocked}")
            return None
        if self.phase == "move":
            if self._jaw_arrived(state):
                self.phase = "hold"
                self._start_hold(state)
                return None
            progress = ((state.servos[settings.jaw].present_tick - self._move_start_jaw)
                        * (1 if self.jaw_target >= self._move_start_jaw else -1))
            if progress > self._jaw_best_progress:
                self._jaw_best_progress = progress
                self._released_since_progress = {}
            if settings.release and all(
                    self._released_since_progress.get(index, 0) >= NO_PROGRESS_RELEASE_TICKS
                    for index in settings.release):
                self._stop(f"The jaw did not move while every release tendon "
                           f"({', '.join(map(str, settings.release))}) paid out "
                           f"{NO_PROGRESS_RELEASE_TICKS} ticks: something else blocks it.")
            return None
        self._advance_approach(state)
        settled = (self._jaw_arrived(state) and
                   all(self.tension_ma(state.servos[index])
                       <= settings.preload_ma + settings.band_ma
                       if self._slack_now(index) else
                       self._approach[index] == "done" and self._in_band(state.servos[index])
                       for index in settings.pull))
        self._settled = self._settled + 1 if settled else 0
        if self._settled < SWEEP_SETTLE_SAMPLES:
            return None
        row = {"stage": self.stage, "leg": self.leg, "jaw_target_tick": self.jaw_target,
               "utc": datetime.now(timezone.utc).isoformat(), "mcu_time_ms": state.mcu_time_ms,
               "servos": {str(index): {"goal_tick": state.servos[index].goal_tick,
                                       "present_tick": state.servos[index].present_tick,
                                       "current_ma": state.servos[index].current_ma}
                          for index in [settings.jaw, *settings.pull]}}
        # Line taken in during this hold (ticks, + = pulled in).
        taken_in = {index: (state.servos[index].present_tick - self._hold_start[index])
                    * settings.pull[index] for index in settings.pull}
        row["taken_in_ticks"] = {str(index): ticks for index, ticks in sorted(taken_in.items())}
        row["slack"] = sorted(index for index in settings.pull if self._slack_now(index))
        row["unverified"] = sorted(index for index, ticks in taken_in.items()
                                   if self.stage == 0 and index in self._started_slack
                                   and ticks < PRETENSION_MOTION_MIN_TICKS)
        self.rows.append(row)
        self._settled = 0
        if self.stage == len(self.targets) - 1:
            self.done = True
        else:
            self.stage += 1
            self.phase = "move"
            self._stage_started_s = state.received_monotonic_s
            self._move_start_jaw = state.servos[settings.jaw].present_tick
            self._jaw_best_progress = 0
            self._released_since_progress = {}
            jaw = state.servos[settings.jaw]
            self._jaw_lead_ref = jaw.goal_tick - jaw.present_tick
            self._jaw_current_ref = jaw.current_ma
        return row

    @staticmethod
    def _capped(servo: ServoSample, step: int, max_lead_ticks: int) -> int:
        """The step cut so the goal leads the servo by at most max_lead_ticks;
        0 when nothing in the wanted direction is left."""
        lead_after = servo.goal_tick + step - servo.present_tick
        if abs(lead_after) <= max_lead_ticks:
            return step
        capped = servo.present_tick + (max_lead_ticks if lead_after > 0
                                       else -max_lead_ticks) - servo.goal_tick
        return capped if capped and (capped > 0) == (step > 0) else 0

    def _travel_ok(self, servo: ServoSample, step: int) -> bool:
        travel = servo.goal_tick + step - self.start_ticks[servo.index]
        if abs(travel) <= self.settings.max_travel_ticks:
            return True
        self._stop(f"Servo {servo.index} would move more than "
                   f"{self.settings.max_travel_ticks} ticks from where the sweep started "
                   f"(it reads {self.tension_ma(servo)} mA). Check its pull direction and "
                   "the travel limit.")
        return False

    def _tendon_step(self, servo: ServoSample, pay_out_only: bool = False) -> int:
        error = self.settings.preload_ma - self.tension_ma(servo)
        # Aim at half the band, so a stage is not recorded at its edge.
        if abs(error) <= max(1, self.settings.band_ma // 2) or (pay_out_only and error > 0):
            return 0
        size = max(1, min(TENSION_MAX_STEP_TICKS,
                          round(TENSION_GAIN * abs(error) / LAG_MA_PER_TICK)))
        step = size * (1 if error > 0 else -1) * self.settings.pull[servo.index]
        return step if self._travel_ok(servo, step) else 0

    def _approach_step(self, servo: ServoSample) -> int:
        """A hold step: pay out while unloading, pull in while loading."""
        settings = self.settings
        tension = self.tension_ma(servo)
        phase = self._approach[servo.index]
        if phase == "unload":
            excess = tension - (settings.preload_ma - settings.band_ma)
            if excess <= 0:
                return 0
            size = -min(TENSION_MAX_STEP_TICKS, max(1, -(-excess // LAG_MA_PER_TICK)))
        elif phase == "load":
            error = settings.preload_ma - tension
            if error <= 0:
                return 0
            size = max(1, min(TENSION_MAX_STEP_TICKS,
                              round(TENSION_GAIN * error / LAG_MA_PER_TICK)))
        else:
            return 0
        step = size * settings.pull[servo.index]
        return step if self._travel_ok(servo, step) else 0

    def _release_step(self, servo: ServoSample) -> int:
        step = -RELEASE_STEP_TICKS * self.settings.pull[servo.index]
        if not self._travel_ok(servo, step):
            return 0
        self._released_since_progress[servo.index] = (
            self._released_since_progress.get(servo.index, 0) + RELEASE_STEP_TICKS)
        return step

    def _jaw_step(self, state: ControllerState) -> int:
        """The jaw's next step toward the target, 0 when it is there or, in a
        move, held at its push limit."""
        jaw = state.servos[self.settings.jaw]
        error = self.jaw_target - jaw.present_tick
        if abs(error) <= HOME_ARRIVE_TICKS:
            return 0
        step = max(-JAW_MAX_STEP_TICKS, min(JAW_MAX_STEP_TICKS, error))
        if self.phase == "hold":
            return self._capped(jaw, step, self.jaw_hold_lead_ticks)
        direction = 1 if error > 0 else -1
        # What it held toward the target at the last stage counts as carried
        # load; held the other way (the lines help it), it counts as none, so
        # the load falling as the lines go slack is not taken for a push.
        carried_ma = max(0, self._jaw_current_ref * direction)
        if jaw.current_ma * direction - carried_ma >= self.settings.jaw_push_ma:
            return 0
        room = (max(0, self._jaw_lead_ref * direction) + self.jaw_push_ticks
                - (jaw.goal_tick - jaw.present_tick) * direction)
        return direction * min(abs(step), room) if room > 0 else 0

    def _step_for(self, index: int, state: ControllerState) -> int:
        servo = state.servos[index]
        if index == self.settings.jaw:
            return self._jaw_step(state)
        if self.phase == "hold" and not self._slack_now(index):
            return self._approach_step(servo)
        if self.phase == "move" and index in self.settings.release and \
                self.jaw_blocked and self.leg == "open":
            return self._release_step(servo)
        if self._slack_now(index) or index in self.settings.release:
            return self._tendon_step(servo, pay_out_only=True)
        return self._tendon_step(servo)

    def next_step(self, state: ControllerState) -> Optional[tuple[int, int]]:
        """(servo, step_ticks) to send now, or None to wait. Each servo waits
        for a sample newer than its last answer; the others take turns."""
        if not self.active:
            return None
        order = [*sorted(self.settings.pull), self.settings.jaw]
        for offset in range(len(order)):
            index = order[(self._next + offset) % len(order)]
            if state.received_monotonic_s <= self._last_answer_s.get(index, float("-inf")):
                continue
            step = self._step_for(index, state)
            if not self.active:
                return None
            if step:
                self._next = (self._next + offset + 1) % len(order)
                return index, step
        return None

    def on_result(self, index: int, result: int, answered_s: float) -> None:
        self._last_answer_s[index] = answered_s
        if result == 0:
            self.refusals[index] = 0
            return
        if result in (EAGAIN_RESULT, EBUSY_RESULT):
            self.refusals[index] = self.refusals.get(index, 0) + 1
            if self.refusals[index] < HOME_MAX_REFUSALS:
                return
        self._stop(f"Servo {index}: {step_result_text(result)}")

    def to_json(self) -> Dict[str, Any]:
        settings = self.settings
        return {
            "format": SWEEP_FORMAT,
            "started_utc": self.started_utc,
            "settings": {"jaw": settings.jaw, "jaw_open_tick": settings.jaw_open_tick,
                         "stages": settings.stages,
                         "pull": {str(index): sign for index, sign in sorted(settings.pull.items())},
                         "release": settings.release,
                         "preload_ma": settings.preload_ma, "band_ma": settings.band_ma,
                         "jaw_push_ma": settings.jaw_push_ma, "abort_ma": settings.abort_ma,
                         "max_travel_ticks": settings.max_travel_ticks,
                         "defer_on_open": settings.defer_on_open,
                         "jaw_abort_ma": settings.jaw_abort},
            "start_ticks": {str(index): tick for index, tick in sorted(self.start_ticks.items())},
            "jaw_targets": self.targets,
            "rows": self.rows,
            "finished": self.done,
            "stopped": self.stopped,
        }

    def save(self) -> None:
        write_json_atomically(self.path, self.to_json())


class TuningLink:
    """Non-blocking controller session for a GUI timer.

    Call poll() often. It reads frames, matches answers to requests, renews the
    lease, and reports a lost lease. Nothing here retries a step: one click
    sends one request, and the answer is shown.
    """

    def __init__(self, device: Any, clock=time.monotonic) -> None:
        self.device = device
        self.clock = clock
        self.token = 0
        self.state: Optional[ControllerState] = None
        self.capabilities = 0
        self.control_lost_reason: Optional[str] = None
        self._renew_due_s = 0.0
        self._renew_pending: Optional[tuple[int, float]] = None
        self._step_pending: Optional[tuple[int, int, int, float]] = None
        self.step_results: List[tuple[int, int, int]] = []
        # Called once per received STATE, with the Present PWM sent with it
        # (None when the firmware has no PRESENT_PWM or that frame was lost).
        # poll() reads many frames per call and self.state keeps only the
        # newest, so this is the only place that sees every sample.
        self.sample_listener: Optional[
            Callable[[ControllerState, Optional[tuple[int, ...]]], Any]] = None
        # A STATE waiting for its PRESENT_PWM, which the firmware sends next.
        self._unpaired_state: Optional[ControllerState] = None
        # Newest Present PWM per servo slot, for display; None until the
        # first PRESENT_PWM (or never, with older firmware).
        self.present_pwm_raw: Optional[tuple[int, ...]] = None

    def connect(self) -> None:
        reply = self.device.hello()
        self.capabilities = reply["capabilities"]
        if not self.capabilities & headctl.CAP_TUNING:
            raise RuntimeError("this firmware has no tuning mode; flash the new build")

    def start(self, current_limit_ma: int) -> None:
        self.token = headctl.acquire(self.device)
        headctl.start_tuning(self.device, self.token, current_limit_ma)
        self.control_lost_reason = None
        self._renew_due_s = self.clock() + LEASE_RENEW_PERIOD_S

    @property
    def has_control(self) -> bool:
        return self.token != 0 and self.control_lost_reason is None

    @property
    def step_in_flight(self) -> bool:
        return self._step_pending is not None

    def state_is_fresh(self) -> bool:
        return (self.state is not None and
                self.clock() - self.state.received_monotonic_s <= STATE_FRESH_S)

    def request_step(self, servo_index: int, step_ticks: int) -> None:
        if step_ticks == 0 or abs(step_ticks) > headctl.TUNING_MAX_STEP_TICKS:
            raise ValueError("step out of range")
        if not self.has_control or self._step_pending is not None:
            return
        transaction_id = self.device.send_confirmed(
            headctl.TUNING_STEP, struct.pack("<IBb", self.token, servo_index, step_ticks))
        self._step_pending = (transaction_id, servo_index, step_ticks, self.clock())

    def disable(self) -> None:
        """Ask for the verified torque-off path. Safe to call more than once."""
        if self.token:
            self.device.send_confirmed(headctl.DISABLE, struct.pack("<I", self.token))
        self.token = 0
        self._renew_pending = None
        self._step_pending = None

    def poll(self, timeout_s: float = 0.0) -> None:
        deadline_s = self.clock() + timeout_s
        while True:
            frame = self.device.recv(max(0.0, deadline_s - self.clock()))
            if frame is None:
                break
            self._handle(*frame)
            if self.clock() >= deadline_s:
                break
        self._service_lease()

    def _emit_sample(self, state: ControllerState,
                     present_pwm_raw: Optional[tuple[int, ...]]) -> None:
        if self.sample_listener is not None:
            self.sample_listener(state, present_pwm_raw)

    def _handle(self, kind: int, payload: bytes) -> None:
        if kind == headctl.PRESENT_PWM:
            try:
                mcu_time_ms, present_pwm_raw = decode_present_pwm(payload)
            except ValueError:
                return
            self.present_pwm_raw = present_pwm_raw
            unpaired = self._unpaired_state
            if unpaired is not None and unpaired.mcu_time_ms == mcu_time_ms:
                self._unpaired_state = None
                self._emit_sample(unpaired, present_pwm_raw)
            return
        if kind == headctl.STATE:
            try:
                self.state = decode_state(payload, self.clock())
            except ValueError:
                return
            # The USB queue keeps only the newest frame of each kind, so a
            # PRESENT_PWM can be replaced before it is sent. A STATE still
            # waiting when the next one arrives is passed on without PWM.
            if self._unpaired_state is not None:
                self._emit_sample(self._unpaired_state, None)
                self._unpaired_state = None
            if self.capabilities & headctl.CAP_PRESENT_PWM:
                self._unpaired_state = self.state
            else:
                self._emit_sample(self.state, None)
            if self.has_control and self.state.state == FAULT_STATE:
                report = hardware_error_report(self.state)
                if not report and self.state.fault_name == "SERVO":
                    # main.c also latches SERVO when the torque-off sweep sees
                    # a servo's alert bit, before telemetry shows the error.
                    report = ("no Hardware Error in the last telemetry; a servo raised its "
                              "alert bit. Run 'headctl status' before power-cycling to see which")
                self.control_lost_reason = (f"Board fault: {self.state.fault_name}"
                                            + (f" ({report})." if report else "."))
            return
        if kind not in (headctl.ACK, headctl.NACK):
            return
        result = struct.unpack_from("<b", payload, 1)[0] if len(payload) > 1 else -1
        if (self._step_pending is not None and
                self.device.ack_matches(headctl.TUNING_STEP, payload, self._step_pending[0])):
            _, servo_index, direction, _ = self._step_pending
            self._step_pending = None
            self.step_results.append((servo_index, direction, result))
        elif (self._renew_pending is not None and
              self.device.ack_matches(headctl.RENEW_LEASE, payload, self._renew_pending[0])):
            self._renew_pending = None
            if kind != headctl.ACK or result != 0:
                self.control_lost_reason = "Control lost. The board did not renew control."

    def _service_lease(self) -> None:
        now = self.clock()
        if self._step_pending is not None and now - self._step_pending[3] > ANSWER_TIMEOUT_S:
            self._step_pending = None
            self.control_lost_reason = "Control lost. The board did not answer a step."
        if not self.has_control:
            return
        if self._renew_pending is not None:
            if now - self._renew_pending[1] > ANSWER_TIMEOUT_S:
                self._renew_pending = None
                self.control_lost_reason = "Control lost. The board did not answer."
            return
        if now >= self._renew_due_s:
            transaction_id = self.device.send_confirmed(
                headctl.RENEW_LEASE, struct.pack("<I", self.token))
            self._renew_pending = (transaction_id, now)
            self._renew_due_s = now + LEASE_RENEW_PERIOD_S


ENABLED_STATE_SIM = headctl.STATE_NAMES.index("ENABLED")
JOINT_TARGETS_SIM = 5          # HEAD_MSG_JOINT_TARGETS (include/protocol.h)


class SimulatedController:
    """In-process stand-in for the controller, for --preview and tests.

    It follows the firmware's tuning rules closely enough to exercise the
    panel: START_TUNING moves HOMING_REQUIRED to TUNING, each accepted step
    moves the goal one tick, and the load grows with distance from neutral so
    the load-limit refusal can be seen. It is not a model of the hardware.
    """

    SESSION_ID = 0x5EED5EED

    # Measured on the hardware: about 3-4 mA per tick of lag (P 900, I 0).
    LAG_MA_PER_TICK = 3.5

    def __init__(self, active_servos: Dict[int, int] | None = None,
                 load_ma_per_tick: float = 0.5, clock=time.monotonic,
                 walls: Dict[int, int] | None = None) -> None:
        self.active_servos = active_servos or {10: 2, 11: 2, 12: 2}
        self.load_ma_per_tick = load_ma_per_tick
        # Optional hard stop per servo: it cannot move past this tick, and its
        # current then grows with lag up to the limit, like a blocked servo.
        self.walls = dict(walls or {})
        self.fault = 0
        self.hardware_errors: Dict[int, int] = {}
        # Optional spring per servo, (rest tick, mA per tick): the mechanism
        # pulls the servo toward its rest tick, so under load it sits short of
        # its goal, as with Position I = 0 on the hardware.
        self.springs: Dict[int, tuple[int, float]] = {}
        # Optional tendons, for the jaw sweep: servo -> (pull sign, tick where
        # the line goes taut, mA per tick of stretch, stretch per tick the jaw
        # is open). A line carries load only when stretched. Opening the jaw
        # (tendon_jaw above tendon_jaw_closed_tick) stretches every line, and
        # the jaw carries their loads. Present Current is signed like the lag.
        self.tendons: Dict[int, tuple[int, int, float, float]] = {}
        self.tendon_jaw: Optional[int] = None
        self.tendon_jaw_closed_tick = 0
        # Hidden tendons: friction in the line path keeps the horn from
        # feeling what the jaw does to the line (seen on the head, 2026-10-15).
        # The horn feels the line as it was at the jaw opening of its last
        # step inward; pulling its own line in shows the true load. The jaw
        # always feels the true line.
        self.tendon_hidden: set[int] = set()
        self._seen_opening: Dict[int, float] = {}
        # Lines on one action point: servo -> [(other servo, gain)]. Pulling
        # the other line in past its taut tick stretches this one by gain
        # ticks per tick, so a following line must pay out (face_stream).
        self.tendon_links: Dict[int, List[tuple[int, float]]] = {}
        # Optional: ticks of slack a line has before it winds back onto the
        # horn the other way (2026-10-22: paying out 600 ticks did that on the
        # head and pulled the corner up). Without an entry it stays slack.
        self.tendon_slack_len: Dict[int, float] = {}
        # ENABLED mode: each servo's profile min..max (q = 0 / q = 1), and the
        # firmware's command watchdog (100 ms, then a 50 ms hold).
        self.ranges: Dict[int, tuple[int, int]] = {index: (0, 4095) for index in self.active_servos}
        self.range_generation = 1
        self.last_command_s = 0.0
        self.last_sequence = 0
        self.rejected_targets = 0
        # STATE period; the board sends one every 10 ms, 0.1 s keeps tests fast.
        self.state_period_s = 0.1
        # Stored profile: the slot fields besides min/max (self.ranges), home
        # and operating current; staging and commit like main.c (the commit
        # applies at once and reports it in one DIAGNOSTICS frame).
        self.generation = 17
        self.homes: Dict[int, int] = {}
        self.slot_extra: Dict[int, Dict[str, Any]] = {}
        self._staged: Optional[Dict[int, Dict[str, Any]]] = None
        self.uploads = 0
        self.clock = clock
        self.session_id: Optional[int] = None
        self.transaction_id = 0
        self.state = HOMING_REQUIRED_STATE
        self.torque_state = headctl.TORQUE_NAMES.index("off_verified")
        self.token = 0
        self.limit_ma = 0
        # Each servo's profile operating_current_ma; like the firmware since
        # 2026-10-20, tuning caps a servo at min(limit, this).
        self.operating_ma: Dict[int, int] = {index: 600 for index in self.active_servos}
        self.goal = {index: 2000 + 37 * index for index in self.active_servos}
        self.neutral = dict(self.goal)
        self.inbox: List[tuple[int, bytes]] = []
        self._next_state_s = 0.0

    # headctl.Device surface used by TuningLink and headctl helpers.
    def hello(self) -> dict:
        self.session_id = self.SESSION_ID
        # What headctl.Device keeps from HELLO; upload_profile() moves it on.
        self.configuration_generation = self.generation
        return {"capabilities": (headctl.CAP_TRANSACTION_IDS | headctl.CAP_TUNING |
                                 headctl.CAP_PRESENT_PWM | headctl.CAP_ENABLE_AT_PRESENT_POSE |
                                 headctl.CAP_POSITION_RANGE),
                "session_id": self.SESSION_ID, "generation": self.generation}

    def send(self, kind: int, payload: bytes = b"") -> None:
        self._receive(kind, payload, 0)

    def send_confirmed(self, kind: int, payload: bytes = b"") -> int:
        self.transaction_id = (self.transaction_id + 1) & 0xFFFFFFFF or 1
        self._receive(kind, payload, self.transaction_id)
        return self.transaction_id

    def request(self, kind: int, payload: bytes = b"", timeout_s: float = 1.0):
        transaction_id = self.send_confirmed(kind, payload)
        for index, (response_kind, response) in enumerate(self.inbox):
            if response_kind in (headctl.ACK, headctl.NACK) and \
                    self.ack_matches(kind, response, transaction_id):
                del self.inbox[index]
                return response_kind == headctl.ACK and response[1] == 0, response
        return False, b""

    def get(self, request_kind: int, reply_kind: int, payload: bytes = b"",
            timeout_s: float = 1.0) -> bytes:
        if request_kind == headctl.GET_INFO:
            mask = sum(1 << index for index in self.active_servos)
            return headctl.encode_info({
                "version": 4, "expected_servo_count": len(self.active_servos),
                "allow_partial_inventory": 1, "thermal_start_c": 45,
                "thermal_full_c": 60, "active_servo_mask": mask})
        if request_kind == headctl.GET_SLOT:
            return headctl.encode_slot(self.slot(payload[0]))
        if request_kind == headctl.GET_POSITION_RANGE:
            valid = sum(1 << index for index in self.ranges) if self.state == ENABLED_STATE_SIM else 0
            ticks = []
            for index in range(headctl.HEAD_SERVO_COUNT):
                ticks += list(self.ranges[index]) if valid >> index & 1 else [0, 0]
            return struct.pack("<IB3xII40i", self.transaction_id, self.range_generation,
                               valid, 0, *ticks)
        raise RuntimeError("simulated controller: unsupported request")

    def recv(self, timeout_s: float = 1.0):
        now = self.clock()
        if self.state == ENABLED_STATE_SIM and now - self.last_command_s > 0.15:
            self.state = FAULT_STATE
            self.fault = headctl.FAULT_NAMES.index("WATCHDOG")
            self.torque_state = headctl.TORQUE_NAMES.index("off_verified")
        if now >= self._next_state_s:
            self._next_state_s = now + self.state_period_s
            mcu_time_ms = int(now * 1000) & 0xFFFFFFFF
            self.inbox.append((headctl.STATE, self._state_payload(mcu_time_ms)))
            self.inbox.append((headctl.PRESENT_PWM, self._present_pwm_payload(mcu_time_ms)))
        return self.inbox.pop(0) if self.inbox else None

    def close(self) -> None:
        pass

    SLOT_DEFAULTS = {"max_position_fraction_per_control_cycle": 0.005,
                     "max_acceleration_fraction_per_control_cycle": 0.001,
                     "homing_direction": 1.0, "homing_max_travel_ticks": 4096,
                     "homing_timeout_ms": 5000, "homing_current_ma": 120,
                     "homing_following_error_ticks": 200, "homing_persistence_ms": 50,
                     "homing_current_limit_ma": 200, "homing_speed_ticks_per_second": 200,
                     "homing_backoff_ticks": 40}

    def slot(self, index: int) -> Dict[str, Any]:
        low, high = self.ranges.get(index, (0, 4095))
        slot = dict(self.SLOT_DEFAULTS, **self.slot_extra.get(index, {}))
        slot.update({"index": index, "id": index, "branch": self.active_servos.get(index, 0),
                     "operating_current_ma": self.operating_ma.get(index, 0),
                     "min_tick": low, "max_tick": high,
                     "home_tick": self.homes.get(index, max(low, min(high, self.goal.get(index, low))))})
        return headctl.decode_slot(headctl.encode_slot(slot))   # the float32 round trip

    def _diagnostics_payload(self) -> bytes:
        raw = bytearray(headctl.DIAGNOSTICS_PAYLOAD_LENGTH)
        raw[0], raw[1] = self.state, self.fault
        raw[headctl.DIAGNOSTICS_STORAGE_STATE_OFFSET] = headctl.STORAGE_SUCCEEDED
        struct.pack_into("<I", raw, headctl.DIAGNOSTICS_GENERATION_OFFSET, self.generation)
        return bytes(raw)

    def ack_matches(self, kind: int, payload: bytes, transaction_id: int) -> bool:
        return (len(payload) >= headctl.ACK_PAYLOAD_LENGTH and payload[0] == kind and
                struct.unpack_from("<I", payload, 6)[0] == transaction_id)

    def trip(self, index: int, hardware_error: int) -> None:
        """A servo's own protection fires: the firmware latches FAULT SERVO."""
        self.hardware_errors[index] = hardware_error
        self.state = FAULT_STATE
        self.fault = headctl.FAULT_NAMES.index("SERVO")
        self.torque_state = headctl.TORQUE_NAMES.index("off_verified")

    # Simulated firmware.
    def cap_ma(self, index: int) -> float:
        """This servo's tuning cap (in ENABLED its operating current);
        infinite before tuning starts."""
        if self.state == ENABLED_STATE_SIM:
            return self.operating_ma.get(index, float("inf"))
        if not self.limit_ma:
            return float("inf")
        return min(self.limit_ma, self.operating_ma.get(index, self.limit_ma))

    def _tendon_solution(self) -> Dict[int, tuple[float, float]]:
        """servo -> (present tick, signed load mA) for the jaw and tendons.

        Each servo pulls LAG_MA_PER_TICK mA per tick of lag, up to the limit.
        The lines pull the jaw closed harder the further it is open, so its
        position (where its pull balances theirs) is found by bisection."""
        kp = self.LAG_MA_PER_TICK
        jaw = self.tendon_jaw

        def lines(jaw_present: float) -> tuple[Dict[int, tuple[float, float]], float]:
            opening = jaw_present - self.tendon_jaw_closed_tick if jaw is not None else 0.0
            solution: Dict[int, tuple[float, float]] = {}
            jaw_load = 0.0
            for index, (pull, taut, stiffness, coupling) in self.tendons.items():
                hidden = index in self.tendon_hidden
                felt = self._seen_opening.get(index, 0.0) if hidden else opening
                goal = pull * (self.goal[index] - taut)
                extra = coupling * felt
                for other, gain in self.tendon_links.get(index, ()):
                    o_pull, o_taut = self.tendons[other][0], self.tendons[other][1]
                    extra += gain * max(0.0, o_pull * (self.goal[other] - o_taut))
                slack_len = self.tendon_slack_len.get(index)
                if slack_len is not None and goal + extra <= -slack_len:
                    # Paid out past its slack: the line winds back onto the
                    # horn the other way and pulls again, turning the horn
                    # toward the pull direction (negative load in it).
                    g = goal + extra
                    present = (kp * g - stiffness * slack_len) / (kp + stiffness)
                    tension = stiffness * (-slack_len - present)
                    solution[index] = (taut + pull * (present - extra), -pull * tension)
                    continue
                if goal + extra <= 0:          # slack: the servo sits on its goal
                    stretch_coordinate, tension = float(goal), 0.0
                else:
                    stretch_coordinate = (kp * goal - stiffness * extra) / (kp + stiffness)
                    tension = stiffness * (stretch_coordinate + extra)
                    if tension > self.cap_ma(index):
                        tension = self.cap_ma(index)
                        stretch_coordinate = tension / stiffness - extra
                solution[index] = (taut + pull * stretch_coordinate, pull * tension)
                if hidden:
                    true_stretch = stretch_coordinate + coupling * opening
                    jaw_load += coupling * stiffness * max(0.0, true_stretch)
                else:
                    jaw_load += coupling * tension
            return solution, jaw_load

        if jaw is None:
            return lines(0.0)[0]
        # The jaw holds by lagging its goal: kp x (goal - present) = load,
        # with its pull capped at the limit.
        limit = self.cap_ma(jaw)
        high = float(self.goal[jaw])
        low = high - (limit if limit != float("inf") else 100000.0) / kp - 1.0
        for _ in range(60):
            middle = 0.5 * (low + high)
            # Below the balance the jaw out-pulls the lines; above it, not.
            if kp * (self.goal[jaw] - middle) >= min(lines(middle)[1], limit):
                low = middle
            else:
                high = middle
        solution = lines(high)[0]
        solution[jaw] = (high, kp * (self.goal[jaw] - high))
        return solution

    def present(self, index: int) -> int:
        if index in self.tendons or (index == self.tendon_jaw and self.tendons):
            return int(round(self._tendon_solution()[index][0]))
        goal, wall = self.goal[index], self.walls.get(index)
        if index in self.springs:
            rest, stiffness = self.springs[index]
            kp = self.LAG_MA_PER_TICK
            present = (kp * goal + stiffness * rest) / (kp + stiffness)
            if kp * abs(goal - present) > self.cap_ma(index):
                # Saturated: the servo pulls only its cap.
                present = rest + (self.cap_ma(index) / stiffness) * (1 if goal > rest else -1)
            return int(round(present))
        if wall is None:
            return goal
        start = self.neutral[index]
        if start <= wall:
            return min(goal, wall)
        return max(goal, wall)

    def load_ma(self, index: int) -> int:
        if index in self.tendons or (index == self.tendon_jaw and self.tendons):
            load = self._tendon_solution()[index][1]
            cap = self.cap_ma(index)
            if cap != float("inf"):
                load = max(-cap, min(cap, load))
            return int(round(load))
        load = abs(self.goal[index] - self.neutral[index]) * self.load_ma_per_tick
        lag = abs(self.goal[index] - self.present(index))
        load += lag * self.LAG_MA_PER_TICK
        return int(min(load, self.cap_ma(index)))

    def _ack(self, kind: int, result: int, transaction_id: int) -> None:
        self.inbox.append((headctl.ACK if result == 0 else headctl.NACK,
                           struct.pack("<BbIII", kind, result, self.token,
                                       transaction_id, self.SESSION_ID)))

    def _receive(self, kind: int, payload: bytes, transaction_id: int) -> None:
        token = struct.unpack_from("<I", payload, 0)[0] if len(payload) >= 4 else 0
        token_ok = self.token != 0 and token == self.token
        if kind == headctl.HELLO:
            return
        if kind == headctl.ACQUIRE:
            self.token = 0x1234ABCD
            self._ack(kind, 0, transaction_id)
        elif kind == headctl.RENEW_LEASE:
            self._ack(kind, 0 if token_ok else -1, transaction_id)
        elif kind == headctl.START_TUNING:
            limit = struct.unpack_from("<H", payload, 4)[0]
            if token_ok and self.state == HOMING_REQUIRED_STATE and \
                    0 < limit <= max(self.operating_ma.values(), default=0):
                self.limit_ma = limit
                self.state = TUNING_STATE
                self.torque_state = headctl.TORQUE_NAMES.index("on_verified")
                self.neutral = dict(self.goal)
                self._ack(kind, 0, transaction_id)
            else:
                self._ack(kind, -1, transaction_id)
        elif kind == headctl.TUNING_STEP:
            index, direction = struct.unpack_from("<Bb", payload, 4)
            self._ack(kind, self._step(token_ok, index, direction), transaction_id)
        elif kind == headctl.ENABLE_AT_PRESENT_POSE:
            inside = all(low <= self.present(index) <= high
                         for index, (low, high) in self.ranges.items())
            if not token_ok or self.state != HOMING_REQUIRED_STATE:
                self._ack(kind, -1, transaction_id)
            elif not inside:
                self._ack(kind, -34, transaction_id)
            elif getattr(self, "enable_bus_faults", 0) > 0:
                # Like the head on 2026-11-10: the preparation fails, the board
                # faults BUS with torque still verified off.
                self.enable_bus_faults -= 1
                self._ack(kind, 0, transaction_id)
                self.state = FAULT_STATE
                self.fault = headctl.FAULT_NAMES.index("BUS")
                self.torque_state = headctl.TORQUE_NAMES.index("off_verified")
            else:
                self.state = ENABLED_STATE_SIM
                self.torque_state = headctl.TORQUE_NAMES.index("on_verified")
                self.goal = {index: self.present(index) for index in self.active_servos}
                self.neutral = dict(self.goal)
                self.last_command_s = self.clock()
                self.last_sequence = 0
                self.range_generation += 1
                self._ack(kind, 0, transaction_id)
        elif kind == headctl.CLEAR_FAULT:
            if token_ok and self.state == FAULT_STATE and \
                    self.torque_state == headctl.TORQUE_NAMES.index("off_verified"):
                self.state, self.fault = HOMING_REQUIRED_STATE, 0
                self._ack(kind, 0, transaction_id)
            else:
                self._ack(kind, -1, transaction_id)
        elif kind == JOINT_TARGETS_SIM:
            self._targets(token_ok, payload)
        elif kind == headctl.RELEASE:
            self._ack(kind, 0 if token_ok else -1, transaction_id)
            if token_ok:
                self.token = 0
        elif kind in (headctl.STAGE_INFO, headctl.STAGE_SLOT, headctl.COMMIT):
            mutable = token_ok and self.state == HOMING_REQUIRED_STATE
            if kind == headctl.STAGE_INFO and mutable:
                self._staged = {}
            elif kind == headctl.STAGE_SLOT and mutable and self._staged is not None:
                slot = headctl.decode_slot(payload[4:4 + struct.calcsize(headctl.SLOT_FORMAT)])
                self._staged[slot["index"]] = slot
            elif kind == headctl.COMMIT and mutable and self._staged is not None and \
                    set(self._staged) == set(self.active_servos):
                for index, slot in self._staged.items():
                    self.ranges[index] = (slot["min_tick"], slot["max_tick"])
                    self.homes[index] = slot["home_tick"]
                    self.operating_ma[index] = slot["operating_current_ma"]
                    self.slot_extra[index] = {k: v for k, v in slot.items() if k in self.SLOT_DEFAULTS}
                self._staged = None
                self.generation += 1
                self.range_generation += 1
                self.uploads += 1
                self.inbox.append((headctl.DIAGNOSTICS, self._diagnostics_payload()))
            else:
                mutable = False
            self._ack(kind, 0 if mutable else -1, transaction_id)
        elif kind == headctl.DISABLE:
            if token_ok:
                self.state = HOMING_REQUIRED_STATE
                self.torque_state = headctl.TORQUE_NAMES.index("off_verified")
            self._ack(kind, 0 if token_ok else -1, transaction_id)
            self.token = 0

    def _targets(self, token_ok: bool, payload: bytes) -> None:
        """head_control_accept_command() and head_control_target_tick()
        (src/control.c), with the goal set at once instead of rate limited."""
        if len(payload) != 176:
            self.rejected_targets += 1; return
        _token, sequence, mask, mode = struct.unpack_from("<IIIB", payload, 0)
        q = struct.unpack_from("<20f", payload, 16)
        active_mask = sum(1 << index for index in self.active_servos)
        if (self.state != ENABLED_STATE_SIM or not token_ok or mask != active_mask or mode != 0
                or (self.last_sequence and (sequence - self.last_sequence) & 0xFFFFFFFF >= 0x80000000)
                or sequence == self.last_sequence
                or not all(0.0 <= q[index] <= 1.0 for index in self.active_servos)):
            self.rejected_targets += 1
            return
        self.last_sequence = sequence
        self.last_command_s = self.clock()
        for index in self.active_servos:
            low, high = self.ranges[index]
            self.goal[index] = max(low, min(high, low + int((high - low) * q[index])))

    def _step(self, token_ok: bool, index: int, direction: int) -> int:
        if not token_ok or self.state != TUNING_STATE:
            return -1
        if index not in self.active_servos or direction == 0 or \
                abs(direction) > headctl.TUNING_MAX_STEP_TICKS:
            return -22
        goal = self.goal[index] + direction
        travel = abs(goal - self.neutral[index])
        if travel > headctl_one_turn_ticks():
            return -34
        lag = abs(self.goal[index] - self.present(index))
        if travel > abs(self.goal[index] - self.neutral[index]) and \
                (abs(self.load_ma(index)) >= self.cap_ma(index) or
                 lag >= max(20, int(self.cap_ma(index)) // 3)):
            return -16
        if index in self.tendon_hidden and direction * self.tendons[index][0] > 0 and \
                self.tendon_jaw is not None:
            # Pulling the line in overcomes the friction: the horn now feels
            # what the jaw has done to it.
            self._seen_opening[index] = (self._tendon_solution()[self.tendon_jaw][0] -
                                         self.tendon_jaw_closed_tick)
        self.goal[index] = goal
        return 0

    def present_pwm_raw(self, index: int) -> int:
        """Any value that rises with load will do; this is not a motor model."""
        if index not in self.active_servos or self.state not in (TUNING_STATE, ENABLED_STATE_SIM):
            return 0
        return self.load_ma(index) // 2

    def _present_pwm_payload(self, mcu_time_ms: int) -> bytes:
        return struct.pack(f"<I{headctl.HEAD_SERVO_COUNT}h", mcu_time_ms,
                           *(self.present_pwm_raw(index)
                             for index in range(headctl.HEAD_SERVO_COUNT)))

    def _state_payload(self, mcu_time_ms: int) -> bytes:
        header = struct.pack("<BBBBHIIHBBBBIII", self.state, self.fault,
                             1 if self.state in (TUNING_STATE, ENABLED_STATE_SIM) else 0, 0, 0,
                             mcu_time_ms, 0, 500,
                             self.torque_state, 0, 0, self.range_generation & 0xFF,
                             0, 0, self.SESSION_ID)
        records = b""
        for index in range(headctl.HEAD_SERVO_COUNT):
            if index in self.active_servos:
                goal = self.goal[index]
                records += struct.pack("<iiihHBBBBI", goal, self.present(index), 0,
                                       self.load_ma(index), 11800,
                                       getattr(self, "temperatures", {}).get(index, 30), 0,
                                       self.hardware_errors.get(index, 0), 1, 4)
            else:
                records += struct.pack("<iiihHBBBBI", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFFFFFFFF)
        return header + records


def preview_controller(clock=time.monotonic) -> SimulatedController:
    """The --preview board: the jaw (7) and the right modiolus (8 down,
    11 diag, 14 up) on the tendon model, plus two free servos. The jaw opens
    toward higher ticks. 11 and 14 are hidden behind friction and hold the
    jaw closed until they pay out (`8+ 11- 14+`, release 11 14); 8 feels the
    jaw. It is for trying the panel, not a model of the head."""
    board = SimulatedController({7: 2, 8: 2, 10: 1, 11: 1, 12: 1, 14: 1}, clock=clock)
    board.tendon_jaw = 7
    board.tendon_jaw_closed_tick = board.goal[7]
    board.tendons = {8: (1, board.goal[8] + 30, 8.0, 0.3),
                     11: (-1, board.goal[11] - 40, 40.0, 0.5),
                     14: (1, board.goal[14] + 30, 40.0, 0.4)}
    board.tendon_hidden = {11, 14}
    # Like the head's profile from 2026-10-20: the jaw may take 650 mA, the rest 300.
    board.operating_ma = {index: 650 if index == 7 else 300 for index in board.active_servos}
    return board


# Sweep boxes that fit preview_controller(): right side only, jaw opening
# toward higher ticks there.
PREVIEW_SWEEP_BOXES = dict(SWEEP_BOX_DEFAULTS, pull="8+ 11- 14+", release="8 11 14",
                           defer_on_open="8")


def headctl_one_turn_ticks() -> int:
    """HEAD_TUNING_MAX_TRAVEL_TICKS: one revolution (include/head.h)."""
    return 4096
