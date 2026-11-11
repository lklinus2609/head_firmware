"""ROS-independent logic for the manual jog panel.

The firmware accepts only normalized actuator positions (`q`, 0.0 to 1.0 over
each servo's calibrated workspace). The bridge derives the present `q` from
raw ticks and the firmware's POSITION_RANGE reply, and publishes NaN while it
has no current range (older firmware, a range not read yet, or one made stale
by homing or a profile change). Every function here that needs a present
position takes it from `ServoState.present_normalized` and fails closed when
that value is not a finite number in 0..1.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import IntEnum
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Dict, List, Tuple


class HeadLifecycle(IntEnum):
    """Mirror of the HeadState lifecycle constants, usable without ROS."""

    BOOT = 0
    HOMING_REQUIRED = 2
    HOMING = 3
    MAINTENANCE_CALIBRATION = 4
    READY = 5
    ENABLED = 6
    FAULT = 7
    PROPRIOCEPTION_SETTLING = 8
    PROPRIOCEPTION_HOLD = 9
    ROUTING = 10
    TUNING = 11


def lifecycle_name(state: int) -> str:
    try:
        return HeadLifecycle(state).name
    except ValueError:
        return f"UNKNOWN({state})"


# In ENABLED the firmware latches a watchdog fault when no target has been
# accepted for HEAD_COMMAND_TIMEOUT_MS (100 ms) plus HEAD_TIMEOUT_HOLD_MS
# (50 ms), both in include/head.h. Publishing every 20 ms tolerates several
# lost best-effort samples inside that window.
TARGET_PUBLISH_PERIOD_S = 0.02
# Same renewal period as the routing controls in collector_gui.py; the
# firmware lease lasts 1,000 ms.
LEASE_RENEW_PERIOD_S = 0.4
# The bridge treats READY as current only when the last state arrived within
# 0.25 s (bridge.py _wait_for_ready). Motion and renewal use the same bound.
STATE_FRESH_S = 0.25

# Jog steps in normalized units. 0.05 is deliberately the largest: anything
# coarser is typed into the target box or dragged on the slider, and the
# firmware rate limiter paces the move either way.
JOG_STEPS = (-0.05, -0.01, -0.001, 0.001, 0.01, 0.05)


class NormalizedPositionUnavailable(RuntimeError):
    """Raised when a present `q` needed for safe seeding is missing."""


def active_servo_indexes(active_servo_mask: int, servo_count: int) -> List[int]:
    return [index for index in range(servo_count) if active_servo_mask & (1 << index)]


def active_servos(head: Any) -> List[Any]:
    return [head.servos[index]
            for index in active_servo_indexes(int(head.active_servo_mask), len(head.servos))]


def is_normalized(value: float) -> bool:
    return math.isfinite(value) and 0.0 <= value <= 1.0


def clamp_normalized(value: float) -> float:
    if not math.isfinite(value):
        raise ValueError(f"non-finite normalized position {value}")
    return min(1.0, max(0.0, float(value)))


def present_positions_normalized(head: Any) -> Dict[str, float]:
    """Present `q` for every active servo, or raise naming each one missing.

    An offline servo, a NaN placeholder or a value outside 0..1 all count as
    missing: seeding a target from any of them could command a jump.
    """
    positions: Dict[str, float] = {}
    missing: List[str] = []
    outside: List[str] = []
    for servo in active_servos(head):
        value = float(servo.present_normalized)
        if not servo.online or not math.isfinite(value):
            missing.append(servo.name)
        elif not is_normalized(value):
            outside.append(f"{servo.name} (q {value:.3f})")
        else:
            positions[servo.name] = value
    if not positions and not missing and not outside:
        raise NormalizedPositionUnavailable("the controller reports no active servos")
    reasons: List[str] = []
    if missing:
        reasons.append("present_normalized is unavailable for " + ", ".join(missing))
    if outside:
        reasons.append("outside the profile range: " + ", ".join(outside))
    if reasons:
        raise NormalizedPositionUnavailable("; ".join(reasons))
    return positions


class TargetVector:
    """The complete target the panel streams: every active joint, every time.

    The bridge rejects partial, duplicated or unknown joint lists, so the
    panel never publishes a single joint on its own.
    """

    def __init__(self, positions: Dict[str, float]) -> None:
        if not positions:
            raise ValueError("a target needs at least one joint")
        self._positions = {name: clamp_normalized(value) for name, value in positions.items()}

    @property
    def names(self) -> List[str]:
        return list(self._positions)

    def get(self, name: str) -> float:
        return self._positions[name]

    def set(self, name: str, value: float) -> float:
        if name not in self._positions:
            raise KeyError(name)
        self._positions[name] = clamp_normalized(value)
        return self._positions[name]

    def step(self, name: str, delta: float) -> float:
        return self.set(name, self.get(name) + delta)

    def as_lists(self) -> Tuple[List[str], List[float]]:
        names = self.names
        return names, [self._positions[name] for name in names]


@dataclass
class JogPose:
    name: str
    positions_normalized: Dict[str, float] = field(default_factory=dict)
    created: str = ""
    note: str = ""

    def to_json(self) -> dict:
        return {
            "name": self.name,
            "created": self.created,
            "note": self.note,
            "positions_normalized": dict(sorted(self.positions_normalized.items())),
        }

    @classmethod
    def from_json(cls, data: dict) -> "JogPose":
        positions = {str(name): float(value)
                     for name, value in dict(data.get("positions_normalized", {})).items()}
        invalid = [name for name, value in positions.items() if not is_normalized(value)]
        if invalid:
            raise ValueError(f"pose {data.get('name')!r} has invalid positions for "
                             + ", ".join(invalid))
        return cls(name=str(data["name"]), positions_normalized=positions,
                   created=str(data.get("created", "")), note=str(data.get("note", "")))


def recall_pose(pose: JogPose, targets: TargetVector) -> Tuple[Dict[str, float], List[str]]:
    """Apply a pose to the joints it shares with the live target.

    Returns the applied positions and the pose joints that are not active.
    Active joints the pose does not mention keep their current target.
    """
    applied: Dict[str, float] = {}
    skipped: List[str] = []
    active = set(targets.names)
    for name, value in pose.positions_normalized.items():
        if name in active:
            applied[name] = targets.set(name, value)
        else:
            skipped.append(name)
    return applied, sorted(skipped)


POSE_FILE_VERSION = 1


class JogPoseLibrary:
    """Named `q` poses in a JSON file. Saves are atomic.

    A pose stores joint names and normalized positions only. It cannot carry
    ticks, currents or limits, so recalling one can only ask joints to move
    inside the calibrated workspace the firmware already enforces.
    """

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.poses: List[JogPose] = []

    def load(self) -> None:
        self.poses = []
        if not self.path.exists():
            return
        with self.path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
        version = int(data.get("version", 0))
        if version != POSE_FILE_VERSION:
            raise ValueError(f"{self.path} is jog pose file version {version}, "
                             f"this tool writes version {POSE_FILE_VERSION}")
        self.poses = [JogPose.from_json(entry) for entry in data.get("poses", [])]

    def save(self) -> None:
        payload = {"version": POSE_FILE_VERSION,
                   "poses": [pose.to_json() for pose in self.poses]}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        handle = tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", dir=str(self.path.parent),
            prefix=self.path.name + ".", suffix=".tmp", delete=False)
        try:
            with handle:
                json.dump(payload, handle, indent=2)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(handle.name, self.path)
        except BaseException:
            try:
                os.unlink(handle.name)
            except OSError:
                pass
            raise

    def names(self) -> List[str]:
        return [pose.name for pose in self.poses]

    def get(self, name: str) -> JogPose:
        for pose in self.poses:
            if pose.name == name:
                return pose
        raise KeyError(name)

    def store(self, name: str, positions_normalized: Dict[str, float], note: str = "") -> JogPose:
        pose = JogPose.from_json({"name": name, "note": note,
                                  "positions_normalized": positions_normalized})
        pose.created = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%SZ")
        for index, existing in enumerate(self.poses):
            if existing.name == name:
                self.poses[index] = pose
                break
        else:
            self.poses.append(pose)
        self.save()
        return pose

    def delete(self, name: str) -> None:
        self.poses = [pose for pose in self.poses if pose.name != name]
        self.save()


def enable_blockers(head: Any, has_lease: bool, is_state_fresh: bool,
                    required_state: HeadLifecycle = HeadLifecycle.READY) -> List[str]:
    """Reasons Enable must not be requested now; empty when it may be.

    READY is the homed Enable. HOMING_REQUIRED is Enable at present pose,
    which the firmware refuses unless every servo is inside its profile range;
    a present `q` outside 0..1 is reported here first.
    """
    blockers: List[str] = []
    if head is None:
        return ["no /head/state received"]
    if not is_state_fresh:
        blockers.append("/head/state is stale")
    if int(head.state) != required_state:
        blockers.append(f"firmware is {lifecycle_name(int(head.state))}, "
                        f"not {required_state.name}")
    if head.torque_enabled:
        blockers.append("torque may still be on")
    if int(head.fault) != 0:
        blockers.append(f"fault {int(head.fault)} is latched")
    if not has_lease:
        blockers.append("this panel holds no control lease")
    try:
        present_positions_normalized(head)
    except NormalizedPositionUnavailable as error:
        blockers.append(str(error))
    return blockers


@dataclass
class HeadObservation:
    """What the caller must do after JogSession.observe_head()."""

    notices: List[str] = field(default_factory=list)
    publish_now: bool = False
    request_disable: bool = False


class JogSession:
    """Host-side jog state shared by the ROS node and the preview backend.

    Invariants:
    - a target exists only while the firmware is ENABLED under this panel's
      lease, and it is seeded from measured `present_normalized` values;
    - if ENABLED is observed without usable measured positions, the session
      never streams and asks the caller to request Disable;
    - streaming stops as soon as /head/state goes stale, leaving the firmware
      command watchdog to remove torque rather than commanding blind.
    """

    def __init__(self) -> None:
        self.head: Any = None
        self.last_head_monotonic_s = 0.0
        self.lease_token = 0
        self.lease_boot_session_id = 0
        self.targets: TargetVector | None = None
        self.is_motion_frozen = False

    # --- lease -------------------------------------------------------------
    def grant_lease(self, lease_token: int) -> None:
        if self.head is None:
            raise RuntimeError("a lease is only recorded against an observed controller boot")
        self.lease_token = int(lease_token)
        self.lease_boot_session_id = int(self.head.boot_session_id)

    def drop_lease(self, reason: str) -> str:
        self.lease_token = 0
        self.lease_boot_session_id = 0
        self.targets = None
        return f"control lease dropped: {reason}"

    @property
    def has_lease(self) -> bool:
        return self.lease_token != 0

    # --- state feed --------------------------------------------------------
    def is_state_fresh(self, now_monotonic_s: float) -> bool:
        return (self.head is not None and
                now_monotonic_s - self.last_head_monotonic_s < STATE_FRESH_S)

    def can_renew(self, now_monotonic_s: float) -> bool:
        return (self.has_lease and self.is_state_fresh(now_monotonic_s) and
                int(self.head.boot_session_id) == self.lease_boot_session_id)

    def observe_head(self, head: Any, now_monotonic_s: float) -> HeadObservation:
        result = HeadObservation()
        self.head = head
        self.last_head_monotonic_s = now_monotonic_s
        if self.has_lease and int(head.boot_session_id) != self.lease_boot_session_id:
            result.notices.append(self.drop_lease("controller boot session changed"))
        if int(head.state) != HeadLifecycle.ENABLED:
            if self.targets is not None:
                result.notices.append(
                    f"stopped streaming: firmware is {lifecycle_name(int(head.state))}")
            self.targets = None
            self.is_motion_frozen = False
            return result
        if self.targets is not None or not self.has_lease or self.is_motion_frozen:
            return result
        try:
            self.targets = TargetVector(present_positions_normalized(head))
        except NormalizedPositionUnavailable as error:
            self.is_motion_frozen = True
            result.request_disable = True
            result.notices.append(
                f"ENABLED without usable measured positions ({error}); requesting Disable")
            return result
        result.publish_now = True
        result.notices.append("ENABLED: streaming targets seeded from measured positions")
        return result

    def check_stale(self, now_monotonic_s: float) -> str | None:
        if self.targets is None or self.is_state_fresh(now_monotonic_s):
            return None
        self.targets = None
        return ("/head/state went stale; stopped streaming. The firmware command "
                "watchdog will fault and remove torque.")

    # --- motion ------------------------------------------------------------
    def freeze_motion(self) -> None:
        """Hold the current target (still streamed) and ignore further jogs."""
        self.is_motion_frozen = True

    def can_jog(self, now_monotonic_s: float) -> bool:
        return (self.targets is not None and not self.is_motion_frozen and
                self.is_state_fresh(now_monotonic_s) and
                int(self.head.state) == HeadLifecycle.ENABLED)

    def set_target(self, name: str, value: float, now_monotonic_s: float) -> float | None:
        if not self.can_jog(now_monotonic_s):
            return None
        return self.targets.set(name, value)

    def step_target(self, name: str, delta: float, now_monotonic_s: float) -> float | None:
        if not self.can_jog(now_monotonic_s):
            return None
        return self.targets.step(name, delta)

    def recall(self, pose: JogPose,
               now_monotonic_s: float) -> Tuple[Dict[str, float], List[str]] | None:
        if not self.can_jog(now_monotonic_s):
            return None
        return recall_pose(pose, self.targets)

    def enable_blockers(self, now_monotonic_s: float) -> List[str]:
        return enable_blockers(self.head, self.has_lease, self.is_state_fresh(now_monotonic_s))

    def enable_at_present_pose_blockers(self, now_monotonic_s: float) -> List[str]:
        return enable_blockers(self.head, self.has_lease, self.is_state_fresh(now_monotonic_s),
                               HeadLifecycle.HOMING_REQUIRED)
