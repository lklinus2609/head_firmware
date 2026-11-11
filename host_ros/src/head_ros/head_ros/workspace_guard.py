"""Keep every face command inside the measured safe workspace.

Everything the ML model (or the SDK) commands for the jaw and the two modioli
goes through WorkspaceGuard. Pure logic, no hardware: it turns actions into
tick targets, limits their speed, tells which lines must only follow, and
checks the live telemetry for the stop conditions found on the head.

Action, per side (right / left), a 2-D point (d, v):
    d in [0, 1]   diag activation
    v in [-1, 1]  v >= 0: up activation v and diag activation d drive, down
                          follows; v < 0: down activation -v drives, up and
                          diag follow (d is ignored).
So the down line never pulls against up and diag, nor they against it (rule
from the operator, 2026-10-20). A driving line goes from its session neutral
to neutral + activation x range in its pull direction. Every line that does
not drive follows: FollowerRegulator keeps it at the preload (it pays out
when loaded over the band and takes in slack when under).

Jaw action j in [0, 1]: closed tick (j = 0) to the open hold limit (j = 1).

Measured limits (tuning_runs/maps/20261020_workspace_map.json and
20261020T215830Z_coact.json): up 700 and diag 580 ticks, any mix of the two
to full range at jaw 4000 and 3880; down 300 ticks (line 9 lost grip at
308-355 ticks when pulled alone; 8 kept the same for symmetry; operator,
2026-10-20); jaw 4000 closed to 3880 open (at 3852 it needed 401 mA to hold).
Neutrals move about +-200 ticks between sessions, so they are found again
each session (pretension) and passed in. They also move with the jaw: from
jaw 4000 to 3880 in one session they moved by -50..+166 ticks (16 most;
20261020T215830Z_coact.json), about 500 mA at 3 mA/tick if a driver held its
target. So neutrals are found at the closed and the open jaw tick, and a
line's neutral at jaw action j is the straight line between the two. Two
points per line only: the straight line between them is an assumption.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# New head prototype (2026-11-09): left diag is servo 15 and left brow is 16
# (swapped from the old head, servo_notes.json). Pull directions unchanged per
# role (operator; head-direction run 150403: 15 -, 19 -, 11 +). Except the
# down lines 8 and 9: on this head they tighten toward MINUS (panel
# 2026-11-09_160439: 8 held -167 mA near 900 and 9 -159 mA near 2590 moving
# minus; knee run 155544: +850 ticks gave no load), most likely wound onto the
# horn the other way. Operator: "whatever works" (2026-11-09).
SIDES = {"right": {"down": 8, "diag": 11, "up": 14}, "left": {"down": 9, "diag": 15, "up": 19}}
PULL = {8: -1, 11: 1, 14: 1, 9: -1, 15: -1, 19: -1}
RANGES = {"up": 700, "diag": 580, "down": 300}
# Lower lip (3 right, 4 left; servo_notes.json): pull toward the operator's
# 2026-10-23 panel marks after the lip part was reprinted (3: neutral 3098 ->
# end 3833, 4: 1813 -> 980; "worked really well"). The 2026-10-12 marks (3:
# 1829 -> 424, 4: 2816 -> 4443) point the other way; they were used on
# 2026-10-22/23 and pulled the lips toward slack.
UPPER_SIDES = {"right": {"brow": 10, "oculi": 12, "lip": 3},
               "left": {"brow": 16, "oculi": 17, "lip": 4}}
# 12 toward lower ticks since the operator's 10-23 marks (1686 -> 695); the
# 10-12 mark (1738 -> 2801) pointed the other way.
UPPER_PULL = {10: -1, 12: -1, 16: 1, 17: -1, 3: 1, 4: -1}
# Brow: 440 from the right brow's 2026-10-12 end mark (443 ticks at -25 mA);
# raised to 480, then 520, on 2026-10-22 at the operator's request (at 440
# the brows read 55-65 mA, at 480 58-71 mA; the left brow's 10-12 end was
# 1042 ticks at 28 mA).
# Lip 735: the right lip's 10-23 end mark (the left's is 833); the operator
# kept it there instead of doubling to 800 (2026-10-23). Oculi 991: the right
# oculi's 10-23 mark span (1686 -> 695; the left's is 746); each oculi servo
# still stops at its own mark (face_stream.choose_upper_spans).
UPPER_RANGES = {"brow": 520, "oculi": 991, "lip": 735}
ALL_PULL = {**PULL, **UPPER_PULL}
# Upper lips (13 right, 18 left; servo_notes.json). They have no pull
# direction or range here: none was measured (their panel records hold no
# marks, 2026-11-06). configure_upper_lips() adds them to the tables above
# from the operator's tuning-panel marks (upper_lips_from_marks), so they are
# driven only after the operator marks a neutral and an end for both (test T1
# in docs/LIP_SYNC_TEST_PLAN.md), and never past those marks.
UPPER_LIP_SERVOS = {"right": 13, "left": 18}
UPPER_LIP_ROLE = "upper_lip"
JAW = 7
JAW_CLOSED_TICK, JAW_OPEN_TICK = 4000, 3880

# Live stop conditions, the same values the tuning tools use.
LINE_ABORT_MA = 280
DROP_FRACTION, DROP_MIN_PEAK_MA, DROP_SAMPLES = 0.5, 80, 3
# Follower: preload and band from the mapping (35 mA from below, band 8).
PRELOAD_MA, BAND_MA = 35, 8
LAG_MA_PER_TICK = 3
FOLLOW_GAIN, FOLLOW_MAX_STEP_TICKS = 0.5, 10


def clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


def upper_lips_from_marks(marks: Optional[Dict[int, Dict[str, dict]]]
                          ) -> Tuple[Dict[int, int], int, List[str]]:
    """(servo -> pull direction, role range in ticks, notes) for the upper
    lips from the tuning panel's marks (tuning_core.load_earlier_marks). A
    servo's direction is the sign of (end - neutral); both ends, when both are
    marked, must agree. The range is the larger travel (each servo still stops
    at its own end: face_stream.choose_upper_spans). Empty unless both upper
    lips have a usable neutral and end."""
    pulls: Dict[int, int] = {}
    travels: List[int] = []
    notes: List[str] = []
    for side, index in UPPER_LIP_SERVOS.items():
        servo_marks = (marks or {}).get(index) or {}
        neutral = (servo_marks.get("neutral") or {}).get("present_tick")
        ends = [(servo_marks.get(name) or {}).get("present_tick") for name in ("end_1", "end_2")]
        ends = [int(end) for end in ends if end is not None]
        if neutral is None or not ends:
            notes.append(f"upper lip {index} ({side}): no neutral and end mark in the panel records")
            continue
        signs = {1 if end > int(neutral) else -1 for end in ends if end != int(neutral)}
        if len(signs) != 1:
            notes.append(f"upper lip {index} ({side}): end marks {ends} do not give one pull "
                         f"direction from neutral {neutral}")
            continue
        pulls[index] = signs.pop()
        travels.append(max(abs(end - int(neutral)) for end in ends))
    if len(pulls) != len(UPPER_LIP_SERVOS):
        return {}, 0, notes
    return pulls, max(travels), notes


def configure_upper_lips(pulls: Dict[int, int], range_ticks: int) -> None:
    """Add the upper lips to UPPER_SIDES, UPPER_PULL, ALL_PULL and
    UPPER_RANGES (module tables, used by every session in this process)."""
    if set(pulls) != set(UPPER_LIP_SERVOS.values()) or any(p not in (1, -1) for p in pulls.values()):
        raise ValueError(f"upper lip pulls {pulls}: need +1 or -1 for servos "
                         f"{sorted(UPPER_LIP_SERVOS.values())}")
    if not 0 < range_ticks <= 4096:
        raise ValueError(f"upper lip range {range_ticks} must be 1-4096 ticks")
    for side, index in UPPER_LIP_SERVOS.items():
        UPPER_SIDES[side][UPPER_LIP_ROLE] = index
        UPPER_PULL[index] = pulls[index]
        ALL_PULL[index] = pulls[index]
    UPPER_RANGES[UPPER_LIP_ROLE] = int(range_ticks)


def clear_upper_lips() -> None:
    """Undo configure_upper_lips()."""
    for side, index in UPPER_LIP_SERVOS.items():
        UPPER_SIDES[side].pop(UPPER_LIP_ROLE, None)
        UPPER_PULL.pop(index, None)
        ALL_PULL.pop(index, None)
    UPPER_RANGES.pop(UPPER_LIP_ROLE, None)


def upper_lips_configured() -> bool:
    return UPPER_LIP_ROLE in UPPER_RANGES


@dataclass
class SideCommand:
    drivers: Dict[int, int]          # servo -> target tick
    followers: Tuple[int, ...]       # servos that only keep pretension


@dataclass
class WorkspaceGuard:
    neutral: Dict[int, int]                          # servo -> this session's neutral tick, jaw closed
    ranges: Dict[str, int] = field(default_factory=lambda: dict(RANGES))
    jaw_closed_tick: int = JAW_CLOSED_TICK
    jaw_open_tick: int = JAW_OPEN_TICK
    # Speed limits, ticks per second; None = no limit beyond the firmware's.
    line_speed_tps: Optional[float] = None
    jaw_speed_tps: Optional[float] = None
    # servo -> neutral tick with the jaw at jaw_open_tick; None = same as closed.
    neutral_open: Optional[Dict[int, int]] = None
    # Upper-face single-tendon lines (10 right brow, 12 right oculi, 16 left
    # brow, 17 left oculi): servo -> this session's neutral tick, and ranges.
    upper_neutral: Optional[Dict[int, int]] = None
    upper_ranges: Dict[str, int] = field(default_factory=lambda: dict(UPPER_RANGES))
    # Servo -> its own span (ticks), when shorter than its role's range: the
    # operator's panel end mark (2026-10-23: the T-zone skin is thick and the
    # brow magnets lose their attraction when pulled past it).
    upper_spans: Dict[int, int] = field(default_factory=dict)
    _last: Dict[int, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        missing = [index for side in SIDES.values() for index in side.values()
                   if index not in self.neutral]
        if missing:
            raise ValueError(f"no session neutral for servos {missing}")
        if self.neutral_open is None:
            self.neutral_open = dict(self.neutral)
        missing = [index for index in self.neutral if index not in self.neutral_open]
        if missing:
            raise ValueError(f"no open-jaw neutral for servos {missing}")
        if not self.jaw_open_tick < self.jaw_closed_tick:
            raise ValueError("the jaw opens toward lower ticks: open tick must be below closed")

    # Actions -> targets ------------------------------------------------------------
    def neutral_at(self, index: int, j: float = 0.0) -> int:
        """A line's neutral tick at jaw action j (0 closed, 1 open)."""
        j = clamp(j, 0.0, 1.0)
        return round(self.neutral[index] + j * (self.neutral_open[index] - self.neutral[index]))

    def side_command(self, side: str, d: float, v: float, j: float = 0.0) -> SideCommand:
        lines = SIDES[side]
        d, v = clamp(d, 0.0, 1.0), clamp(v, -1.0, 1.0)
        if v >= 0.0:
            activation = {"up": v, "diag": d}
        else:
            activation = {"down": -v}
        drivers = {}
        for role, value in activation.items():
            if value <= 0.0:
                continue
            index = lines[role]
            drivers[index] = self.neutral_at(index, j) + PULL[index] * round(value * self.ranges[role])
        followers = tuple(index for index in lines.values() if index not in drivers)
        return SideCommand(drivers, followers)

    def jaw_target(self, j: float) -> int:
        j = clamp(j, 0.0, 1.0)
        return round(self.jaw_closed_tick + j * (self.jaw_open_tick - self.jaw_closed_tick))

    def jaw_fraction(self, tick: float) -> float:
        """j for a jaw tick (clamped to 0..1)."""
        return clamp((tick - self.jaw_closed_tick) / (self.jaw_open_tick - self.jaw_closed_tick),
                     0.0, 1.0)

    def command(self, actions: Dict[str, Tuple[float, float]], j: float,
                jaw_tick: Optional[float] = None) -> Tuple[Dict[int, int], Tuple[int, ...]]:
        """(servo -> target tick for the jaw and every driving line, following
        lines) for one control update. actions: side -> (d, v). The lines'
        neutrals follow jaw_tick, where the jaw is now; without it, the jaw
        target."""
        targets = {JAW: self.jaw_target(j)}
        line_j = self.jaw_fraction(targets[JAW] if jaw_tick is None else jaw_tick)
        followers: Tuple[int, ...] = ()
        for side in SIDES:
            d, v = actions.get(side, (0.0, 0.0))
            cmd = self.side_command(side, d, v, line_j)
            targets.update(cmd.drivers)
            followers += cmd.followers
        return targets, followers

    def upper_command(self, actions: Dict[str, Tuple[float, float]]) -> Tuple[Dict[int, int], Tuple[int, ...]]:
        """(servo -> target tick for every configured upper-face line, driving
        upper-face servos). Single-tendon spring-return lines: 0 returns to
        upper_neutral, > 0 pulls by activation x upper_ranges[role]."""
        if not self.upper_neutral:
            return {}, ()
        targets: Dict[int, int] = {}
        drivers = []
        for role in UPPER_RANGES:          # brow, oculi, lip (+ upper_lip when configured)
            right_v, left_v = actions.get(role, (0.0, 0.0))
            for side, raw in (("right", right_v), ("left", left_v)):
                index = UPPER_SIDES[side][role]
                if index not in self.upper_neutral:
                    continue
                val = clamp(raw, 0.0, 1.0)
                span = round(val * min(self.upper_ranges[role],
                                       self.upper_spans.get(index, self.upper_ranges[role])))
                targets[index] = self.upper_neutral[index] + UPPER_PULL[index] * span
                if span > 0:
                    drivers.append(index)
        return targets, tuple(drivers)

    def limit_speed(self, targets: Dict[int, int], dt_s: float) -> Dict[int, int]:
        """Move each target at most speed x dt from the last one sent."""
        out = {}
        for index, target in targets.items():
            speed = self.jaw_speed_tps if index == JAW else self.line_speed_tps
            last = self._last.get(index)
            if speed is not None and last is not None:
                step = speed * dt_s
                target = round(clamp(target, last - step, last + step))
            self._last[index] = target
            out[index] = target
        return out

    def forget(self, index: int, tick: float) -> None:
        """Speed limiting continues from this tick (a follower's own move)."""
        self._last[index] = tick

    # Bounds, for checks -------------------------------------------------------------
    @staticmethod
    def role_of(index: int) -> str:
        return next(r for side in SIDES.values() for r, i in side.items() if i == index)

    @staticmethod
    def upper_role_of(index: int) -> str:
        return next(r for side in UPPER_SIDES.values() for r, i in side.items() if i == index)

    def line_bounds(self, index: int) -> Tuple[int, int]:
        """(lowest, highest) tick a line may ever be commanded to as a driver,
        at any jaw opening."""
        span = PULL[index] * self.ranges[self.role_of(index)]
        ticks = [n + k for n in (self.neutral[index], self.neutral_open[index]) for k in (0, span)]
        return min(ticks), max(ticks)

    def follower_bounds(self, index: int, j: float, max_pay_out: int) -> Tuple[int, int]:
        """(lowest, highest) tick a following line may go to at jaw action j:
        at most max_pay_out ticks out from its neutral, and never pulled in
        past its own driving range."""
        neutral = self.neutral_at(index, j)
        a = neutral - PULL[index] * max_pay_out
        b = neutral + PULL[index] * self.ranges[self.role_of(index)]
        return min(a, b), max(a, b)


def follow_step(current_ma: int, pull: int, preload_ma: int = PRELOAD_MA,
                band_ma: int = BAND_MA) -> int:
    """A following line's next goal step (ticks): out when loaded over the
    band, in when under it, nothing inside half the band."""
    tension = max(0, current_ma * pull)
    error = preload_ma - tension
    if abs(error) <= max(1, band_ma // 2):
        return 0
    size = max(1, min(FOLLOW_MAX_STEP_TICKS, round(FOLLOW_GAIN * abs(error) / LAG_MA_PER_TICK)))
    return size * (1 if error > 0 else -1) * pull


# A driver starts its peak again when its target goes back toward neutral by
# more than this (its load falls because it lets go), or when another driver
# on its side moves by more than this (the load moves between the lines: on
# 2026-10-20 line 19 held at full up fell from 117 to 52 mA while 16 pulled
# diag in, 20261020T235414Z_face.json). Only a driver whose side is otherwise
# still is checked for grip loss.
DROP_RELEASE_TICKS = 5
# The drop check uses a driver's load only while it has caught up with its
# goal (|goal - present| <= this). The servos are P-only (current ~ 3 mA per
# tick of position error), so a fast move reads its lag as load: on
# 2026-11-10 (run 200042_face_frames) line 9 read 85 mA while 32 ticks behind
# at 1000 ticks/s, then 23 mA once there, and the check stopped the run as a
# "magnet losing grip". Chosen: 8 ticks (~25 mA of lag).
DROP_SETTLED_LAG_TICKS = 8


class LiveChecks:
    """Stop conditions from the telemetry: a line over LINE_ABORT_MA, or a
    driving line whose load falls under DROP_FRACTION of its peak (the magnet
    losing grip, seen on 2026-10-20). Returns a reason, or None.

    pulled (servo -> ticks its target is pulled from neutral) lets a driver
    that is being let go start its peak again; without it, every fall counts."""

    def __init__(self) -> None:
        self._peak: Dict[int, int] = {}
        self._drops: Dict[int, int] = {}
        self._most_pulled: Dict[int, int] = {}
        self._others: Dict[int, Dict[int, int]] = {}

    def observe(self, current_ma: Dict[int, int], drivers: Tuple[int, ...],
                pulled: Optional[Dict[int, int]] = None,
                lag_ticks: Optional[Dict[int, int]] = None) -> Optional[str]:
        for index, current in current_ma.items():
            if index != JAW and abs(current) >= LINE_ABORT_MA:
                return f"servo {index} at {current} mA (line abort {LINE_ABORT_MA} mA)"
        for index in list(self._peak):
            if index not in drivers:                 # stopped driving: start over
                self._peak.pop(index); self._drops.pop(index, None)
                self._most_pulled.pop(index, None); self._others.pop(index, None)
        for index in drivers:
            if index not in ALL_PULL:                # the jaw: JawGuard watches it
                continue
            load = max(0, current_ma.get(index, 0) * ALL_PULL[index])
            if pulled is not None and index in pulled:
                most = self._most_pulled.get(index, pulled[index])
                if index in PULL:
                    side = next(lines for lines in SIDES.values() if index in lines.values())
                    others = {i: pulled[i] for i in side.values()
                              if i != index and i in drivers and i in pulled}
                else:
                    others = {}
                before = self._others.get(index, others)
                moved = (set(before) != set(others) or
                         any(abs(others[i] - before[i]) > DROP_RELEASE_TICKS for i in others))
                if pulled[index] < most - DROP_RELEASE_TICKS or moved:
                    self._peak[index] = load
                    self._drops[index] = 0
                    most = pulled[index]
                    before = others
                self._most_pulled[index] = max(most, pulled[index])
                self._others[index] = before
            if lag_ticks is not None and abs(lag_ticks.get(index, 0)) > DROP_SETTLED_LAG_TICKS:
                continue                     # still moving: its current is mostly lag
            self._peak[index] = max(self._peak.get(index, 0), load)
            dropped = self._peak[index] >= DROP_MIN_PEAK_MA and load < DROP_FRACTION * self._peak[index]
            self._drops[index] = self._drops.get(index, 0) + 1 if dropped else 0
            if self._drops[index] >= DROP_SAMPLES:
                return (f"servo {index}: load fell from {self._peak[index]} to {load} mA while "
                        "pulling (magnet losing grip?)")
        return None
