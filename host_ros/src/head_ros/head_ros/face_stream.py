"""Drive the face from workspace actions in ENABLED mode, without ROS.

The SDK / ML path. A session:

1. loads this boot's session neutrals (head-ranges --neutral-only: every
   line's neutral goal tick with the jaw closed and open);
2. makes the session envelope (session_envelope()): the profile's min..max
   for every face line from these neutrals and ranges, and uploads it when it
   differs from the board's (decided with the operator, 2026-10-20: neutrals
   move between sessions, so fixed ticks do not work as the firmware's limit);
3. enables at the present pose (no homing; the head holds where it is);
3. every 1/STREAM_HZ s turns the actions (per side (d, v), jaw j) into tick
   targets with WorkspaceGuard, moves the following lines by their load
   (pretension only, never against the drivers), limits the speed, and
   sends one target frame for every active servo. Servos that are not on
   the face hold the pose they had at Enable.

The firmware faults (WATCHDOG, torque off) when no target comes for 100 ms
plus a 50 ms hold (HEAD_COMMAND_TIMEOUT_MS, HEAD_TIMEOUT_HOLD_MS in
include/head.h), so a stalled host relaxes the face rather than holding it.

Every stop turns torque off and the face relaxes: a line over the abort
load, a driving line losing its load (magnet), a board fault, stale state,
Ctrl-C/SIGTERM or the stop file. The jaw held over 400 mA for 2 s first goes
back to closed with every line following, then torque goes off; over 600 mA
for 0.5 s turns torque off at once (tuning_core.JawGuard).

    scripts/head-face --preview
    scripts/head-face --neutrals tuning_runs/runs/<date>/<time>_knee/neutrals.json --loops 2
    scripts/head-face --trajectory speech.csv --audio speech.wav    (mouth_actions.py)

Every run writes commands.csv (CommandLog): the mouth actions and targets of
each update with their host UTC, to pair with the camera's video.frames.csv.
The upper lips (13, 18) are driven only when the tuning panel marks give both
a neutral and an end (configure_upper_lips_for_session).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import struct
import sys
import time
from typing import Callable, Dict, List, Optional, Sequence, Tuple

try:
    from . import headctl, mouth_actions, records_layout, tuning_core, workspace_guard as wg
    from .range_finder import SESSION_NEUTRAL_FORMAT, SESSION_NEUTRAL_SUFFIX
    from .sweep_runner import DEFAULT_RECORDS
except ImportError:  # run as a plain script from this directory
    import headctl  # type: ignore[no-redef]
    import mouth_actions  # type: ignore[no-redef]
    import records_layout  # type: ignore[no-redef]
    import tuning_core  # type: ignore[no-redef]
    import workspace_guard as wg  # type: ignore[no-redef]
    from range_finder import SESSION_NEUTRAL_FORMAT, SESSION_NEUTRAL_SUFFIX  # type: ignore[no-redef]
    from sweep_runner import DEFAULT_RECORDS  # type: ignore[no-redef]

JOINT_TARGETS = 5                         # HEAD_MSG_JOINT_TARGETS (include/protocol.h)
COMMAND_MODE_POSITION = 0                 # HEAD_COMMAND_POSITION (include/head.h)
ENABLED_STATE = headctl.STATE_NAMES.index("ENABLED")
TORQUE_ON = headctl.TORQUE_NAMES.index("on_verified")
STREAM_HZ = 50
ENABLE_TIMEOUT_S = 2.0
# Speeds chosen for the first runs, not measured limits: lines 300 ticks/s
# (the 700-tick up range in about 2.3 s), jaw 60 ticks/s (closed to open in
# 2 s; tuning moved it at about 75 ticks/s).
LINE_SPEED_TPS = 300.0
JAW_SPEED_TPS = 60.0
# A following line pays out at most this far from its neutral (as in
# coact_test.py, FOLLOW_MAX_TICKS).
FOLLOW_MAX_PAY_OUT_TICKS = 600
# Jaw back-off: closed within this many ticks, or this long, then torque off.
BACK_OFF_ARRIVE_TICKS = 5
BACK_OFF_TIMEOUT_S = 3.0
SESSION_FORMAT = "head-face-session-1"
# Operator, 2026-11-10: stop the exploration when any servo goes beyond 52 C.
# Every session stops (torque off) when a servo reads this or more.
TEMP_STOP_C = 52
# --jaw-release ROLE=TICKS: a role's lines' neutral moves this far toward
# slack at the fully open jaw (j = 1), in proportion to the jaw opening
# (operator, 2026-11-10: the corner down line must release as the jaw opens).
# The most allowed (chosen, not measured).
JAW_RELEASE_MAX_TICKS = 400
# Servos driven through FaceSession.extra (eyes, lids): their envelope is
# their limits plus this margin (as eye_tour.ENVELOPE_MARGIN_TICKS).
EXTRA_ENVELOPE_MARGIN_TICKS = 20
# The most open jaw GOAL --jaw-open may ask for (operator, 2026-10-23: "try
# with the jaw's limit as 650"). The jaw has no integral term, so it settles
# load / ~3.5 mA per tick short of its goal (3943 at 218 mA for goal 3880);
# a goal past the mapped 3880 lets it push harder. Its guards still apply:
# over 400 mA for 2 s back to closed, over 600 mA for 0.5 s torque off.
# 3650 since the operator allowed the jaw up to 650 mA for 1 s (same day).
JAW_GOAL_MIN_TICK = 3600         # 3600 with the 750 mA jaw allowance (same day)
# The jaw's current for a session (--jaw-current): the operator allowed up to
# 750 mA, over 650 mA for at most 0.5 s (2026-10-23; stall is 0.80 A at
# 11.1 V and Current Limit(38) goes to 910 mA, xc330-t181.pdf).
JAW_CURRENT_MAX_MA = 750
# The fastest a face servo may be asked to move: about 70 % of the XC330's
# no-load speed (104 rev/min x 4096 / 60 = 7100 ticks/s at 11.1 V,
# xc330-t181.pdf). The operator asked for 70 % on 2026-10-23, reached in steps.
SPEED_MAX_TPS = 5000
# After the script ends: hold its last pose until the targets have not
# changed for SETTLE_UPDATES updates (0.2 s), at most SETTLE_MAX_S.
SETTLE_UPDATES = 10
SETTLE_MAX_S = 5.0
# Session envelope (chosen 2026-10-20, not measured). A face line's pull side
# ends PULL_SIDE_MARGIN_TICKS past the furthest a driver may go (the guard
# stops there first); its slack side SLACK_SIDE_TICKS out from neutral (the
# follower's 600-tick pay-out cap plus room for the face relaxing while torque
# is off), and always PRESENT_MARGIN_TICKS past where it sits now.
PULL_SIDE_MARGIN_TICKS = 10
SLACK_SIDE_TICKS = 1000
PRESENT_MARGIN_TICKS = 100
# A face line this far on the slack side of its neutral is not slack but
# shifted: a turn-count jump after a 12 V cycle (2026-11-10: line 9 sat 3936
# "slack", really 160 tight, and the follower wound it to the 280 mA abort)
# or stale neutrals. Followers pay out at most FOLLOW_MAX_PAY_OUT_TICKS (600);
# 2026-11-10 talking left line 8 284 ticks slack. Chosen: 1500.
LINE_SLACK_MAX_TICKS = 1500
# The jaw's envelope: from 10 ticks past this session's open limit to 4050.
JAW_ENVELOPE_OPEN_MARGIN_TICKS, JAW_ENVELOPE_CLOSED_TICK = 10, 4050
JAW_PRESENT_MARGIN_TICKS = 20
# A servo off the face keeps its range when it is inside; otherwise it gets
# this window around where it is (it only holds its pose here).
OTHER_WINDOW_TICKS = 2048
BOARD_READY_TIMEOUT_S = 10.0
# HEAD_DXL_POSITION_MIN_TICK / _MAX_TICK (include/head.h).
ENCODER_MIN_TICK, ENCODER_MAX_TICK = -1048575, 1048575


# Session neutrals ------------------------------------------------------------------
@dataclass
class SessionNeutrals:
    closed: Dict[int, int]
    open: Dict[int, int]
    jaw_closed_tick: int
    jaw_open_tick: int
    preload_ma: int
    controller_session_id: Optional[int]
    path: Optional[Path] = None
    # The jaw tick the open slice was asked for (head-ranges --jaw-tick); the
    # session's open limit (j = 1). Files from before 2026-10-21 have none:
    # their limit is the mapped wg.JAW_OPEN_TICK.
    jaw_open_target: Optional[int] = None
    # "camera_knee" for head-knee's camera neutrals (already emotionless: no
    # slack offset); None for head-ranges' 35 mA neutrals.
    kind: Optional[str] = None

    @property
    def jaw_open_limit(self) -> int:
        return wg.JAW_OPEN_TICK if self.jaw_open_target is None else self.jaw_open_target


def load_session_neutrals(path: Path) -> SessionNeutrals:
    data = json.loads(path.read_text())
    if data.get("format") != SESSION_NEUTRAL_FORMAT:
        raise ValueError(f"{path} is not a {SESSION_NEUTRAL_FORMAT} file")
    return SessionNeutrals({int(k): int(v) for k, v in data["closed"].items()},
                           {int(k): int(v) for k, v in data["open"].items()},
                           int(data["jaw_closed_tick"]), int(data["jaw_open_tick"]),
                           int(data["preload_ma"]), data.get("controller_session_id"), path,
                           data.get("jaw_open_target"), data.get("neutral_kind"))


# The finder's 35 mA neutral pulls the corners up noticeably; the skin's own
# rest is about this many ticks slacker for up and diag (operator, 2026-10-21:
# left up and diag reeled out 200-300 ticks by hand looked truly neutral; at
# 200 ticks 14, 16, 19 still held 17-22 mA upward tension). Down is not changed.
# The ranges count from the shifted neutral, so the most-pulled end is this much
# less pulled.
NEUTRAL_SLACK_OFFSET_TICKS = {"up": 300, "diag": 300, "down": 0}
# Ranges head-face drives, from the shifted neutral: conservative for now
# (operator, 2026-10-21). Against the 35 mA neutral the most-pulled ends are
# up +200, diag +100, down +300 ticks; the right magnet came off near diag
# +390 that day and line 9 lost grip at 308-355 ticks (2026-10-20). The mapped
# limits are wg.RANGES (up 700, diag 580, down 300).
SESSION_RANGES = {"up": 500, "diag": 400, "down": 300}
# Upper-face ranges driven from each servo's neutral pose at Enable, inside
# wg.UPPER_RANGES (brow 520, oculi 740; see workspace_guard.py).
# Brow 520 = the mapped maximum, chosen by the operator on 2026-10-22 after
# brow-only takes at 350/440/480/520 (loads 55-71 mA; the camera saw no more
# brow motion past about 350 ticks).
# Lower lip 400: "a little bit" (operator, 2026-10-22) of the 735-tick marks.
# Oculi 991: each oculi servo then pulls to its own 10-23 panel end mark.
UPPER_SESSION_RANGES = {"brow": 520, "oculi": 991, "lip": 400}


def complete_upper_ranges(upper_ranges: Optional[Dict[str, int]]) -> Dict[str, int]:
    """The given upper-face ranges plus the mapped range of every role they
    leave out (the upper lips, once configured from their panel marks, use
    their full marked span unless --upper-range narrows it)."""
    out = dict(wg.UPPER_RANGES if upper_ranges is None else upper_ranges)
    for role, ticks in wg.UPPER_RANGES.items():
        out.setdefault(role, ticks)
    return out


def shifted_neutrals(neutrals: SessionNeutrals,
                     offsets: Optional[Dict[str, int]] = None) -> SessionNeutrals:
    """The neutrals moved toward slack by each role's offset (ticks)."""
    offsets = NEUTRAL_SLACK_OFFSET_TICKS if offsets is None else offsets

    def shift(table: Dict[int, int]) -> Dict[int, int]:
        return {i: tick - wg.PULL[i] * offsets.get(wg.WorkspaceGuard.role_of(i), 0)
                for i, tick in table.items()}
    return SessionNeutrals(shift(neutrals.closed), shift(neutrals.open), neutrals.jaw_closed_tick,
                           neutrals.jaw_open_tick, neutrals.preload_ma,
                           neutrals.controller_session_id, neutrals.path, neutrals.jaw_open_target,
                           neutrals.kind)


def newest_session_neutrals(records: Path) -> Optional[Path]:
    """The newest runs/*/*/neutrals.json (head-knee, head-ranges
    --neutral-only); files from before the 2026-10-23 layout as a fallback."""
    newest = records_layout.newest(records, records_layout.NEUTRALS)
    if newest is not None:
        return newest
    found = sorted(Path(records).glob("*" + SESSION_NEUTRAL_SUFFIX))
    return found[-1] if found else None


def guard_from_neutrals(neutrals: SessionNeutrals, ranges: Optional[Dict[str, int]] = None,
                        upper_ranges: Optional[Dict[str, int]] = None,
                        jaw_open: Optional[int] = None,
                        **speeds) -> wg.WorkspaceGuard:
    """The guard for this session. The jaw goes from the mapped closed tick
    (wg.JAW_CLOSED_TICK) to the session's open limit, which may be less open
    than the mapped wg.JAW_OPEN_TICK (on 2026-10-21 the jaw needed over
    400 mA at 3880, so 3900 was used) but never more. The slices only have
    to be near those ends, since the line neutrals are interpolated between
    the slices."""
    limit = neutrals.jaw_open_limit if jaw_open is None else jaw_open
    lowest = JAW_GOAL_MIN_TICK if jaw_open is not None else wg.JAW_OPEN_TICK
    if not lowest <= limit <= wg.JAW_CLOSED_TICK - 20:
        raise ValueError(f"the jaw open limit {limit} must be {lowest} to "
                         f"{wg.JAW_CLOSED_TICK - 20}")
    for name, tick, end in (("closed", neutrals.jaw_closed_tick, wg.JAW_CLOSED_TICK),
                            ("open", neutrals.jaw_open_tick, limit)):
        # Camera neutrals are found with the jaw where it rests (3977 on
        # 2026-10-22), not driven to 4000. A more open jaw leaves every line's
        # neutral a little slacker (8 ~48, 14 ~39, 16/19 ~18, 9 ~13, 11 ~0
        # ticks per 23 jaw ticks, from the 2026-10-21 finder slices), so used
        # at 4000 the lines are slightly slack, never tighter: 30 ticks there.
        tolerance = 30 if name == "closed" and neutrals.kind == "camera_knee" else 15
        if name == "open" and neutrals.kind == "camera_knee":
            continue        # camera neutrals: the open-jaw neutrals are a copy, no slice
        if abs(tick - end) > tolerance:
            raise ValueError(f"the {name}-jaw slice was at tick {tick}, not near {end}: "
                             f"find the neutrals again at --jaw-tick 4000 --jaw-tick {limit}")
    ranges = dict(wg.RANGES if ranges is None else ranges)
    if any(not 0 < ranges[role] <= wg.RANGES[role] for role in wg.RANGES):
        raise ValueError(f"ranges {ranges} must be within the mapped {wg.RANGES}")
    upper_ranges = complete_upper_ranges(upper_ranges)
    if any(not 0 < upper_ranges[role] <= wg.UPPER_RANGES[role] for role in wg.UPPER_RANGES):
        raise ValueError(f"upper ranges {upper_ranges} must be within the mapped {wg.UPPER_RANGES}")
    return wg.WorkspaceGuard(dict(neutrals.closed), neutral_open=dict(neutrals.open), ranges=ranges,
                             upper_ranges=upper_ranges, jaw_open_tick=limit, **speeds)


# Session envelope -----------------------------------------------------------------------
def session_envelope(profile: dict, neutrals: SessionNeutrals, present: Dict[int, int],
                     ranges: Optional[Dict[str, int]] = None,
                     upper_ranges: Optional[Dict[str, int]] = None,
                     jaw_open: Optional[int] = None,
                     upper_neutral: Optional[Dict[int, int]] = None,
                     upper_spans: Optional[Dict[int, int]] = None,
                     jaw_current_ma: Optional[int] = None,
                     jaw_speed_tps: Optional[float] = None,
                     line_speed_tps: Optional[float] = None,
                     extra_limits: Optional[Dict[int, Tuple[int, int, int, float]]] = None
                     ) -> Tuple[dict, List[str]]:
    """A copy of the board's profile with min..max (and home) set for this
    session, and one note per servo it changed. Raises ValueError when a face
    line sits past its pull end (slipped, or the neutrals are stale) or the jaw
    sits open past the open limit. extra_limits: servo -> (low, high, home,
    speed ticks/s) for servos a FaceSession.extra drives (eyes, lids)."""
    ranges = ranges or wg.RANGES
    extra_limits = extra_limits or {}
    upper_ranges = complete_upper_ranges(upper_ranges)
    active = int(profile["active_servo_mask"])
    out = json.loads(json.dumps(profile))
    notes: List[str] = []
    for slot in out["joints"]:
        index = int(slot["index"])
        if not active >> index & 1:
            continue
        if index not in present:
            raise ValueError(f"no present tick for servo {index}")
        tick = present[index]
        old = (slot["min_tick"], slot["max_tick"])
        if index in wg.PULL:
            pull = wg.PULL[index]
            span = ranges[wg.WorkspaceGuard.role_of(index)]
            seen = [pull * neutrals.closed[index], pull * neutrals.open[index]]
            pull_end = max(seen) + span + PULL_SIDE_MARGIN_TICKS
            if pull * tick > pull_end:
                raise ValueError(f"servo {index} sits at {tick}, {pull * tick - pull_end} ticks past "
                                 "its pull end: the line slipped or the neutrals are stale; "
                                 "find them again")
            if min(seen) - pull * tick > LINE_SLACK_MAX_TICKS:
                raise ValueError(f"servo {index} sits at {tick}, {min(seen) - pull * tick} ticks "
                                 "slack of its neutral: a turn-count jump (12 V cycled?) or stale "
                                 "neutrals; find them again (head-knee)")
            slack_end = min(min(seen) - SLACK_SIDE_TICKS, pull * tick - PRESENT_MARGIN_TICKS)
            low, high = (slack_end, pull_end) if pull > 0 else (-pull_end, -slack_end)
            home = neutrals.closed[index]
        elif index in wg.UPPER_PULL:
            pull = wg.UPPER_PULL[index]
            span = min(upper_ranges[wg.WorkspaceGuard.upper_role_of(index)],
                       (upper_spans or {}).get(index, 10**9))
            rest = (upper_neutral or {}).get(index, tick)     # it moves there at Enable
            pull_end = max(pull * tick, pull * rest) + span + PULL_SIDE_MARGIN_TICKS
            slack_end = min(pull * tick, pull * rest) - SLACK_SIDE_TICKS
            low, high = (slack_end, pull_end) if pull > 0 else (-pull_end, -slack_end)
            home = rest
        elif index == wg.JAW:
            low = ((neutrals.jaw_open_limit if jaw_open is None else jaw_open)
                   - JAW_ENVELOPE_OPEN_MARGIN_TICKS)
            high = JAW_ENVELOPE_CLOSED_TICK
            if tick < low:
                raise ValueError(f"the jaw sits at {tick}, open past {low}")
            high = max(high, tick + JAW_PRESENT_MARGIN_TICKS)
            home = min(max(wg.JAW_CLOSED_TICK, low), high)
        elif index in extra_limits:
            low, high, home, _speed = extra_limits[index]
            low = min(low, tick) - EXTRA_ENVELOPE_MARGIN_TICKS
            high = max(high, tick) + EXTRA_ENVELOPE_MARGIN_TICKS
        elif old[0] <= tick <= old[1]:
            continue
        elif index in tuning_core.SKIPPED_SERVOS:
            # A skipped servo's profile is not changed; Enable needs it inside.
            raise ValueError(f"servo {index} (skipped) sits at {tick}, outside its profile "
                             f"range {old[0]}..{old[1]}; Enable would be refused")
        else:
            low, high = tick - OTHER_WINDOW_TICKS, tick + OTHER_WINDOW_TICKS
            home = tick
        if not ENCODER_MIN_TICK <= low < high <= ENCODER_MAX_TICK:
            raise ValueError(f"servo {index}: envelope {low}..{high} is outside the encoder range")
        slot["min_tick"], slot["max_tick"], slot["home_tick"] = int(low), int(high), int(home)
        if (low, high) != old:
            notes.append(f"servo {index}: {old[0]}..{old[1]} -> {low}..{high} (at {tick})")
        if index == wg.JAW and jaw_current_ma is not None and \
                jaw_current_ma != slot["operating_current_ma"]:
            notes.append(f"servo {index}: current {slot['operating_current_ma']} -> {jaw_current_ma} mA")
            slot["operating_current_ma"] = int(jaw_current_ma)
        speed = (jaw_speed_tps if index == wg.JAW else
                 extra_limits[index][3] if index in extra_limits else
                 line_speed_tps if index in wg.PULL or index in wg.UPPER_PULL else None)
        if speed is not None:
            # The firmware moves a goal at most range x this fraction per
            # 2 ms cycle (whole ticks, at least 1): give each face servo
            # enough whole ticks per cycle for the session's speed.
            ticks = max(1, math.ceil(speed * 0.002))
            fraction = (ticks + 0.5) / (high - low)
            if fraction > slot["max_position_fraction_per_control_cycle"]:
                notes.append(f"servo {index}: step {ticks} ticks per cycle "
                             f"({ticks * 500} ticks/s) for {speed:g} ticks/s")
                slot["max_position_fraction_per_control_cycle"] = min(1.0, fraction)
    return out, notes


# An upper-face servo's rest is its latest tuning-panel "neutral" mark (the
# operator's Home, 2026-10-23). Until 2026-11-10 a servo more than this many
# ticks from its mark at Enable rested at its pose at Enable instead; on the
# new head that drove brows 10/16 640 and 335 ticks past their end marks (run
# 192015_face_duchenne). Operator decision 2026-11-10: move it to its mark
# first. Now this is only the "far" level: a farther servo is moved to its mark
# before the script starts (FaceSession, APPROACH_*).
PANEL_MARK_MAX_TICKS = 300
# The approach to the marks is done when every far servo is this close.
APPROACH_DONE_TICKS = 20
# Time for the approach: travel at the line speed limit plus this margin;
# over it, torque goes off.
APPROACH_MARGIN_S = 5.0
# Per-servo shift of the upper-face rest from its panel mark, in its pull
# Per-servo shift of the upper-face rest from its panel mark, in its pull
# direction (ticks); the range moves with it. Right lower lip (operator,
# 2026-10-23): +100 ("if it was 0-735, make it 100-835"), then +100 three more
# times, to make the two lip corners look the same, then -50 for both lips:
# right rests at 3448 (mark 3098) and pulls to 4183, left at 1863 (mark 1813)
# and pulls to 1128. A negative shift moves the rest toward slack.
# Cleared 2026-11-09 for the new head prototype (old head: {3: 350, 4: -50};
# on the new head +350 put the right lower lip 200 ticks past its end mark).
UPPER_REST_SHIFT_TICKS: Dict[int, int] = {}


def choose_upper_neutral(present: Dict[int, int], marks: Optional[Dict[int, Dict[str, dict]]]
                         ) -> Tuple[Dict[int, int], Dict[int, str]]:
    """(servo -> rest tick, servo -> where it came from) for the upper face:
    the panel mark plus its UPPER_REST_SHIFT_TICKS shift, wherever the servo
    sits (operator, 2026-11-10: move it to its mark first; FaceSession does
    that move before the script). A servo without a neutral mark rests at its
    pose at Enable. ValueError when a rest is more than one turn away (the
    one-turn move rule; likely a turn-count jump or a stale mark: re-mark)."""
    rest: Dict[int, int] = {}
    source: Dict[int, str] = {}
    too_far: List[str] = []
    one_turn = tuning_core.headctl_one_turn_ticks()
    for index in wg.UPPER_PULL:
        if index not in present:
            continue
        mark = ((marks or {}).get(index) or {}).get("neutral")
        tick = None if mark is None else mark.get("present_tick")
        if tick is None:
            rest[index] = present[index]
            source[index] = "pose at Enable (no panel mark)"
            continue
        shift = UPPER_REST_SHIFT_TICKS.get(index, 0)
        target = int(tick) + wg.UPPER_PULL[index] * shift
        away = target - present[index]
        # How far it sits outside its own marked range (neutral .. ends).
        # More than its own span outside is not a moved-off servo but a
        # turn-count jump or stale marks (2026-11-10: upper lip 13 sat 3413
        # ticks from its mark after a 12 V cycle; "move first" would have
        # turned it about 300 degrees).
        marked = [int(tick)] + [int(m["present_tick"]) for name, m in
                                ((marks or {}).get(index) or {}).items()
                                if name.startswith("end") and m.get("present_tick") is not None]
        span = max(marked) - min(marked)
        if span == 0:                      # neutral mark only: the role's range
            span = wg.UPPER_RANGES.get(wg.WorkspaceGuard.upper_role_of(index), 0)
        outside = max(min(marked) - present[index], present[index] - max(marked), 0)
        if abs(away) > one_turn or outside > max(PANEL_MARK_MAX_TICKS, span):
            too_far.append(f"{index} (at {present[index]}, mark {tick}, {away:+d}; "
                           f"{outside} outside its marked range of {span})")
            continue
        rest[index] = target
        source[index] = ("panel mark" + (f", shifted {shift:+d}" if shift else "")
                         + (f"; moves {away:+d} to it first" if abs(away) > PANEL_MARK_MAX_TICKS else ""))
    if too_far:
        raise ValueError("upper-face servo too far from its panel marks (turn-count jump after "
                         "a 12 V cycle, or stale marks): " + ", ".join(too_far)
                         + ". Re-mark it in the tuning panel.")
    return rest, source


# Per-servo ticks added to the span from the panel marks (past the end mark,
# by the operator's explicit numbers). Left oculi +50 (2026-10-23: 746 -> 796).
# Cleared 2026-11-09 for the new head prototype (old head: {17: 50}).
UPPER_SPAN_EXTRA_TICKS: Dict[int, int] = {}
# servo -> other servo: its span is capped at the other's span (after the
# extra above). Left oculi 17 to the right oculi 12 (operator, 2026-11-10: the
# left moved too much; marks 880 vs 747 ticks).
UPPER_SPAN_MATCH: Dict[int, int] = {17: 12}


def choose_upper_spans(marks: Optional[Dict[int, Dict[str, dict]]]) -> Tuple[Dict[int, int], List[str]]:
    """(servo -> span from its panel marks, notes): |end - neutral| for each
    upper-face servo with a neutral and an end mark that point the guard's
    pull direction. The session range still caps it."""
    spans: Dict[int, int] = {}
    notes: List[str] = []
    for index, pull in wg.UPPER_PULL.items():
        servo_marks = (marks or {}).get(index) or {}
        neutral = (servo_marks.get("neutral") or {}).get("present_tick")
        end = next(((servo_marks.get(name) or {}).get("present_tick") for name in ("end_1", "end_2")
                    if (servo_marks.get(name) or {}).get("present_tick") is not None), None)
        if neutral is None or end is None:
            continue
        travel = pull * (int(end) - int(neutral))
        if travel <= 0:
            notes.append(f"servo {index}: panel end {end} is not in its pull direction from "
                         f"{neutral}; mark not used")
            continue
        spans[index] = travel + UPPER_SPAN_EXTRA_TICKS.get(index, 0)
    for index, other in UPPER_SPAN_MATCH.items():
        if index in spans and other in spans and spans[index] > spans[other]:
            notes.append(f"servo {index}: span {spans[index]} capped to servo {other}'s {spans[other]}")
            spans[index] = spans[other]
    return spans, notes


# Link --------------------------------------------------------------------------------
def q_for_tick(tick: int, tick_range: Tuple[int, int]) -> float:
    """q that the firmware maps back to exactly this tick. head_control_target_tick()
    truncates minimum + range x q, so half a tick is added (as the firmware's
    own Enable seeding does); tick must be inside the range."""
    minimum, maximum = tick_range
    if not minimum <= tick <= maximum:
        raise ValueError(f"tick {tick} is outside {minimum}..{maximum}")
    return min(1.0, (tick - minimum + 0.5) / (maximum - minimum))


def firmware_tick_for_q(q: float, tick_range: Tuple[int, int]) -> int:
    """head_control_target_tick() in src/control.c, with q as float32."""
    minimum, maximum = tick_range
    q32 = struct.unpack("<f", struct.pack("<f", q))[0]
    return max(minimum, min(maximum, minimum + int((maximum - minimum) * q32)))


class StreamLink(tuning_core.TuningLink):
    """TuningLink (lease renewal, STATE, fault report) for ENABLED mode."""

    def __init__(self, device, clock=time.monotonic) -> None:
        super().__init__(device, clock)
        self.active: List[int] = []
        self.ranges: Dict[int, Tuple[int, int]] = {}
        self.sequence = 0
        self.enable_retries = 0          # BUS-fault retries at Enable (enable())

    def connect(self) -> None:
        reply = self.device.hello()
        self.capabilities = reply["capabilities"]
        missing = [name for name, bit in (("ENABLE_AT_PRESENT_POSE", headctl.CAP_ENABLE_AT_PRESENT_POSE),
                                          ("POSITION_RANGE", headctl.CAP_POSITION_RANGE))
                   if not self.capabilities & bit]
        if missing:
            raise RuntimeError(f"this firmware has no {', '.join(missing)}; flash the new build")
        info = headctl.decode_info(self.device.get(headctl.GET_INFO, headctl.INFO))
        self.active = headctl_active(info["active_servo_mask"])

    def wait_torque_off(self, timeout_s: float = BOARD_READY_TIMEOUT_S) -> tuning_core.ControllerState:
        """Fresh STATE in HOMING_REQUIRED with torque verified off, every
        active servo online (a profile can be changed and Enable asked only
        then; after a commit the board runs discovery again)."""
        deadline = self.clock() + timeout_s
        while True:
            self.poll(0.01)
            state = self.state
            if (state is not None and self.state_is_fresh() and
                    state.state == tuning_core.HOMING_REQUIRED_STATE and
                    state.torque_state == headctl.TORQUE_NAMES.index("off_verified") and
                    all(state.servos[i].online for i in self.active)):
                return state
            if self.clock() > deadline:
                where = (f"{state.state_name}, fault {state.fault_name}, torque "
                         f"{headctl.TORQUE_NAMES[state.torque_state] if state.torque_state is not None and state.torque_state < len(headctl.TORQUE_NAMES) else state.torque_state}"
                         if state is not None else "no STATE")
                raise RuntimeError(f"the board is not torque-off in HOMING_REQUIRED with every "
                                   f"servo online ({where})")

    def apply_envelope(self, profile: dict, say=print) -> Optional[int]:
        """Upload this profile unless the board has it already. Returns the
        new generation, or None when nothing was uploaded."""
        board = headctl.read_profile(self.device)
        if headctl.profile_matches(profile, board):
            return None
        token = headctl.acquire(self.device)
        try:
            generation = headctl.upload_profile(self.device, token, profile, say)
        finally:
            self.device.request(headctl.RELEASE, struct.pack("<I", token))
        self.wait_torque_off()
        return generation

    def enable(self, retry_bus_fault: bool = False, say=print) -> None:
        """Enable at the present pose and wait for ENABLED with torque on.
        retry_bus_fault: when the board faults BUS during the Enable
        preparation while torque is still verified off (2026-11-10: 4 of about
        30 Enables; all servos online, bus 100 % complete), clear the fault
        and enable once more. Never for any other fault, never twice."""
        try:
            self._enable_once()
        except RuntimeError as error:
            state = self.state
            if not (retry_bus_fault and str(error).startswith("Board fault: BUS") and
                    state is not None and state.state == tuning_core.FAULT_STATE and
                    state.torque_state == headctl.TORQUE_NAMES.index("off_verified")):
                raise
            say(f"Enable: {error} Torque is verified off; clearing the fault and enabling once more.")
            ok, response = self.device.request(headctl.CLEAR_FAULT, struct.pack("<I", self.token),
                                               headctl.CLEAR_FAULT_REPLY_TIMEOUT_S)
            if not ok:
                raise RuntimeError(f"{error} (clear-fault refused: "
                                   f"{headctl.describe_firmware_error(response)})") from error
            self.control_lost_reason = None
            self.wait_torque_off()
            self.enable_retries += 1
            self._enable_once()

    def _enable_once(self) -> None:
        self.token = headctl.acquire(self.device)
        self.control_lost_reason = None
        self._renew_due_s = self.clock() + tuning_core.LEASE_RENEW_PERIOD_S
        ok, response = self.device.request(headctl.ENABLE_AT_PRESENT_POSE,
                                           struct.pack("<I", self.token))
        if not ok:
            result = struct.unpack_from("<b", response, 1)[0] if len(response) > 1 else None
            if result is not None and -result == 34:
                raise RuntimeError("Enable refused (ERANGE): a servo is outside its profile "
                                   "min..max. Run 'headctl status' and compare with the profile.")
            raise RuntimeError(f"Enable refused (result {result}): needs HOMING_REQUIRED, "
                               "torque off and fresh feedback")
        deadline = self.clock() + ENABLE_TIMEOUT_S
        while True:
            self.poll(0.01)
            state = self.state
            if self.control_lost_reason is not None:
                raise RuntimeError(self.control_lost_reason)
            if state is not None and state.state == ENABLED_STATE and state.torque_state == TORQUE_ON:
                break
            if self.clock() > deadline:
                raise RuntimeError("the board did not reach ENABLED with torque on in "
                                   f"{ENABLE_TIMEOUT_S:.0f} s")
        reply = headctl.decode_position_range(
            self.device.get(headctl.GET_POSITION_RANGE, headctl.POSITION_RANGE))
        missing = [index for index in self.active if index not in reply["ranges"]]
        if missing:
            raise RuntimeError(f"no position range for servos {missing}")
        self.ranges = reply["ranges"]
        self.sequence = 0

    def send_targets(self, ticks: Dict[int, int]) -> None:
        """One target frame: every active servo, in its range."""
        values = [0.0] * headctl.HEAD_SERVO_COUNT
        for index in self.active:
            values[index] = q_for_tick(ticks[index], self.ranges[index])
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF or 1
        mask = sum(1 << index for index in self.active)
        payload = struct.pack("<IIIB3x20f20f", self.token, self.sequence, mask,
                              COMMAND_MODE_POSITION, *values, *([0.0] * headctl.HEAD_SERVO_COUNT))
        self.device.send(JOINT_TARGETS, payload)


def headctl_active(mask: int) -> List[int]:
    return [index for index in range(headctl.HEAD_SERVO_COUNT) if mask >> index & 1]


# Per-update logic ------------------------------------------------------------------
@dataclass
class Update:
    targets: Dict[int, int]
    drivers: Tuple[int, ...]
    stop: Optional[str] = None


@dataclass
class FaceController:
    """Pure logic for one update: actions + newest STATE -> targets for every
    active servo, or a stop reason (the caller then turns torque off)."""
    guard: wg.WorkspaceGuard
    hold: Dict[int, int]                         # every active servo's goal at Enable
    ranges: Dict[int, Tuple[int, int]]           # the firmware's q = 0 / q = 1 ticks
    preload_ma: int = wg.PRELOAD_MA
    max_pay_out: int = FOLLOW_MAX_PAY_OUT_TICKS
    jaw_guard: tuning_core.JawGuard = field(default_factory=lambda: tuning_core.JawGuard(wg.JAW))
    checks: wg.LiveChecks = field(default_factory=wg.LiveChecks)
    clamped: Dict[int, int] = field(default_factory=dict)     # servo -> times clamped by the profile
    backing_off_since: Optional[float] = None
    _sent: Dict[int, int] = field(default_factory=dict)
    _seen_s: float = float("-inf")

    def __post_init__(self) -> None:
        missing = [i for i in (wg.JAW, *wg.PULL) if i not in self.hold]
        if missing:
            raise ValueError(f"servos {missing} are not active")
        if self.guard.upper_neutral is None:
            self.guard.upper_neutral = {i: self.hold[i] for i in wg.UPPER_PULL if i in self.hold}
        self._sent = dict(self.hold)
        for index in (wg.JAW, *wg.PULL, *self.guard.upper_neutral):
            self.guard.forget(index, self.hold[index])

    def update(self, state: tuning_core.ControllerState, actions: Dict[str, Tuple[float, float]],
               j: float, dt_s: float) -> Update:
        now = state.received_monotonic_s
        new_sample = now > self._seen_s
        self._seen_s = max(self._seen_s, now)
        servos = state.servos
        jaw_reason = self.jaw_guard.observe(state) if new_sample else None
        if jaw_reason == "torque off":
            return Update(dict(self._sent), (), f"jaw over {self.jaw_guard.peak_ma} mA for "
                                                f"{self.jaw_guard.peak_s} s")
        if jaw_reason == "back off" and self.backing_off_since is None:
            self.backing_off_since = now
        if self.backing_off_since is not None:
            actions, j = {}, 0.0
            if abs(servos[wg.JAW].present_tick - self.guard.jaw_closed_tick) <= BACK_OFF_ARRIVE_TICKS:
                return Update(dict(self._sent), (), "jaw held over "
                              f"{self.jaw_guard.hold_ma} mA for {self.jaw_guard.hold_s} s; backed off closed")
            if now - self.backing_off_since > BACK_OFF_TIMEOUT_S:
                return Update(dict(self._sent), (), "jaw back-off did not reach closed in "
                              f"{BACK_OFF_TIMEOUT_S:.0f} s")
        jaw_tick = servos[wg.JAW].present_tick
        driven, followers = self.guard.command(actions, j, jaw_tick=jaw_tick)
        line_j = self.guard.jaw_fraction(jaw_tick)
        wanted = dict(driven)
        for index in (*followers, *(i for i in driven if i != wg.JAW)):
            pull = wg.PULL[index]
            step = (wg.follow_step(servos[index].current_ma, pull, self.preload_ma)
                    if new_sample else 0)
            low, high = self.guard.follower_bounds(index, line_j, self.max_pay_out)
            # No line is pulled in further than neutral + its own group's
            # activity x its range, so a side at rest brings every line back
            # to its neutral tick by position (2026-10-21: the face stayed
            # pulled at the end of every demo while the lines only kept the
            # preload), and when down drives (v < 0), up and diag actively
            # compensate (pay out UP_DIAG_DOWN_COMPENSATION x down's pull) so
            # they never hold the lip corners up against down. Paying out
            # further by load is still allowed up to max_pay_out.
            cap = (self.guard.neutral_at(index, line_j) +
                   pull * line_cap_offset(actions, index, self.guard.ranges))
            if pull > 0:
                high = min(high, cap)
            else:
                low = max(low, cap)
            kept = round(wg.clamp(self._sent[index] + step, low, high))
            if index in driven:
                # A driver goes to its activation target, or further where
                # that is needed to keep the preload: it never goes slack.
                # (A line that took in slack as a follower would otherwise
                # first let go to neutral + activation when it starts to drive.)
                kept = driven[index] if pull * driven[index] >= pull * kept else kept
            wanted[index] = kept
        upper_targets, upper_drivers = self.guard.upper_command(actions)
        wanted.update(upper_targets)
        limited = self.guard.limit_speed(wanted, dt_s)
        drivers = tuple(i for i in (*driven, *upper_drivers) if i != wg.JAW)
        pulled = {i: (wg.PULL[i] * (limited[i] - self.guard.neutral_at(i, line_j))
                      if i in wg.PULL else
                      wg.UPPER_PULL[i] * (limited[i] - self.guard.upper_neutral[i]))
                  for i in drivers}
        watched_currents = {i: servos[i].current_ma
                            for i in (*wg.PULL, *(self.guard.upper_neutral or ()))}
        lag = {i: servos[i].goal_tick - servos[i].present_tick
               for i in (*wg.PULL, *(self.guard.upper_neutral or ()))}
        reason = self.checks.observe(watched_currents, drivers, pulled, lag) \
            if new_sample else None
        targets = dict(self.hold)
        targets.update(limited)
        for index, tick in targets.items():
            low, high = self.ranges[index]
            if not low <= tick <= high:
                self.clamped[index] = self.clamped.get(index, 0) + 1
                targets[index] = min(high, max(low, tick))
                self.guard.forget(index, targets[index])
        self._sent = targets
        return Update(targets, drivers, reason)


def side_of(index: int) -> str:
    return next(side for side, lines in wg.SIDES.items() if index in lines.values())


def side_activity(actions: Dict[str, Tuple[float, float]], side: str) -> float:
    """How far a side is from rest, 0..1: the largest activation it asks for
    (max(d, v) above the middle, -v below it, where d does not count)."""
    d, v = actions.get(side, (0.0, 0.0))
    d, v = wg.clamp(d, 0.0, 1.0), wg.clamp(v, -1.0, 1.0)
    return -v if v < 0.0 else max(d, v)


# When down drives (v < 0), up and diag pay out this fraction of down's pull
# from their neutral so they never hold the lip corners up against down.
# 0.5 until 2026-10-21, then 1.0; 0 since 2026-10-22: with camera neutrals
# (just touching) a fixed 300-tick pay-out ran past the slack and wound up
# and diag back onto their horns the other way (11 and 16 held -56..-65 mA)
# and the sad corners went UP. They now pay out by load only (followers).
UP_DIAG_DOWN_COMPENSATION = 0.0
# Follower preload with camera neutrals: a light tension only, so the lines
# that do not drive never hold the corner (35 mA is the head-ranges mark).
CAMERA_FOLLOW_PRELOAD_MA = 10


def line_activity(actions: Dict[str, Tuple[float, float]], index: int) -> float:
    """How far a line's direction group is active, 0..1: max(d, v) for up/diag
    when v >= 0 (0 when v < 0), and -v for down when v < 0 (0 when v >= 0)."""
    d, v = actions.get(side_of(index), (0.0, 0.0))
    d, v = wg.clamp(d, 0.0, 1.0), wg.clamp(v, -1.0, 1.0)
    if wg.WorkspaceGuard.role_of(index) == "down":
        return -v if v < 0.0 else 0.0
    return max(d, v) if v >= 0.0 else 0.0


def line_cap_offset(actions: Dict[str, Tuple[float, float]], index: int,
                    ranges: Dict[str, int]) -> int:
    """Signed tick offset from neutral for a line's pull-in cap (+ = pull in,
    - = pay out to compensate for an opposing driver)."""
    role = wg.WorkspaceGuard.role_of(index)
    act = line_activity(actions, index)
    if role == "down":
        return round(act * ranges["down"])
    _d, v = actions.get(side_of(index), (0.0, 0.0))
    v = wg.clamp(v, -1.0, 1.0)
    if v < 0.0:
        return -round(UP_DIAG_DOWN_COMPENSATION * (-v) * ranges["down"])
    return round(act * ranges[role])


# Session loop ------------------------------------------------------------------------
Script = Callable[[float], Optional[Tuple[Dict[str, Tuple[float, float]], float]]]


class ExtraServos:
    """Servos a FaceSession drives besides the face (the eyes and lids for
    natural_face.py). limits() gives each one's (low, high, home, speed
    ticks/s) for the session envelope; update() gives its targets for one
    update and a stop reason or None. It must keep its own targets inside its
    limits and speed; the session clamps them to the firmware's range too."""
    servos: Tuple[int, ...] = ()

    def limits(self, present: Dict[int, int]) -> Dict[int, Tuple[int, int, int, float]]:
        raise NotImplementedError

    def update(self, t_s: float, state: tuning_core.ControllerState, dt_s: float,
               approaching: bool) -> Tuple[Dict[int, int], Optional[str]]:
        raise NotImplementedError


def with_jaw_release(neutrals: SessionNeutrals, release: Dict[str, int]) -> SessionNeutrals:
    """The neutrals with each line's open-jaw neutral moved toward slack from
    its own open-jaw neutral, so that the guard pays the line out in
    proportion to the jaw opening (wg.neutral_at). release: role name
    ("down", "diag", "up") or servo index as a string ("8") -> ticks; a
    servo's own entry wins over its role's."""
    def shift(index: int, tick: int) -> int:
        ticks = release.get(str(index), release.get(wg.WorkspaceGuard.role_of(index), 0))
        return tick - wg.PULL[index] * ticks
    opened = {i: shift(i, t) for i, t in neutrals.open.items()}
    return SessionNeutrals(dict(neutrals.closed), opened, neutrals.jaw_closed_tick,
                           neutrals.jaw_open_tick, neutrals.preload_ma,
                           neutrals.controller_session_id, neutrals.path, neutrals.jaw_open_target,
                           neutrals.kind)


class FaceSession:
    """Link + controller + timing. step() does one poll and, when due, one
    update; run() calls step() until the script ends or a stop."""

    def __init__(self, link: StreamLink, neutrals: SessionNeutrals, records: Path,
                 line_speed_tps: float = LINE_SPEED_TPS, jaw_speed_tps: float = JAW_SPEED_TPS,
                 stop_file: Optional[Path] = None, say=print, envelope: bool = True,
                 ranges: Optional[Dict[str, int]] = None,
                 upper_ranges: Optional[Dict[str, int]] = None,
                 jaw_open: Optional[int] = None,
                 panel_marks: Optional[Dict[int, Dict[str, dict]]] = None,
                 jaw_guard: Optional[Dict[str, float]] = None,
                 jaw_current_ma: Optional[int] = None,
                 temp_stop_c: int = TEMP_STOP_C,
                 extra: Optional["ExtraServos"] = None,
                 retry_enable_bus_fault: bool = False) -> None:
        self.link, self.neutrals, self.records = link, neutrals, records
        self.say = say                 # set first: the span notes below are said
        # Any servo at this temperature or more stops the session (TEMP_STOP_C).
        self.temp_stop_c = int(temp_stop_c)
        self.max_temperature_c: Dict[int, int] = {}
        # More servos driven with the face (eyes, lids): see ExtraServos.
        self.extra = extra
        self.retry_enable_bus_fault = retry_enable_bus_fault
        # tuning_core.JawGuard settings (hold_ma, hold_s, peak_ma, peak_s).
        self.jaw_guard = dict(jaw_guard or {})
        self.jaw_current_ma = jaw_current_ma
        # The tuning panel's marks (tuning_core.load_earlier_marks), for the
        # upper face's rest; None = the pose at Enable.
        self.panel_marks = panel_marks
        self.upper_neutral: Optional[Dict[int, int]] = None
        self.upper_spans, notes = choose_upper_spans(panel_marks)
        for note in notes:
            self.say("  " + note)
        self.jaw_open = jaw_open      # the jaw's open limit (j = 1), if not the neutrals' own
        self.line_ranges = dict(wg.RANGES if ranges is None else ranges)
        self.upper_ranges = complete_upper_ranges(upper_ranges)
        self.envelope = envelope
        self.speeds = {"line_speed_tps": line_speed_tps, "jaw_speed_tps": jaw_speed_tps}
        self.stop_file, self.say = stop_file, say
        # Called with the script time after every update (stills, for one).
        self.on_update: Optional[Callable[[float], None]] = None
        # Called after every update with (script time, actions, j, targets sent).
        self.on_command: Optional[Callable[[float, Dict[str, Tuple[float, float]], float,
                                            Dict[int, int]], None]] = None
        self.controller: Optional[FaceController] = None
        self.stopped: Optional[str] = None
        self.requested_stop: Optional[str] = None
        self._last_wanted = None
        self._last_targets: Optional[Dict[int, int]] = None
        self._settle_since: Optional[float] = None
        self._settled = 0
        # Upper-face servos far from their rest at Enable (servo -> rest): the
        # script waits at t = 0 until they are there (APPROACH_*).
        self._approach: Dict[int, int] = {}
        self._approach_until = 0.0
        self._enabled_s = 0.0
        self.started_s = 0.0
        self._due_s = 0.0
        self._last_update_s: Optional[float] = None
        self.log: Dict = {"format": SESSION_FORMAT, "started_utc": datetime.now(timezone.utc).isoformat(),
                          "neutrals": str(neutrals.path) if neutrals.path else None,
                          "speeds": self.speeds, "stream_hz": STREAM_HZ, "stopped": None,
                          "temp_stop_c": self.temp_stop_c}

    def start(self) -> None:
        self.link.connect()
        board_session = getattr(self.link.device, "session_id", None)
        if self.neutrals.controller_session_id is not None and \
                board_session != self.neutrals.controller_session_id:
            raise RuntimeError("the session neutrals are from before a controller restart; "
                               "find them again (head-ranges --neutral-only)")
        try:
            guard = guard_from_neutrals(self.neutrals, self.line_ranges, self.upper_ranges,
                                        jaw_open=self.jaw_open, **self.speeds)
        except ValueError as error:          # before Enable: nothing has moved
            raise RuntimeError(str(error)) from error
        if self.envelope:
            self.make_envelope()
        self.link.enable(self.retry_enable_bus_fault, self.say)
        self.log["enable_retries"] = self.link.enable_retries
        state = self.link.state
        hold = {index: state.servos[index].goal_tick for index in self.link.active}
        if self.upper_neutral is None:                 # no envelope step: choose it now
            self.choose_upper_neutral(hold)
        # The upper face rests at its chosen ticks; the speed limit moves it
        # there from the pose at Enable.
        guard.upper_neutral = {i: t for i, t in self.upper_neutral.items() if i in hold}
        guard.upper_spans = dict(self.upper_spans)
        self.log["upper_spans"] = {str(i): min(self.upper_ranges[wg.WorkspaceGuard.upper_role_of(i)], t)
                                   for i, t in self.upper_spans.items()}
        for index in sorted(self.upper_spans):
            role = wg.WorkspaceGuard.upper_role_of(index)
            self.say(f"  upper-face span {index}: {min(self.upper_ranges[role], self.upper_spans[index])}"
                     f" (panel mark {self.upper_spans[index]}, {role} range {self.upper_ranges[role]})")
        preload = (CAMERA_FOLLOW_PRELOAD_MA if self.neutrals.kind == "camera_knee"
                   else self.neutrals.preload_ma)
        self.controller = FaceController(guard, hold, self.link.ranges, preload,
                                         jaw_guard=tuning_core.JawGuard(wg.JAW, **self.jaw_guard))
        self.log["jaw_guard"] = dict(self.jaw_guard)
        self.log["follow_preload_ma"] = preload
        self.link.send_targets(hold)
        self.started_s = self._last_update_s = self._enabled_s = self.link.clock()
        self._due_s = self.started_s + 1.0 / STREAM_HZ
        self._approach = {i: t for i, t in guard.upper_neutral.items()
                          if abs(t - hold[i]) > PANEL_MARK_MAX_TICKS}
        if self._approach:
            travel = max(abs(t - hold[i]) for i, t in self._approach.items())
            speed = max(1.0, self.speeds["line_speed_tps"])
            self._approach_until = self.started_s + travel / speed + APPROACH_MARGIN_S
            self.say("Moving to the panel marks first: " + ", ".join(
                f"{i} {hold[i]} -> {t}" for i, t in sorted(self._approach.items())))
        self.log["approach"] = {str(i): {"from": hold[i], "to": t} for i, t in self._approach.items()}
        self.log["ranges"] = {str(i): list(r) for i, r in self.link.ranges.items()}
        self.log["hold"] = {str(i): t for i, t in hold.items()}

    def make_envelope(self) -> None:
        state = self.link.wait_torque_off()
        board = headctl.read_profile(self.link.device)
        present = {i: state.servos[i].present_tick for i in self.link.active}
        self.choose_upper_neutral(present)
        try:
            profile, notes = session_envelope(board, self.neutrals, present, self.line_ranges,
                                              self.upper_ranges, self.jaw_open, self.upper_neutral,
                                              self.upper_spans, self.jaw_current_ma,
                                              self.speeds["jaw_speed_tps"],
                                              self.speeds["line_speed_tps"],
                                              self.extra.limits(present) if self.extra else None)
        except ValueError as error:
            raise RuntimeError(f"no session envelope: {error}") from error
        # self.records is the run's folder (records_layout.run_dir).
        before = self.records / "profile_before.json"
        after = self.records / "session_profile.json"
        tuning_core.write_json_atomically(before, board)
        tuning_core.write_json_atomically(after, profile)
        for note in notes:
            self.say("  envelope " + note)
        generation = self.link.apply_envelope(profile, self.say)
        self.say("Session envelope already on the board." if generation is None else
                 f"Session envelope uploaded (generation {generation}).")
        self.log["envelope"] = {"before": str(before), "profile": str(after), "notes": notes,
                                "generation": generation}

    def choose_upper_neutral(self, present: Dict[int, int]) -> None:
        try:
            self.upper_neutral, sources = choose_upper_neutral(present, self.panel_marks)
        except ValueError as error:
            raise RuntimeError(str(error)) from error
        self.log["upper_neutral"] = {str(i): {"tick": t, "from": sources[i]}
                                     for i, t in self.upper_neutral.items()}
        for index in sorted(self.upper_neutral):
            self.say(f"  upper-face rest {index}: {self.upper_neutral[index]} ({sources[index]})")

    def request_stop(self, reason: str) -> None:
        self.requested_stop = reason

    def step(self, script: Script) -> bool:
        """False once stopped (torque is then off)."""
        if self.stopped is not None:
            return False
        link = self.link
        link.poll(0.002)
        now = link.clock()
        if now < self._due_s:
            return True
        reason = self.requested_stop
        if reason is None and self.stop_file is not None and self.stop_file.exists():
            reason = f"stop file {self.stop_file}"
        if reason is None and link.control_lost_reason is not None:
            reason = link.control_lost_reason
        if reason is None and (link.state is None or link.state.state != ENABLED_STATE):
            reason = f"the board left ENABLED ({link.state.state_name if link.state else 'no state'})"
        if reason is None and not link.state_is_fresh():
            reason = "no fresh STATE from the board"
        if reason is None:
            reason = self.check_temperature(link.state)
        approaching = reason is None and bool(self._approach)
        if approaching:
            # Hold the script's first pose (t = 0) and keep its clock at 0
            # until every far upper-face servo is at its rest.
            left = {i for i, t in self._approach.items()
                    if abs(link.state.servos[i].present_tick - t) > APPROACH_DONE_TICKS}
            if not left:
                self.say(f"At the panel marks after {now - self._enabled_s:.1f} s; starting.")
                self.log["approach_s"] = round(now - self._enabled_s, 2)
                self._approach = {}
                approaching = False
            elif now > self._approach_until:
                reason = ("upper face did not reach its panel marks: " + ", ".join(
                    f"{i} at {link.state.servos[i].present_tick} (mark {self._approach[i]})"
                    for i in sorted(left)))
            self.started_s = now
        if reason is None:
            wanted = script(now - self.started_s)
            if wanted is None:
                # The script ended, but a long move may still be on its way
                # (the speed limit): keep the last pose until the targets stop
                # changing, at most SETTLE_MAX_S (2026-10-23: 991-tick oculi
                # were still 39 ticks out when a tour ended).
                if self._settle_since is None:
                    self._settle_since = now
                wanted = self._last_wanted
                if wanted is None or now - self._settle_since > SETTLE_MAX_S:
                    reason = "finished"
            if reason is None:
                self._last_wanted = wanted
                actions, j = wanted
                update = self.controller.update(link.state, actions, j, now - self._last_update_s)
                reason = update.stop
                targets = update.targets
                if self.extra is not None:
                    extra_targets, extra_reason = self.extra.update(
                        now - self.started_s, link.state, now - self._last_update_s, approaching)
                    targets = self.merge_extra(targets, extra_targets)
                    reason = reason or extra_reason
                link.send_targets(targets)
                if self.on_command is not None and not approaching:
                    self.on_command(now - self.started_s, actions, j, targets)
                if self._settle_since is not None and reason is None:
                    self._settled = self._settled + 1 if targets == self._last_targets else 0
                    if self._settled >= SETTLE_UPDATES:
                        reason = "finished"
                self._last_targets = targets
                if self.on_update is not None and not approaching:
                    self.on_update(now - self.started_s)
        self._last_update_s = now
        self._due_s = max(self._due_s + 1.0 / STREAM_HZ, now)
        if reason is not None:
            self.finish(reason)
            return False
        return True

    def check_temperature(self, state: tuning_core.ControllerState) -> Optional[str]:
        """Keep each servo's highest temperature; a reason to stop when one is
        at temp_stop_c or more."""
        hot = []
        for servo in state.servos:
            if not servo.online:
                continue
            if servo.temperature_c > self.max_temperature_c.get(servo.index, -1000):
                self.max_temperature_c[servo.index] = servo.temperature_c
            if servo.temperature_c >= self.temp_stop_c:
                hot.append(f"{servo.index} at {servo.temperature_c} C")
        if hot:
            return f"temperature stop {self.temp_stop_c} C: servo " + ", ".join(hot)
        return None

    def merge_extra(self, targets: Dict[int, int], extra: Dict[int, int]) -> Dict[int, int]:
        """The face targets with the extra servos' targets, each kept inside
        the firmware's range (counted in controller.clamped when it is not)."""
        out = dict(targets)
        for index, tick in extra.items():
            low, high = self.link.ranges[index]
            if not low <= tick <= high:
                self.controller.clamped[index] = self.controller.clamped.get(index, 0) + 1
                tick = min(high, max(low, tick))
            out[index] = int(tick)
        return out

    def run(self, script: Script, sleep=time.sleep) -> str:
        while self.step(script):
            sleep(0.001)
        return self.stopped or "finished"

    def finish(self, reason: str) -> None:
        if self.stopped is not None:
            return
        self.stopped = reason
        self.link.disable()
        self.log["stopped"] = reason
        self.log["max_temperature_c"] = {str(i): t for i, t in sorted(self.max_temperature_c.items())}
        if self.controller is not None:
            self.log["clamped_by_profile"] = {str(i): n for i, n in self.controller.clamped.items()}
        self.say(f"Stopped: {reason}. Torque off.")


# Command log and audio --------------------------------------------------------------
COMMANDS_FILE = "commands.csv"
# The servos whose sent targets the command log keeps: jaw, lower lips, upper
# lips, modiolus lines.
COMMAND_LOG_SERVOS = (wg.JAW, 3, 4, *wg.UPPER_LIP_SERVOS.values(), *sorted(wg.PULL))


class CommandLog:
    """One row per update: host UTC, script time, the mouth action vector and
    the targets sent to the mouth servos. With video.frames.csv (host UTC per
    camera frame) it pairs each frame with the command (lipsync/build_dataset.py).
    Rows are written line by line, so the file is complete up to a crash."""

    def __init__(self, path: Path, clock_utc=lambda: datetime.now(timezone.utc)) -> None:
        import csv

        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._clock_utc = clock_utc
        self._file = open(self.path, "w", encoding="utf-8", newline="", buffering=1)
        self._writer = csv.writer(self._file, lineterminator="\n")
        self._writer.writerow(("host_utc", "t_s", *mouth_actions.MOUTH_COLUMNS,
                               *(f"target_{i}" for i in COMMAND_LOG_SERVOS)))
        self.rows = 0

    def write(self, t_s: float, actions: Dict[str, Tuple[float, float]], j: float,
              targets: Dict[int, int]) -> None:
        if self._file.closed:
            return
        values = mouth_actions.from_actions(actions, j)
        self._writer.writerow((self._clock_utc().isoformat(timespec="microseconds"), f"{t_s:.4f}",
                               *(f"{values[name]:.5f}" for name in mouth_actions.MOUTH_COLUMNS),
                               *(targets.get(i, "") for i in COMMAND_LOG_SERVOS)))
        self.rows += 1

    def close(self) -> None:
        self._file.close()


def wav_duration_s(path: Path) -> float:
    """Length of a WAV file (standard library wave module); ValueError if unreadable."""
    import wave

    try:
        with wave.open(str(path), "rb") as stream:
            return stream.getnframes() / float(stream.getframerate())
    except (wave.Error, EOFError, OSError) as error:
        raise ValueError(f"{path}: not a readable WAV file ({error})") from error


class AudioPlayer:
    """Plays a WAV file with ALSA's aplay (-q: quiet), in its own process."""

    def __init__(self, path: Path, command: Sequence[str] = ("aplay", "-q")) -> None:
        self.path = Path(path)
        self.command = list(command)
        self.process = None
        self.started_utc: Optional[str] = None

    def start(self) -> None:
        import subprocess

        self.started_utc = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        self.process = subprocess.Popen([*self.command, str(self.path)])

    def stop(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except Exception:
                self.process.kill()


def audio_start_s(metadata: Dict[str, str], advance_ms: float) -> float:
    """Script time at which the audio player starts: the trajectory's
    audio_start_s (0 when not given) less the advance. ValueError when that is
    before the stream starts."""
    try:
        start = float(metadata.get("audio_start_s", "0"))
    except ValueError as error:
        raise ValueError(f"bad audio_start_s {metadata.get('audio_start_s')!r}") from error
    at = start - advance_ms / 1000.0
    if not math.isfinite(at) or at < 0.0:
        raise ValueError(f"the audio would start {at:.3f} s into the stream: give the trajectory "
                         "more lead (audio_start_s) or a smaller --audio-advance-ms")
    return at


def parse_camera_mode(text: str) -> Tuple[int, int, int]:
    """'1280x720@15' -> (1280, 720, 15)."""
    size, sep, fps = text.partition("@")
    width, sep2, height = size.partition("x")
    try:
        mode = (int(width), int(height), int(fps))
    except ValueError:
        mode = None
    if not sep or not sep2 or mode is None or min(mode) <= 0:
        raise ValueError(f"bad camera mode {text!r}: use WIDTHxHEIGHT@FPS, e.g. 1280x720@15")
    return mode


# Scripts -----------------------------------------------------------------------------
@dataclass
class Keyframe:
    hold_s: float
    right: Tuple[float, float] = (0.0, 0.0)
    left: Tuple[float, float] = (0.0, 0.0)
    jaw: float = 0.0
    brow: Tuple[float, float] = (0.0, 0.0)
    oculi: Tuple[float, float] = (0.0, 0.0)
    lip: Tuple[float, float] = (0.0, 0.0)
    upper_lip: Tuple[float, float] = (0.0, 0.0)


def _frame_actions(right: Tuple[float, float], left: Tuple[float, float],
                   brow: Tuple[float, float], oculi: Tuple[float, float],
                   lip: Tuple[float, float] = (0.0, 0.0),
                   upper_lip: Tuple[float, float] = (0.0, 0.0)) -> Dict[str, Tuple[float, float]]:
    out: Dict[str, Tuple[float, float]] = {"right": right, "left": left}
    if brow != (0.0, 0.0):
        out["brow"] = brow
    if oculi != (0.0, 0.0):
        out["oculi"] = oculi
    if lip != (0.0, 0.0):
        out["lip"] = lip
    if upper_lip != (0.0, 0.0):
        out[wg.UPPER_LIP_ROLE] = upper_lip
    return out


def keyframe_script(frames: Sequence[Keyframe]) -> Script:
    """Each frame's actions for its hold time (the guard's speed limit makes
    the moves between them); None after the last."""
    ends = []
    total = 0.0
    for frame in frames:
        total += frame.hold_s
        ends.append(total)

    def script(t: float):
        for end, frame in zip(ends, frames):
            if t < end:
                return (_frame_actions(frame.right, frame.left, frame.brow, frame.oculi, frame.lip,
                                       frame.upper_lip),
                        frame.jaw)
        return None
    return script


def parse_keyframe(text: str) -> Keyframe:
    """'hold=3 right=0.5,1 left=0.5,1 jaw=0.3 brow=0.5,0.5 oculi=1,1 lip=1,1 ulip=1,1'
    (missing parts are 0; ulip = the upper lips, right,left)."""
    values: Dict[str, str] = {}
    for item in text.split():
        key, _, value = item.partition("=")
        if key not in ("hold", "right", "left", "jaw", "brow", "oculi", "lip", "ulip") or not value:
            raise ValueError(f"bad keyframe part {item!r}")
        values[key] = value

    def pair(name: str) -> Tuple[float, float]:
        if name not in values:
            return (0.0, 0.0)
        d, v = (float(x) for x in values[name].split(","))
        return (d, v)
    frame = Keyframe(float(values.get("hold", "2")), pair("right"), pair("left"),
                     float(values.get("jaw", "0")), pair("brow"), pair("oculi"), pair("lip"),
                     pair("ulip"))
    numbers = (frame.hold_s, *frame.right, *frame.left, frame.jaw, *frame.brow, *frame.oculi,
               *frame.lip, *frame.upper_lip)
    if not all(math.isfinite(x) for x in numbers) or frame.hold_s <= 0:
        raise ValueError(f"bad keyframe {text!r}")
    return frame


@dataclass
class Waypoint:
    """A pose reached by an eased move of move_s seconds from the one
    before, then held for hold_s."""
    move_s: float
    right: Tuple[float, float] = (0.0, 0.0)
    left: Tuple[float, float] = (0.0, 0.0)
    jaw: float = 0.0
    hold_s: float = 0.0
    brow: Tuple[float, float] = (0.0, 0.0)
    oculi: Tuple[float, float] = (0.0, 0.0)
    lip: Tuple[float, float] = (0.0, 0.0)
    upper_lip: Tuple[float, float] = (0.0, 0.0)


def path_script(waypoints: Sequence[Waypoint], loops: int = 1) -> Script:
    """Smooth moves through the waypoints, from rest, loops times; None
    after. Each move is eased (half a cosine), so it starts and ends at
    rest; the guard's speed limit still applies on top."""
    rest = Waypoint(0.0)
    legs = []                                   # (start s, move s, hold s, from, to)
    t, previous = 0.0, rest
    for _ in range(loops):
        for point in waypoints:
            legs.append((t, point.move_s, point.hold_s, previous, point))
            t += point.move_s + point.hold_s
            previous = point
    total = t

    def mix(a: float, b: float, u: float) -> float:
        return a + (b - a) * u

    def script(t_s: float):
        if t_s >= total:
            return None
        for start, move_s, hold_s, a, b in legs:
            if t_s < start + move_s + hold_s:
                u = 1.0 if move_s <= 0 else min(1.0, (t_s - start) / move_s)
                u = 0.5 - 0.5 * math.cos(math.pi * u)
                right = (mix(a.right[0], b.right[0], u), mix(a.right[1], b.right[1], u))
                left = (mix(a.left[0], b.left[0], u), mix(a.left[1], b.left[1], u))
                brow = (mix(a.brow[0], b.brow[0], u), mix(a.brow[1], b.brow[1], u))
                oculi = (mix(a.oculi[0], b.oculi[0], u), mix(a.oculi[1], b.oculi[1], u))
                lip = (mix(a.lip[0], b.lip[0], u), mix(a.lip[1], b.lip[1], u))
                upper_lip = (mix(a.upper_lip[0], b.upper_lip[0], u),
                             mix(a.upper_lip[1], b.upper_lip[1], u))
                return _frame_actions(right, left, brow, oculi, lip, upper_lip), mix(a.jaw, b.jaw, u)
        return None
    script.total_s = total                      # type: ignore[attr-defined]
    # (time, label) where each move ends, for stills.
    script.marks = [(start + move_s, waypoint_label(b))           # type: ignore[attr-defined]
                    for start, move_s, _hold, _a, b in legs]
    return script


def waypoint_label(point: Waypoint) -> str:
    label = (f"R{point.right[0]:g},{point.right[1]:g}_L{point.left[0]:g},{point.left[1]:g}"
             f"_J{point.jaw:g}")
    if point.brow != (0.0, 0.0) or point.oculi != (0.0, 0.0):
        label += f"_B{point.brow[0]:g},{point.brow[1]:g}_O{point.oculi[0]:g},{point.oculi[1]:g}"
    if point.lip != (0.0, 0.0):
        label += f"_LL{point.lip[0]:g},{point.lip[1]:g}"
    if point.upper_lip != (0.0, 0.0):
        label += f"_UL{point.upper_lip[0]:g},{point.upper_lip[1]:g}"
    return label


def demo_waypoints(leg_s: float = 2.5, hold_s: float = 0.0) -> List[Waypoint]:
    """The range-of-motion demo: each side alone (up, up+diag, diag, rest,
    down, rest), both mirrored, both alternating, the jaw, then the jaw open
    with both up+diag. Every change between up/diag and down passes through
    rest. Down is not driven with the jaw open: the tour of
    2026-10-20 did not try it. The moves follow one another without holds
    (operator, 2026-10-20); the guard's speed limit (300 ticks/s for lines)
    caps the fast parts of a move, so a full up move (700 ticks) takes at
    least about 2.3 s whatever leg_s is."""
    W = Waypoint
    side = [(0.0, 1.0), (1.0, 1.0), (1.0, 0.0), (0.0, 0.0), (0.0, -1.0), (0.0, 0.0)]
    points: List[Waypoint] = []
    points += [W(leg_s, right=p, hold_s=hold_s) for p in side]
    points += [W(leg_s, left=p, hold_s=hold_s) for p in side]
    points += [W(leg_s, right=p, left=p, hold_s=hold_s) for p in side]
    # Alternating, through rest: a side never goes from up/diag straight to
    # down, which would start down pulling while diag is still tight.
    points += [W(leg_s, right=(1.0, 1.0), left=(0.0, -1.0), hold_s=hold_s),
               W(leg_s, hold_s=hold_s),
               W(leg_s, right=(0.0, -1.0), left=(1.0, 1.0), hold_s=hold_s),
               W(leg_s, hold_s=hold_s)]
    points += [W(leg_s, jaw=1.0, hold_s=hold_s), W(leg_s, hold_s=hold_s)]
    points += [W(leg_s, jaw=1.0, hold_s=hold_s),
               W(leg_s, right=(1.0, 1.0), left=(1.0, 1.0), jaw=1.0, hold_s=hold_s),
               W(leg_s, right=(1.0, 0.0), left=(1.0, 0.0), jaw=1.0, hold_s=hold_s),
               W(leg_s, jaw=1.0, hold_s=hold_s),
               W(leg_s, hold_s=1.0)]          # the last hold lets everything arrive
    return points


def upper_waypoints(leg_s: float = 2.5, hold_s: float = 0.5) -> List[Waypoint]:
    """Upper-face tour: right brow (10), left brow (16), both brows, right
    orbicularis oculi (12), left orbicularis oculi (17), both oculi, then
    brows and oculi together, returning to rest between each."""
    W = Waypoint
    return [
        W(leg_s, brow=(1.0, 0.0), hold_s=hold_s),
        W(leg_s, hold_s=0.0),
        W(leg_s, brow=(0.0, 1.0), hold_s=hold_s),
        W(leg_s, hold_s=0.0),
        W(leg_s, brow=(1.0, 1.0), hold_s=hold_s),
        W(leg_s, hold_s=0.0),
        W(leg_s, oculi=(1.0, 0.0), hold_s=hold_s),
        W(leg_s, hold_s=0.0),
        W(leg_s, oculi=(0.0, 1.0), hold_s=hold_s),
        W(leg_s, hold_s=0.0),
        W(leg_s, oculi=(1.0, 1.0), hold_s=hold_s),
        W(leg_s, hold_s=0.0),
        W(leg_s, brow=(1.0, 1.0), oculi=(1.0, 1.0), hold_s=hold_s),
        W(leg_s, hold_s=1.0),
    ]


def duchenne_waypoints(leg_s: float = 2.5, hold_s: float = 1.0) -> List[Waypoint]:
    """Duchenne smile tour: mouth-only smile first for contrast, then
    closed-jaw Duchenne smile (up + diag + orbicularis oculi), then open-jaw
    Duchenne smile with brows and oculi, returning smoothly to rest."""
    W = Waypoint
    return [
        W(leg_s, right=(1.0, 1.0), left=(1.0, 1.0), hold_s=hold_s),
        W(leg_s, hold_s=0.5),
        W(leg_s, right=(1.0, 1.0), left=(1.0, 1.0), oculi=(1.0, 1.0), hold_s=hold_s),
        W(leg_s, hold_s=0.5),
        W(leg_s, jaw=1.0, brow=(0.6, 0.6), hold_s=0.0),
        W(leg_s, right=(1.0, 1.0), left=(1.0, 1.0), jaw=1.0, brow=(0.6, 0.6),
          oculi=(1.0, 1.0), hold_s=hold_s),
        W(leg_s, jaw=1.0, hold_s=0.0),
        W(leg_s, hold_s=1.0),
    ]


def angry_waypoints(leg_s: float = 2.5, hold_s: float = 2.0) -> List[Waypoint]:
    """Angry face: mouth corners pulled down (8, 9; up and diag pay out) and
    both brows to their full range together, then back to rest. Built as the
    sad face (operator, 2026-10-22); renamed angry on 2026-11-09 (operator:
    "the sad looks more like angry" - the brows pull inward and down). No
    oculi. The jaw stays closed (down is never driven with the jaw open)."""
    W = Waypoint
    return [
        W(leg_s, right=(0.0, -1.0), left=(0.0, -1.0), brow=(1.0, 1.0), hold_s=hold_s),
        W(leg_s, hold_s=1.0),
    ]


def expression_waypoints(leg_s: float = 2.5, hold_s: float = 1.0) -> List[Waypoint]:
    """Upper-face tour, then Duchenne smile demo, then angry face demo."""
    return (upper_waypoints(leg_s, hold_s=0.5) +
            duchenne_waypoints(leg_s, hold_s=hold_s) +
            angry_waypoints(leg_s, hold_s=hold_s))


DEMO_TOURS = {
    "rom": demo_waypoints,
    "upper": upper_waypoints,
    "duchenne": duchenne_waypoints,
    "angry": angry_waypoints,
    "expressions": expression_waypoints,
}


def face_preview_controller(clock=time.monotonic) -> tuning_core.SimulatedController:
    """The --preview board for head-face: 20 servos cabled like the head
    (J1 0-4, J2 10-14, J3 5-9, J4 15-19), the six face lines on the tendon
    model with each modiolus' lines coupled (pulling up or diag stretches the
    down line and the other way round), and the jaw on a spring. It is for
    trying the session, not a model of the head."""
    branches = {i: 0 if i < 5 else 2 if i < 10 else 1 if i < 15 else 3 for i in range(20)}
    board = tuning_core.SimulatedController(branches, clock=clock)
    board.operating_ma = {i: 650 if i == wg.JAW else 300 for i in branches}
    board.goal.update({7: 4000, 8: 2250, 11: 2650, 14: 3050, 9: 1700, 15: 500, 19: 2800})
    board.neutral = dict(board.goal)
    taut = PREVIEW_TAUT
    # About 0.2 mA per tick: on the head the up line took 30-125 mA at its
    # full 700 ticks (20261020T215830Z_coact.json).
    board.tendons = {i: (wg.PULL[i], taut[i], 0.2, 0.0) for i in taut}
    # Full up + diag stretches the down line by about 450 ticks here, inside
    # the followers' 600-tick pay-out cap (on the head it moved 0-85 ticks in
    # the combined test, 20261020T215830Z_coact.json).
    board.tendon_links = {8: [(11, 0.3), (14, 0.4)], 9: [(15, 0.3), (19, 0.4)],
                          11: [(8, 0.5)], 14: [(8, 0.5)], 15: [(9, 0.5)], 19: [(9, 0.5)]}
    board.springs = {wg.JAW: (4000, 1.5)}
    # Servos off the six lines (brows, oculi, ...): about 0.1 mA per tick of
    # travel; on the head 12 and 17 read 33-50 mA over 550 ticks (2026-10-21).
    board.load_ma_per_tick = 0.1
    board.state_period_s = 0.01
    return board


PREVIEW_TAUT = {8: 2300, 11: 2700, 14: 3100, 9: 1750, 15: 450, 19: 2750}


def preview_neutrals() -> SessionNeutrals:
    """Neutrals for face_preview_controller(): where each simulated line
    reads the preload (35 mA at 0.2 mA per tick: 175 ticks past taut), as
    head-ranges --neutral-only finds them on the head."""
    closed = {i: t + wg.PULL[i] * round(wg.PRELOAD_MA / 0.2) for i, t in PREVIEW_TAUT.items()}
    return SessionNeutrals(closed, dict(closed), wg.JAW_CLOSED_TICK, wg.JAW_OPEN_TICK,
                           wg.PRELOAD_MA, tuning_core.SimulatedController.SESSION_ID)


# Command line -------------------------------------------------------------------------
# The simulated board's upper lips (13, 18): directions and range for --preview
# only, so the upper-lip code runs there. Not the head's values (none measured).
PREVIEW_UPPER_LIP_PULLS = {13: 1, 18: -1}
PREVIEW_UPPER_LIP_RANGE_TICKS = 400


class SetupError(Exception):
    """A session could not be set up; the message says why (exit code 2)."""


def add_session_arguments(parser: argparse.ArgumentParser) -> None:
    """The arguments every face session takes (head-face, head-viseme)."""
    parser.add_argument("--port", default=headctl.AUTO_PORT)
    parser.add_argument("--records", type=Path, default=DEFAULT_RECORDS)
    parser.add_argument("--preview", action="store_true", help="simulated board")
    parser.add_argument("--neutrals", type=Path,
                        help="session neutral file (default: the newest in --records)")
    parser.add_argument("--line-speed", type=float, default=LINE_SPEED_TPS)
    parser.add_argument("--jaw-speed", type=float, default=JAW_SPEED_TPS)
    parser.add_argument("--camera", choices=("none", "realsense"), default="none",
                        help="record the run: video.webm plus stills in the run folder (bench venv)")
    parser.add_argument("--camera-mode", default="1280x720@15", metavar="WxH@FPS",
                        help="RealSense color mode (default 1280x720@15; face-preview "
                             "--list-modes shows what the camera offers)")
    parser.add_argument("--neutral-offset", action="append", default=[], metavar="ROLE=TICKS",
                        help="move a role's neutral this far toward slack (repeat; default "
                             + " ".join(f"{k}={v}" for k, v in NEUTRAL_SLACK_OFFSET_TICKS.items())
                             + ")")
    parser.add_argument("--range", action="append", default=[], metavar="ROLE=TICKS",
                        help="a role's range from the neutral (repeat; default "
                             + " ".join(f"{k}={v}" for k, v in SESSION_RANGES.items())
                             + "; at most the mapped " + " ".join(
                                 f"{k}={v}" for k, v in wg.RANGES.items()) + ")")
    parser.add_argument("--upper-range", action="append", default=[], metavar="ROLE=TICKS",
                        help="an upper-face role's range from neutral (repeat; default "
                             + " ".join(f"{k}={v}" for k, v in UPPER_SESSION_RANGES.items())
                             + f", {wg.UPPER_LIP_ROLE}=its panel marks; at most the mapped " + " ".join(
                                 f"{k}={v}" for k, v in wg.UPPER_RANGES.items()) + ")")
    parser.add_argument("--jaw-open", type=int,
                        help=f"the jaw's open goal (j = 1) for this run, {JAW_GOAL_MIN_TICK} to 3990 "
                             f"(the mapped limit is {wg.JAW_OPEN_TICK}; the jaw settles short of its "
                             "goal by its load); default: the neutrals' own")
    parser.add_argument("--jaw-current", type=int,
                        help=f"the jaw's current cap for this session, up to {JAW_CURRENT_MAX_MA} mA "
                             "(default: the profile's)")
    parser.add_argument("--jaw-hold-ma", type=int, default=tuning_core.JAW_HOLD_MA)
    parser.add_argument("--jaw-hold-s", type=float, default=tuning_core.JAW_HOLD_S,
                        help="jaw over --jaw-hold-ma this long: back to closed, run ends")
    parser.add_argument("--jaw-peak-ma", type=int, default=tuning_core.JAW_PEAK_MA)
    parser.add_argument("--jaw-peak-s", type=float, default=tuning_core.JAW_PEAK_S,
                        help="jaw over --jaw-peak-ma this long: torque off")
    parser.add_argument("--jaw-release", action="append", default=[], metavar="ROLE=TICKS",
                        help="pay a role's lines (down diag up) or one line (8 9 11 14 15 19) out "
                             "this far at the fully open jaw goal, in proportion to the measured "
                             f"opening (repeat; 0-{JAW_RELEASE_MAX_TICKS}; a line's own value wins)")
    parser.add_argument("--hold-upper-lips", action="store_true",
                        help="do not drive the upper lips 13/18 (they hold their pose), even "
                             "when the panel marks would turn them on")
    parser.add_argument("--retry-enable-bus-fault", action="store_true",
                        help="if the board faults BUS while preparing Enable with torque still "
                             "verified off, clear it and enable once more (2026-11-10: 4 of ~30)")
    parser.add_argument("--temp-stop-c", type=int, default=TEMP_STOP_C,
                        help=f"stop (torque off) when any servo reads this many C (at most {TEMP_STOP_C})")
    parser.add_argument("--keep-profile", action="store_true",
                        help="do not make and upload the session envelope; use the board's profile")


def configure_upper_lips_for_session(preview: bool, marks: Optional[Dict[int, Dict[str, dict]]],
                                     say=print) -> Optional[Dict[str, object]]:
    """Configure the upper lips for this process: on the simulated board with
    PREVIEW_UPPER_LIP_*, on the head from the panel marks. Returns what was
    configured (for the run record), or None when they stay off."""
    wg.clear_upper_lips()
    if preview:
        pulls, range_ticks, notes = dict(PREVIEW_UPPER_LIP_PULLS), PREVIEW_UPPER_LIP_RANGE_TICKS, []
    else:
        pulls, range_ticks, notes = wg.upper_lips_from_marks(marks)
    for note in notes:
        say("  " + note)
    if not pulls:
        say("  upper lips 13/18 stay off: mark a neutral and an end for both in the tuning panel")
        return None
    wg.configure_upper_lips(pulls, range_ticks)
    say(f"  upper lips on: pull {pulls}, range {range_ticks} ticks "
        f"({'simulated' if preview else 'from the panel marks'})")
    return {"pulls": {str(i): p for i, p in pulls.items()}, "range_ticks": range_ticks,
            "from": "preview" if preview else "panel marks"}


def _role_ticks(items: Sequence[str], defaults: Dict[str, int], limits: Dict[str, int],
                flag: str) -> Dict[str, int]:
    out = dict(defaults)
    for item in items:
        role, _, ticks = item.partition("=")
        if role not in limits or not ticks.isdigit() or not 0 < int(ticks) <= limits[role]:
            raise SetupError(f"bad {flag} {item!r} (roles {', '.join(limits)}, 1 to the mapped range)")
        out[role] = int(ticks)
    return out


@dataclass
class SessionSetup:
    session: FaceSession
    link: StreamLink
    device: object
    run: Path
    records: Path
    stop_file: Path
    telemetry: tuning_core.TelemetryLog
    commands: CommandLog
    recorder: Optional[object] = None
    say: Callable[[str], None] = print


def open_session(a: argparse.Namespace, kind: str, label: str, say=print,
                 install_signals: bool = True, extra: Optional[ExtraServos] = None) -> SessionSetup:
    """Check the session arguments, configure the upper lips, make the run
    folder, the session, its telemetry and command logs and (with --camera
    realsense) the recorder. Nothing moves yet: the caller runs
    session.start(), then the script, then close_session(). SetupError when an
    argument or file is wrong."""
    if not (0 < a.line_speed <= SPEED_MAX_TPS and 0 < a.jaw_speed <= SPEED_MAX_TPS):
        raise SetupError(f"--line-speed and --jaw-speed must be 1 to {SPEED_MAX_TPS} ticks/s")
    if a.jaw_current is not None and not 0 < a.jaw_current <= JAW_CURRENT_MAX_MA:
        raise SetupError(f"--jaw-current must be 1 to {JAW_CURRENT_MAX_MA} mA")
    if not (0 < a.jaw_hold_ma < a.jaw_peak_ma <= JAW_CURRENT_MAX_MA and 0 < a.jaw_hold_s <= 3.0
            and 0 < a.jaw_peak_s <= 1.0):
        raise SetupError("Jaw guard: need 0 < hold mA < peak mA <= 650, hold <= 3 s, peak <= 1 s "
                         "(peak 1 s: the operator's 650 mA for 1 s, 2026-10-23)")
    if a.jaw_open is not None and not JAW_GOAL_MIN_TICK <= a.jaw_open <= 3990:
        raise SetupError(f"--jaw-open must be {JAW_GOAL_MIN_TICK} to 3990")
    temp_stop_c = getattr(a, "temp_stop_c", TEMP_STOP_C)
    if not 20 <= temp_stop_c <= TEMP_STOP_C:
        raise SetupError(f"--temp-stop-c must be 20 to {TEMP_STOP_C}")
    release: Dict[str, int] = {}
    for item in getattr(a, "jaw_release", []):
        role, _, ticks = item.partition("=")
        lines = [str(i) for i in wg.PULL]
        if role not in (*wg.RANGES, *lines) or not ticks.isdigit() or \
                not 0 <= int(ticks) <= JAW_RELEASE_MAX_TICKS:
            raise SetupError(f"bad --jaw-release {item!r} (roles {', '.join(wg.RANGES)} or lines "
                             f"{', '.join(lines)}; 0-{JAW_RELEASE_MAX_TICKS} ticks)")
        release[role] = int(ticks)
    try:
        camera_mode = parse_camera_mode(a.camera_mode)
    except ValueError as error:
        raise SetupError(str(error)) from error
    records = a.records / "preview" if a.preview else a.records
    stop_file = records / "STOP"
    if stop_file.exists():
        raise SetupError(f"Remove the stop file first: {stop_file}")
    if a.preview:
        neutrals = preview_neutrals()
        panel_marks = None
    else:
        path = a.neutrals or newest_session_neutrals(records)
        if path is None:
            raise SetupError("No session neutral file: run head-ranges --neutral-only first.")
        neutrals = load_session_neutrals(path)
        panel_marks = tuning_core.load_earlier_marks(records, since=tuning_core.HEAD_MARKS_SINCE)[0]
        # Fail closed (new head, 2026-11-09): an upper-face servo without its
        # own neutral and end mark would be driven over its whole role range.
        spans, _notes = choose_upper_spans(panel_marks)
        unmarked = sorted(index for index in wg.UPPER_PULL if index not in spans)
        if unmarked:
            raise SetupError(
                f"No panel neutral + end mark (since {tuning_core.HEAD_MARKS_SINCE}) for "
                f"upper-face servos {unmarked}: mark them in the tuning panel first.")
    if getattr(a, "hold_upper_lips", False):
        wg.clear_upper_lips()
        upper_lips = None
        say("  upper lips 13/18 held (--hold-upper-lips)")
    else:
        upper_lips = configure_upper_lips_for_session(a.preview, panel_marks, say)
    # Camera neutrals are measured at the skin's rest: no offset unless asked.
    offsets = ({role: 0 for role in NEUTRAL_SLACK_OFFSET_TICKS} if neutrals.kind == "camera_knee"
               else dict(NEUTRAL_SLACK_OFFSET_TICKS))
    for item in a.neutral_offset:
        role, _, ticks = item.partition("=")
        if role not in offsets or not ticks.lstrip("-").isdigit() or not 0 <= int(ticks) <= 600:
            raise SetupError(f"bad --neutral-offset {item!r} (roles {', '.join(offsets)}, 0-600 ticks)")
        offsets[role] = int(ticks)
    ranges = _role_ticks(a.range, SESSION_RANGES, wg.RANGES, "--range")
    upper_ranges = _role_ticks(a.upper_range, complete_upper_ranges(UPPER_SESSION_RANGES),
                               wg.UPPER_RANGES, "--upper-range")
    measured = neutrals
    neutrals = with_jaw_release(shifted_neutrals(neutrals, offsets), release)
    device = face_preview_controller() if a.preview else headctl.Device(a.port)
    run = records_layout.run_dir(records, kind, label)
    say(f"Run folder: {run}")
    link = StreamLink(device)
    session = FaceSession(link, neutrals, run, a.line_speed, a.jaw_speed, stop_file,
                          say=say, envelope=not a.keep_profile,
                          ranges=ranges, upper_ranges=upper_ranges, jaw_open=a.jaw_open,
                          panel_marks=panel_marks,
                          jaw_guard={"hold_ma": a.jaw_hold_ma, "hold_s": a.jaw_hold_s,
                                     "peak_ma": a.jaw_peak_ma, "peak_s": a.jaw_peak_s},
                          jaw_current_ma=a.jaw_current, temp_stop_c=temp_stop_c, extra=extra,
                          retry_enable_bus_fault=getattr(a, "retry_enable_bus_fault", False))
    telemetry = tuning_core.TelemetryLog(run / records_layout.TELEMETRY,
                                         [wg.JAW, *wg.PULL, *wg.UPPER_PULL,
                                          *(extra.servos if extra is not None else ())])
    link.sample_listener = lambda state, pwm: telemetry.observe(state, pwm)
    commands = CommandLog(run / COMMANDS_FILE)
    session.on_command = commands.write
    if install_signals:
        for number in (signal.SIGINT, signal.SIGTERM):
            signal.signal(number, lambda signum, _f: session.request_stop(signal.Signals(signum).name))
    session.log["neutral_offset_ticks"] = offsets
    session.log["jaw_release_ticks"] = release
    session.log["line_ranges"] = ranges
    session.log["upper_ranges"] = upper_ranges
    session.log["upper_lips"] = upper_lips
    session.log["commands"] = str(run / COMMANDS_FILE)
    say("Ranges from neutral: " + " ".join(f"{k}={v}" for k, v in ranges.items())
        + " " + " ".join(f"{k}={v}" for k, v in upper_ranges.items()))
    session.log["neutrals_measured"] = {"closed": measured.closed, "open": measured.open}
    session.log["neutrals_used"] = {"closed": neutrals.closed, "open": neutrals.open}
    say("Neutrals (closed jaw), measured -> used: " + " ".join(
        f"{i}:{measured.closed[i]}->{neutrals.closed[i]}" for i in sorted(neutrals.closed)))
    recorder = None
    if a.camera == "realsense":
        try:
            from face_camera import RealSenseCamera, Recorder  # optional
        except ImportError:
            from .face_camera import RealSenseCamera, Recorder  # type: ignore[no-redef]
        width, height, fps = camera_mode
        recorder = Recorder(RealSenseCamera(width, height, fps), run / records_layout.VIDEO, fps=fps)
        session.log["video"] = str(recorder.video_path)
        session.log["camera_mode"] = {"width": width, "height": height, "fps": fps}
        time.sleep(1.0)
    return SessionSetup(session, link, device, run, records, stop_file, telemetry, commands,
                        recorder, say)


def close_session(setup: SessionSetup, torque_off_still: bool = True) -> Path:
    """Torque off, then stop the recorder (after a still of the relaxed face),
    close the logs and write record.json. Returns its path."""
    session, recorder = setup.session, setup.recorder
    session.finish("ended")
    if recorder is not None:
        # Keep recording a little with torque off: the face relaxes.
        time.sleep(1.0)
        if torque_off_still:
            recorder.still(setup.run / records_layout.STILLS / "99_torque_off_1s.jpg")
        time.sleep(2.0)
        recorder.close()
        session.log["video_frames"] = recorder.frames
        session.log["video_error"] = recorder.error
        setup.say(f"Video: {recorder.video_path} ({recorder.frames} frames"
                  + (f", error: {recorder.error}" if recorder.error else "") + ")")
    setup.telemetry.close()
    setup.commands.close()
    out_path = setup.run / records_layout.RECORD
    tuning_core.write_json_atomically(out_path, session.log)
    setup.say(f"Session: {out_path}")
    try:
        setup.device.close()
    except Exception:
        pass
    return out_path


def main(argv: Optional[List[str]] = None, audio_player_factory=AudioPlayer) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    add_session_arguments(parser)
    parser.add_argument("--frame", action="append", default=[], metavar="'hold=S right=D,V left=D,V jaw=J'",
                        help="a keyframe (repeat); without any, run --demo")
    parser.add_argument("--trajectory", type=Path,
                        help=f"a mouth trajectory CSV ({mouth_actions.TRAJECTORY_FORMAT}, see "
                             "mouth_actions.py); instead of --frame or --demo")
    parser.add_argument("--audio", type=Path,
                        help="a WAV file played (aplay) with the script, starting at the "
                             "trajectory's audio_start_s (0 without one)")
    parser.add_argument("--audio-advance-ms", type=float, default=0.0,
                        help="start the audio this much earlier than the trajectory says "
                             "(the measured audio-vs-motion delay, test T5; may be negative)")
    parser.add_argument("--demo", choices=tuple(DEMO_TOURS), default="rom",
                        help="built-in tour when no --frame is given (default: rom)")
    parser.add_argument("--loops", type=int, default=1, help="demo: how many times")
    parser.add_argument("--leg-s", type=float, default=2.5,
                        help="demo: seconds per move (the speed limit still caps it)")
    a = parser.parse_args(argv)
    try:
        frames = [parse_keyframe(text) for text in a.frame]
    except ValueError as error:
        print(error); return 2
    if a.trajectory is not None and frames:
        print("Give --trajectory or --frame, not both."); return 2
    if not frames and a.trajectory is None and not (1 <= a.loops <= 100 and 0.2 <= a.leg_s <= 30.0):
        print("--loops must be 1-100 and --leg-s 0.2-30 s."); return 2
    if not -2000.0 <= a.audio_advance_ms <= 2000.0:
        print("--audio-advance-ms must be -2000 to 2000."); return 2
    metadata: Dict[str, str] = {}
    waypoints: List[Waypoint] = []
    if a.trajectory is not None:
        try:
            rows, metadata = mouth_actions.read_trajectory(a.trajectory)
        except (OSError, ValueError) as error:
            print(f"Trajectory: {error}"); return 2
        script = mouth_actions.trajectory_script(rows)
        label = a.trajectory.stem
    elif frames:
        script = keyframe_script(frames)
        label = "frames"
    else:
        waypoints = DEMO_TOURS[a.demo](a.leg_s)
        script = path_script(waypoints, a.loops)
        label = a.demo
    player = None
    audio_at_s = 0.0
    if a.audio is not None:
        try:
            audio_s = wav_duration_s(a.audio)
            audio_at_s = audio_start_s(metadata, a.audio_advance_ms)
        except ValueError as error:
            print(f"Audio: {error}"); return 2
        total = getattr(script, "total_s", None)
        if total is not None and audio_at_s + audio_s > total + 0.5:
            print(f"Note: the audio ({audio_s:.2f} s from {audio_at_s:.2f} s) runs past the "
                  f"script's end ({total:.2f} s).", flush=True)
        player = audio_player_factory(a.audio)
    say = lambda text: print(text, flush=True)  # noqa: E731
    try:
        setup = open_session(a, "face", label, say)
    except SetupError as error:
        print(error); return 2
    session = setup.session
    session.log["frames"] = [vars(frame) for frame in frames]
    if a.trajectory is not None:
        session.log["trajectory"] = {"path": str(a.trajectory), "metadata": metadata,
                                     "rows": len(rows), "total_s": script.total_s}
    if not frames and a.trajectory is None:
        session.log["demo"] = {"name": a.demo, "loops": a.loops, "leg_s": a.leg_s,
                               "waypoints": [vars(w) for w in waypoints]}
    hooks: List[Callable[[float], None]] = []
    recorder = setup.recorder
    if recorder is not None:
        stills = setup.run / records_layout.STILLS
        recorder.still(stills / "00_before_enable.jpg")
        marks = list(getattr(script, "marks", []))
        if hasattr(script, "total_s"):        # the last update still has torque on
            marks.append((script.total_s - 0.1, "end_torque_on"))
        taken = [0]

        def take_stills(t_s: float) -> None:
            while marks and t_s >= marks[0][0]:
                _t, mark_label = marks.pop(0)
                taken[0] += 1
                recorder.still(stills / f"{taken[0]:02d}_{mark_label}.jpg")
        hooks.append(take_stills)
    if player is not None:
        session.log["audio"] = {"path": str(a.audio), "start_s": audio_at_s,
                                "advance_ms": a.audio_advance_ms}

        def start_audio(t_s: float) -> None:
            if player.process is None and t_s >= audio_at_s:
                player.start()
                session.log["audio"]["started_utc"] = player.started_utc
                session.log["audio"]["started_script_s"] = t_s
        hooks.append(start_audio)
    if hooks:
        session.on_update = lambda t_s: [hook(t_s) for hook in hooks]
    try:
        try:
            session.start()
        except RuntimeError as error:
            session.finish(f"not started: {error}")
            return 2
        what = (f"the trajectory {a.trajectory.name} ({script.total_s:.1f} s)" if a.trajectory else
                f"{len(frames)} keyframes" if frames else
                f"the {a.demo} demo ({script.total_s:.0f} s)")
        print(f"Enabled at the present pose; streaming {what} at {STREAM_HZ} Hz.", flush=True)
        reason = session.run(script)
        return 0 if reason == "finished" else 1
    finally:
        if player is not None:
            player.stop()
        close_session(setup)


if __name__ == "__main__":
    sys.exit(main())
