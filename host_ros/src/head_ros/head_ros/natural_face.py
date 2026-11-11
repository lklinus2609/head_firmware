"""Natural face: blinks, eye saccades, a fake smile and talking jaw, at once.

    scripts/head-natural --preview
    scripts/head-natural --neutrals tuning_runs/runs/<date>/<time>_knee/neutrals.json \
        --seconds 40 --camera realsense --jaw-release down=60

A test of whether the head can look alive: the eyes (0 right, 2 left, 1
pitch) fixate and jump to new targets, the lids (5 right, 6 left) blink at
their fastest allowed speed, the mouth holds a fake smile (up + diag lines
only; the oculi do not squint, so the smile is a "social", non-Duchenne one)
and the jaw makes syllables in phrases with pauses.

Every move stays inside the operator's marks and the face session's guard:
- eyes and lids: between their tuning-panel marks (eye_tour.tour_limits);
  the two eyes move together by the same ticks (the smaller span limits);
- lines and jaw: face_stream.FaceSession (the session envelope, follower
  regulation, line abort 280 mA, magnet-drop stop, jaw guard, and
  --jaw-release: the down lines pay out in proportion to the jaw opening);
- every servo: the temperature stop (face_stream.TEMP_STOP_C, 52 C);
- eyes and lids: torque off at EXTRA_ABORT_MA.

The timing numbers below (blink gaps, fixation times, syllable lengths) are
chosen to look human; they are not measured on people here.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import math
from pathlib import Path
import random
import sys
from typing import Dict, List, Optional, Sequence, Tuple

try:
    from . import eye_tour, face_stream as fs, face_workspace, records_layout, speech_face, \
        tuning_core, workspace_guard as wg
except ImportError:  # run as a plain script from this directory
    import eye_tour  # type: ignore[no-redef]
    import speech_face  # type: ignore[no-redef]
    import face_workspace  # type: ignore[no-redef]
    import face_stream as fs  # type: ignore[no-redef]
    import records_layout  # type: ignore[no-redef]
    import tuning_core  # type: ignore[no-redef]
    import workspace_guard as wg  # type: ignore[no-redef]

EYES = (0, 2)                    # yaw, moved together (end_1 points the same way on both)
PITCH = 1                        # end_1 = down, end_2 = up (stills 185335_eyes, 2026-11-10)
LIDS = (5, 6)                    # neutral = open, end_1 = closed
EXTRA_SERVOS = (*EYES, PITCH, *LIDS)
# The jaw's current cap for these sessions (--jaw-current default).
JAW_CURRENT_MA = 650
# Eyes and lids: torque off at this current (as head-eyes, under the 300 mA rule).
EXTRA_ABORT_MA = 280
# Oculi rule (2026-10-23): oculi 12/17 slipped at ~4600 ticks/s; this demo does
# not move them. Eyes and lids may go up to the session maximum.
EYE_SPEED_TPS = 2500.0           # eye tour 160401/185607 ran clean at 2500
# Lids: at 5000 ticks/s with a hard start/stop lid 5 hit -294 mA braking at the
# open end (run 200812, abort 280); at 3500 lid 6 read up to 251 mA (200915).
# 2026-11-10 blink tests: closing 5000 t/s at 80000 t/s^2 with the speed
# scaled per lid by its travel: close 0.36 s, peaks lid 5 222 / lid 6 254 mA;
# opening at 5000 read 275 mA, so opening is slower (3500).
LID_SPEED_TPS = fs.SPEED_MAX_TPS
LID_OPEN_SPEED_TPS = 3500.0
# Acceleration limits (ticks/s^2, chosen): each move speeds up and slows down
# smoothly, so the P-only servos do not lag (= current spikes) at start/stop.
EYE_ACCEL_TPS2 = 40000.0
LID_ACCEL_TPS2 = 80000.0
# Jaw: goal 3700 opens 110-118 ticks in syllables at 600-650 mA peaks.
JAW_OPEN_GOAL_TICK = 3700
# Down-line release at the full jaw goal span (2026-11-10, corners held down
# while talking at 3700, change of load jaw open vs closed): line 8 0 -> +7.5,
# 38 -> -3.5, 60 -> -4.2 mA; line 9 0 -> +5.7, 22 -> -4.8, 60 -> -10.1 mA.
JAW_RELEASE_DEFAULT = ("8=30", "9=15")
# A blink: closing ends when both lids are this close to closed (or after the
# closing time at lid speed plus BLINK_CLOSE_SLACK_S), then the lids open.
BLINK_CLOSED_TICKS = 80
BLINK_CLOSE_SLACK_S = 0.15
# Per-lid blink depth (fraction of the closed mark) from the lid ladder.
BLINK_DEPTH = {5: 0.55, 6: 1.0}
# Camera still this long after a blink starts (the lids are shut then).
BLINK_STILL_S = 0.37


@dataclass
class NaturalSettings:
    seconds: float = 40.0
    seed: int = 1
    smile_up: float = 0.6            # fake smile: up activation
    smile_diag: float = 0.5          # and diag activation (both sides)
    talk: bool = True
    jaw_amp: Tuple[float, float] = (0.4, 1.0)
    syllable_s: Tuple[float, float] = (0.18, 0.32)
    phrase_s: Tuple[float, float] = (1.5, 3.5)
    pause_s: Tuple[float, float] = (0.5, 1.5)
    blink_gap_s: Tuple[float, float] = (1.8, 5.0)
    double_blink_p: float = 0.15
    blink_hold_s: float = 0.04
    fixation_s: Tuple[float, float] = (0.4, 1.8)
    gaze_x: float = 0.9              # fraction of the yaw span used
    gaze_y: float = 0.6              # fraction of the pitch span used
    lid_follow_down: float = 0.3     # lids lower by this x downward gaze
    smile_in_s: float = 3.0          # rest (eyes only) before the smile
    ramp_s: float = 1.2
    # A blink closes each lid to this fraction of its closed mark (lid -> fraction).
    # Lid ladder 2026-11-10 (run 204310): lid 5 covers the eye at 0.4-0.6 and
    # uncovers it again past 0.7; lid 6 covers it only at 1.0.
    blink_depth: Dict[int, float] = field(default_factory=lambda: dict(BLINK_DEPTH))
    # Lid ladder (--lid-ladder): the lids step to these closed fractions and
    # hold, nothing else moves; stills show where the eye is covered.
    ladder: Tuple[float, ...] = ()
    ladder_hold_s: float = 1.5


def _ease(u: float) -> float:
    u = min(1.0, max(0.0, u))
    return 0.5 - 0.5 * math.cos(math.pi * u)


@dataclass
class NaturalPlan:
    """The whole show, planned from a seed: smile level and jaw (face(t)),
    gaze targets and blink times (for EyeLids)."""
    settings: NaturalSettings
    syllables: List[Tuple[float, float, float]] = field(default_factory=list)   # start, length, amp
    phrases: List[Tuple[float, float]] = field(default_factory=list)
    saccades: List[Tuple[float, float, float]] = field(default_factory=list)    # t, x, y
    blinks: List[float] = field(default_factory=list)

    @classmethod
    def make(cls, settings: NaturalSettings) -> "NaturalPlan":
        s = settings
        rng = random.Random(s.seed)
        plan = cls(s)
        end = s.seconds
        if s.ladder:
            plan.saccades = [(0.0, 0.0, 0.0)]
            return plan
        # Talking: phrases of syllables between the smile's ramp-in and ramp-out.
        t = s.smile_in_s + s.ramp_s + 0.5
        talk_end = end - s.ramp_s - 1.5
        while s.talk and t < talk_end:
            length = min(rng.uniform(*s.phrase_s), talk_end - t)
            phrase_end = t + length
            plan.phrases.append((t, phrase_end))
            while t < phrase_end:
                syl = rng.uniform(*s.syllable_s)
                if t + syl > phrase_end:
                    break
                plan.syllables.append((t, syl, rng.uniform(*s.jaw_amp)))
                t += syl
            t = phrase_end + rng.uniform(*s.pause_s)
        # Gaze: fixations, then a jump to a new point.
        t = rng.uniform(*s.fixation_s)
        while t < end - 1.0:
            plan.saccades.append((t, rng.uniform(-s.gaze_x, s.gaze_x), rng.uniform(-s.gaze_y, s.gaze_y)))
            t += rng.uniform(*s.fixation_s)
        plan.saccades.append((end - 1.0, 0.0, 0.0))
        # Blinks: random gaps, sometimes a double blink.
        t = rng.uniform(0.5, s.blink_gap_s[0])
        while t < end - 1.0:
            plan.blinks.append(t)
            if rng.random() < s.double_blink_p:
                plan.blinks.append(t + 0.45)
            t += rng.uniform(*s.blink_gap_s)
        return plan

    def ladder_fraction(self, t: float) -> float:
        s = self.settings
        if not s.ladder or t < 0.5:
            return 0.0
        step = int((t - 0.5) // s.ladder_hold_s)
        return s.ladder[step] if step < len(s.ladder) else 0.0

    def smile(self, t: float) -> float:
        s = self.settings
        rise = _ease((t - s.smile_in_s) / s.ramp_s)
        fall = _ease((s.seconds - t) / s.ramp_s)
        return min(rise, fall)

    def jaw(self, t: float) -> float:
        for start, length, amp in self.syllables:
            if start <= t < start + length:
                return amp * math.sin(math.pi * (t - start) / length) ** 2
            if start > t:
                break
        return 0.0

    def gaze(self, t: float) -> Tuple[float, float]:
        x = y = 0.0
        for start, sx, sy in self.saccades:
            if start > t:
                break
            x, y = sx, sy
        return x, y

    def face(self, t: float):
        """FaceSession script: (actions, jaw j), None after the end."""
        if t > self.settings.seconds:
            return None
        k = self.smile(t)
        if self.settings.smile_up < 0.0:            # corners held down: down drives
            side = (0.0, k * self.settings.smile_up)
        else:
            side = (k * self.settings.smile_diag, k * self.settings.smile_up)
        return {"right": side, "left": side}, self.jaw(t)

    def marks(self) -> List[Tuple[float, str]]:
        """Times for camera stills (script time, label)."""
        s = self.settings
        if s.ladder:
            return [(0.5 + (n + 1) * s.ladder_hold_s - 0.2, f"lid_{round(f * 100):03d}")
                    for n, f in enumerate(s.ladder)]
        out = [(self.settings.smile_in_s + self.settings.ramp_s + 0.3, "smile")]
        out += [(start + length / 2, f"syllable_{n}") for n, (start, length, _a)
                in enumerate(self.syllables[:6])]
        # A blink closes in about 0.36 s on the head (2026-11-10, lids 5000 t/s, 80000 t/s^2).
        out += [(t + BLINK_STILL_S, f"blink_{n}") for n, t in enumerate(self.blinks[:4])]
        return sorted(out)


# Speech (--say): how the visemes change the fake smile (chosen). Spread adds
# up/diag activation; round takes part of the smile away (the tendons can
# only pull outward, so rounding = relaxing the corners).
SPEECH_SPREAD_UP = 0.25
SPEECH_SPREAD_DIAG = 0.3
SPEECH_ROUND_RELAX = 0.7
# --lips: the lower lips (3/4, "lip") and upper lips (13/18, "upper_lip")
# press for p/b/m by this fraction of their marked range (chosen; their
# effect on this head is not measured yet).
SPEECH_SEAL_LIP = 0.5
# Gaze while speaking (chosen, from how people look in conversation): at a
# phrase start the eyes look away with this chance, then come back to the
# listener (the camera, gaze 0,0) for the rest of the phrase.
SPEECH_AVERT_P = 0.4
SPEECH_AVERT_X = 0.45
SPEECH_AVERT_Y = -0.25
SPEECH_AVERT_S = (0.4, 0.9)
# Blinks at phrase ends (people blink more at pauses), with this chance,
# unless a blink was within SPEECH_BLINK_GAP_S.
SPEECH_BLINK_P = 0.7
SPEECH_BLINK_GAP_S = 1.2


@dataclass
class SpeechPlan(NaturalPlan):
    """NaturalPlan with a spoken text (speech_face.SpeechTrack) instead of
    random syllables: the jaw and lip corners follow the visemes, gaze and
    blinks follow the phrases."""
    track: Optional[object] = None
    lips: bool = False

    @classmethod
    def make_speech(cls, settings: NaturalSettings, track, lips: bool = False) -> "SpeechPlan":
        base = NaturalPlan.make(NaturalSettings(**{**vars(settings), "talk": False}))
        plan = cls(settings, track=track, lips=lips)
        rng = random.Random(settings.seed + 1000)
        # Gaze: the base plan's saccades outside speech; while speaking,
        # avert-and-return at each phrase start, small moves inside a phrase.
        first, last = track.phrases[0][0], track.phrases[-1][1]
        saccades = [s for s in base.saccades if s[0] < first - 0.3 or s[0] > last + 0.5]
        for start, end in track.phrases:
            if rng.random() < SPEECH_AVERT_P:
                side = rng.choice((-1.0, 1.0))
                saccades.append((max(0.0, start - 0.15), side * SPEECH_AVERT_X, SPEECH_AVERT_Y))
                back = start + rng.uniform(*SPEECH_AVERT_S)
            else:
                back = max(0.0, start - 0.1)
            t = back
            while t < end:
                saccades.append((t, rng.uniform(-0.12, 0.12), rng.uniform(-0.08, 0.08)))
                t += rng.uniform(0.6, 1.4)
        saccades.sort()
        if not saccades or saccades[-1][0] < settings.seconds - 1.0:
            saccades.append((settings.seconds - 1.0, 0.0, 0.0))
        plan.saccades = saccades
        # Blinks: the base plan's outside speech, plus one at most phrase ends.
        blinks = [b for b in base.blinks if b < first - 0.5 or b > last + 0.5]
        for _start, end in track.phrases:
            if rng.random() < SPEECH_BLINK_P and all(abs(b - end) > SPEECH_BLINK_GAP_S for b in blinks):
                blinks.append(end + 0.05)
        plan.blinks = sorted(b for b in blinks if b < settings.seconds - 1.0)
        return plan

    def jaw(self, t: float) -> float:
        return self.track.mouth(t).jaw

    def face(self, t: float):
        if t > self.settings.seconds:
            return None
        s = self.settings
        k = self.smile(t)
        m = self.track.mouth(t)
        keep = 1.0 - SPEECH_ROUND_RELAX * m.round
        if s.smile_up < 0.0:
            side = (0.0, k * s.smile_up * keep)
        else:
            side = (min(1.0, k * s.smile_diag * keep + SPEECH_SPREAD_DIAG * m.spread),
                    min(1.0, k * s.smile_up * keep + SPEECH_SPREAD_UP * m.spread))
        actions: Dict[str, object] = {"right": side, "left": side}
        if self.lips:
            seal = SPEECH_SEAL_LIP * m.seal
            actions["lip"] = (seal, seal)
            actions["upper_lip"] = (seal, seal)
        return actions, m.jaw

    def marks(self) -> List[Tuple[float, str]]:
        out = [(self.settings.smile_in_s + self.settings.ramp_s + 0.3, "smile")]
        # A still at the widest jaw and at each p/b/m seal, a few of each.
        step = 0.02
        t, opens, seals = self.track.start_s, [], []
        while t < self.track.end_s:
            m = self.track.mouth(t)
            if m.jaw > 0.7 and (not opens or t - opens[-1] > 0.6):
                opens.append(t)
            if m.seal > 0.9 and (not seals or t - seals[-1] > 0.6):
                seals.append(t)
            t += step
        out += [(t, f"open_{n}") for n, t in enumerate(opens[:4])]
        out += [(t, f"seal_{n}") for n, t in enumerate(seals[:3])]
        out += [(t + BLINK_STILL_S, f"blink_{n}") for n, t in enumerate(self.blinks[:3])]
        return sorted(out)


EXPLORE_KEYS = ("right", "left", "jaw", "brow", "oculi", "lip", "upper_lip", "gaze", "lids")
# Face-session keys (gaze and lids go to EyeLids instead).
FACE_KEYS = ("right", "left", "brow", "oculi", "lip", "upper_lip")


@dataclass
class ExplorePlan(NaturalPlan):
    """--explore: random actions over the whole ML action space
    (face_workspace.project), a new uniformly drawn target every
    0.4-1.5 s, eased moves between them, plus the usual random blinks.
    It checks that every part of the action space can be commanded."""
    waypoints: List[Tuple[float, Dict[str, object]]] = field(default_factory=list)

    @classmethod
    def make(cls, settings: NaturalSettings) -> "ExplorePlan":
        base = NaturalPlan.make(NaturalSettings(seconds=settings.seconds, seed=settings.seed,
                                                talk=False, smile_up=0.0))
        plan = cls(settings, blinks=base.blinks)
        rng = random.Random(settings.seed + 1000)
        rest = {"right": (0.0, 0.0), "left": (0.0, 0.0), "jaw": 0.0, "brow": (0.0, 0.0),
                "oculi": (0.0, 0.0), "lip": (0.0, 0.0), "upper_lip": (0.0, 0.0),
                "gaze": (0.0, 0.0), "lids": (0.0, 0.0)}
        plan.waypoints.append((0.0, rest))
        t = 1.0
        unit = lambda: (rng.random(), rng.random())          # noqa: E731
        while t < settings.seconds - 2.5:
            plan.waypoints.append((t, {
                "right": (rng.random(), rng.uniform(-1.0, 1.0)),
                "left": (rng.random(), rng.uniform(-1.0, 1.0)),
                "jaw": rng.random() if rng.random() < 0.6 else 0.0,
                "brow": unit(), "oculi": unit(), "lip": unit(), "upper_lip": unit(),
                "gaze": (rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)),
                "lids": (rng.uniform(0.0, 0.6), rng.uniform(0.0, 0.6))}))
            t += rng.uniform(0.4, 1.5)
        plan.waypoints.append((settings.seconds - 1.5, rest))
        plan.saccades = [(0.0, 0.0, 0.0)]
        return plan

    def action(self, t: float) -> Dict[str, object]:
        """The eased action at t (each value between two waypoints)."""
        points = self.waypoints
        k = 0
        while k + 1 < len(points) and points[k + 1][0] <= t:
            k += 1
        if k + 1 >= len(points):
            return dict(points[-1][1])
        (t0, a0), (t1, a1) = points[k], points[k + 1]
        u = _ease((t - t0) / max(1e-6, min(0.35, t1 - t0)))
        out: Dict[str, object] = {}
        for key in EXPLORE_KEYS:
            x, y = a0[key], a1[key]
            if isinstance(x, tuple):
                out[key] = tuple(p + u * (q - p) for p, q in zip(x, y))
            else:
                out[key] = x + u * (y - x)
        return out

    def gaze(self, t: float) -> Tuple[float, float]:
        return self.action(t)["gaze"]                       # type: ignore[return-value]

    def ladder_fraction(self, t: float):
        right, left = self.action(t)["lids"]                # type: ignore[misc]
        return {5: right, 6: left}

    def face(self, t: float):
        if t > self.settings.seconds:
            return None
        action = self.action(t)
        return {k: action[k] for k in FACE_KEYS}, action["jaw"]


@dataclass
class ValidatePlan(ExplorePlan):
    """--validate N: N held poses over the whole action space, each
    dimension stratified (Latin hypercube: every dimension's range is cut in
    N equal bins and each bin is used once), MOVE_S eased move then HOLD_S
    hold, rest between; no blinks. face_workspace.py --report measures each
    hold's last REPORT_WINDOW_S."""
    poses: List[Tuple[float, float]] = field(default_factory=list)   # (hold start, hold end)

    MOVE_S = 0.5
    HOLD_S = 1.4

    @classmethod
    def make_validate(cls, n: int, seed: int) -> "ValidatePlan":
        rng = random.Random(seed + 2000)

        def column(low: float, high: float) -> List[float]:
            values = [low + (k + rng.random()) * (high - low) / n for k in range(n)]
            rng.shuffle(values)
            return values
        dims = {"rd": column(0, 1), "rv": column(-1, 1), "ld": column(0, 1), "lv": column(-1, 1),
                "jaw": column(0, 1), "brow_r": column(0, 1), "brow_l": column(0, 1),
                "oculi_r": column(0, 1), "oculi_l": column(0, 1), "lip_r": column(0, 1),
                "lip_l": column(0, 1), "ulip_r": column(0, 1), "ulip_l": column(0, 1),
                "gx": column(-1, 1), "gy": column(-1, 1), "lid_r": column(0, 0.8),
                "lid_l": column(0, 0.8)}
        step = cls.MOVE_S + cls.HOLD_S
        seconds = 1.0 + n * step + 2.0
        plan = cls(NaturalSettings(seconds=seconds, seed=seed, talk=False, smile_up=0.0))
        rest = ExplorePlan.make(NaturalSettings(seconds=10.0)).waypoints[0][1]
        plan.waypoints.append((0.0, rest))
        t = 1.0
        for k in range(n):
            pose = {"right": (dims["rd"][k], dims["rv"][k]), "left": (dims["ld"][k], dims["lv"][k]),
                    "jaw": dims["jaw"][k], "brow": (dims["brow_r"][k], dims["brow_l"][k]),
                    "oculi": (dims["oculi_r"][k], dims["oculi_l"][k]),
                    "lip": (dims["lip_r"][k], dims["lip_l"][k]),
                    "upper_lip": (dims["ulip_r"][k], dims["ulip_l"][k]),
                    "gaze": (dims["gx"][k], dims["gy"][k]), "lids": (dims["lid_r"][k], dims["lid_l"][k])}
            plan.waypoints.append((t, pose))
            plan.poses.append((t + cls.MOVE_S, t + step))
            t += step
        plan.waypoints.append((t, rest))
        plan.saccades = [(0.0, 0.0, 0.0)]
        return plan

    def action(self, t: float) -> Dict[str, object]:
        points = self.waypoints
        k = 0
        while k + 1 < len(points) and points[k + 1][0] <= t:
            k += 1
        if k + 1 >= len(points):
            return dict(points[-1][1])
        (t0, a0), (_t1, a1) = points[k], points[k + 1]
        if k == 0:
            return dict(a0)
        # Hold the previous pose until its hold ends, then move.
        (tp, ap) = points[k - 1] if k > 0 else points[k]
        u = _ease((t - t0) / self.MOVE_S)
        out: Dict[str, object] = {}
        for key in EXPLORE_KEYS:
            x, y = ap[key], a0[key]
            out[key] = (tuple(p + u * (q - p) for p, q in zip(x, y)) if isinstance(x, tuple)
                        else x + u * (y - x))
        return out


class EyeLids(fs.ExtraServos):
    """Eyes and lids for FaceSession: gaze and blinks from the plan, inside
    the panel marks, speed-limited; torque off at EXTRA_ABORT_MA."""
    servos = EXTRA_SERVOS

    def __init__(self, plan: NaturalPlan, limits: Dict[int, dict],
                 eye_speed_tps: float = EYE_SPEED_TPS, lid_speed_tps: float = LID_SPEED_TPS,
                 eye_accel: float = EYE_ACCEL_TPS2, lid_accel: float = LID_ACCEL_TPS2,
                 lid_open_speed_tps: Optional[float] = None) -> None:
        missing = [i for i in EXTRA_SERVOS if i not in limits]
        if missing:
            raise ValueError(f"no panel marks for eye/lid servos {missing}")
        self.plan, self.lim = plan, limits
        self.eye_speed_tps, self.lid_speed_tps = float(eye_speed_tps), float(lid_speed_tps)
        self.eye_accel_tps2, self.lid_accel_tps2 = float(eye_accel), float(lid_accel)
        # Opening may be slower than closing (a human blink closes faster than
        # it opens); None = the same speed.
        self.lid_open_speed_tps = float(lid_open_speed_tps or lid_speed_tps)
        self._vel: Dict[int, float] = {i: 0.0 for i in EXTRA_SERVOS}
        self.peak_ma: Dict[int, int] = {}
        n = {i: limits[i]["neutral"] for i in EXTRA_SERVOS}
        self.neutral = n
        # Yaw: the same ticks for both eyes; each direction limited by the eye
        # with the smaller span that way.
        ends = {i: limits[i]["ends"] for i in EYES}
        self.yaw_pos_ticks = min(abs(ends[i][0] - n[i]) for i in EYES)      # x > 0: toward end_1
        self.yaw_neg_ticks = min(abs(ends[i][1] - n[i]) for i in EYES)      # x < 0: toward end_2
        self.yaw_sign = {i: (1 if ends[i][0] > n[i] else -1) for i in EYES}
        if any((ends[i][1] > n[i]) == (ends[i][0] > n[i]) for i in EYES):
            raise ValueError("an eye's end_1 and end_2 are on the same side of its neutral")
        self._last: Dict[int, float] = {}
        travel = {i: plan.settings.blink_depth.get(i, 1.0) * abs(limits[i]["ends"][0] - n[i])
                  for i in LIDS}
        longest = max(travel.values()) or 1.0
        self.lid_share = {i: max(0.2, travel[i] / longest) for i in LIDS}
        self._blink_k = 0                  # next plan.blinks index
        self._blink: Optional[str] = None  # None, "closing", "holding", "opening"
        self._blink_t = 0.0
        self._blink_start = 0.0
        self.blink_log: List[Dict[str, float]] = []
        self.saccade_log: List[Dict[str, float]] = []
        self._saccade_k = 0
        # The lids' target without a blink (gaze-down lowering), for the
        # end of a blink.
        self._lid_rest: Optional[Dict[int, int]] = None
        self._saccade_pending: Optional[Tuple[float, Dict[int, int]]] = None

    def limits(self, present: Dict[int, int]) -> Dict[int, Tuple[int, int, int, float]]:
        return {i: (self.lim[i]["low"], self.lim[i]["high"], self.neutral[i],
                    self.lid_speed_tps if i in LIDS else self.eye_speed_tps) for i in EXTRA_SERVOS}

    # Targets ---------------------------------------------------------------------
    def eye_targets(self, x: float, y: float) -> Dict[int, int]:
        out = {}
        for i in EYES:
            ticks = x * self.yaw_pos_ticks if x >= 0 else x * self.yaw_neg_ticks
            out[i] = round(self.neutral[i] + self.yaw_sign[i] * ticks)
        n, (down, up) = self.neutral[PITCH], self.lim[PITCH]["ends"]
        out[PITCH] = round(n + (y * (up - n) if y >= 0 else -y * (down - n)))
        return out

    def lid_targets(self, closed) -> Dict[int, int]:
        """closed: one fraction for both lids, or lid -> fraction."""
        out = {}
        for i in LIDS:
            c = min(1.0, max(0.0, closed[i] if isinstance(closed, dict) else closed))
            out[i] = round(self.neutral[i] + c * (self.lim[i]["ends"][0] - self.neutral[i]))
        return out

    def _blink_step(self, t: float, present: Dict[int, int]) -> float:
        """The blink's closed fraction (0 open, 1 closed) at t."""
        plan = self.plan
        if self._blink is None and self._blink_k < len(plan.blinks) and t >= plan.blinks[self._blink_k]:
            self._blink_k += 1
            self._blink, self._blink_t, self._blink_start = "closing", t, t
        if self._blink == "closing":
            depth = plan.settings.blink_depth
            span = max(depth[i] * abs(self.lim[i]["ends"][0] - self.neutral[i]) for i in LIDS)
            shut = self.lid_targets(depth)
            if all(abs(present[i] - shut[i]) <= BLINK_CLOSED_TICKS for i in LIDS) or \
                    t - self._blink_t > (span / self.lid_speed_tps + self.lid_speed_tps / self.lid_accel_tps2
                                         + BLINK_CLOSE_SLACK_S):
                self.blink_log.append({"start_s": round(self._blink_start, 3),
                                       "closed_s": round(t - self._blink_start, 3)})
                self._blink, self._blink_t = "holding", t
            return depth
        if self._blink == "holding":
            if t - self._blink_t >= self.plan.settings.blink_hold_s:
                self._blink, self._blink_t = "opening", t
            return plan.settings.blink_depth
        if self._blink == "opening":
            open_ = self._lid_rest or self.lid_targets(0.0)     # where the lids rest now
            span = max(abs(self.lim[i]["ends"][0] - self.neutral[i]) for i in LIDS)
            if all(abs(present[i] - open_[i]) <= BLINK_CLOSED_TICKS for i in LIDS) or \
                    t - self._blink_t > span / self.lid_open_speed_tps + 0.5:
                self.blink_log[-1]["total_s"] = round(t - self._blink_start, 3)
                self._blink = None
        return 0.0

    def update(self, t_s: float, state: tuning_core.ControllerState, dt_s: float,
               approaching: bool) -> Tuple[Dict[int, int], Optional[str]]:
        servos = {s.index: s for s in state.servos}
        for i in EXTRA_SERVOS:
            current = servos[i].current_ma
            self.peak_ma[i] = max(self.peak_ma.get(i, 0), abs(current))
            if abs(current) >= EXTRA_ABORT_MA:
                return {}, f"servo {i} at {current} mA (eye/lid abort {EXTRA_ABORT_MA} mA)"
        present = {i: servos[i].present_tick for i in EXTRA_SERVOS}
        if not self._last:
            self._last = {i: float(servos[i].goal_tick) for i in EXTRA_SERVOS}
        if approaching or t_s > self.plan.settings.seconds:
            wanted = {**self.eye_targets(0.0, 0.0), **self.lid_targets(0.0)}
        else:
            x, y = self.plan.gaze(t_s)
            wanted = self.eye_targets(x, y)
            down = self.plan.settings.lid_follow_down * max(0.0, -y)
            extra = self.plan.ladder_fraction(t_s)
            other = {i: max(down, extra[i] if isinstance(extra, dict) else extra) for i in LIDS}
            self._lid_rest = self.lid_targets(other)
            blink = self._blink_step(t_s, present)
            closed = {i: max(blink[i] if isinstance(blink, dict) else blink, other[i]) for i in LIDS}
            wanted.update(self.lid_targets(closed))
            self._log_saccade(t_s, wanted, present)
        out = {}
        for i, target in wanted.items():
            low, high = self.lim[i]["low"], self.lim[i]["high"]
            target = min(high, max(low, target))                 # never past a mark
            lid = i in LIDS
            # A lid moving toward open (back to neutral) uses the opening speed.
            # Each lid's speed scales with its blink travel, so both lids take
            # the same time and the shorter one runs slower (less current).
            opening = lid and abs(target - self.neutral[i]) < abs(self._last[i] - self.neutral[i])
            speed = ((self.lid_open_speed_tps if opening else self.lid_speed_tps) * self.lid_share[i]
                     if lid else self.eye_speed_tps)
            self._last[i], self._vel[i] = smooth_step(
                self._last[i], self._vel[i], target, speed,
                self.lid_accel_tps2 if lid else self.eye_accel_tps2, dt_s)
            out[i] = round(self._last[i])
        return out, None

    def _log_saccade(self, t: float, wanted: Dict[int, int], present: Dict[int, int]) -> None:
        plan = self.plan
        while self._saccade_k < len(plan.saccades) and plan.saccades[self._saccade_k][0] <= t:
            start = plan.saccades[self._saccade_k][0]
            self._saccade_k += 1
            self._saccade_pending = (start, {i: wanted[i] for i in (*EYES, PITCH)})
        if self._saccade_pending is not None:
            start, goal = self._saccade_pending
            if all(abs(present[i] - goal[i]) <= 20 for i in goal):
                self.saccade_log.append({"start_s": round(start, 3), "reached_s": round(t - start, 3)})
                self._saccade_pending = None


def smooth_step(position: float, velocity: float, target: float, speed_max_tps: float, accel_tps2: float,
                dt_s: float) -> Tuple[float, float]:
    """One dt of a move toward target with speed <= speed_max_tps and acceleration
    <= accel_tps2, slowing down in time to stop at the target (trapezoid
    profile). Returns (position, velocity); never passes the target."""
    if dt_s <= 0.0:
        return position, velocity
    error = target - position
    if abs(error) < 0.5:
        return target, 0.0
    direction = 1.0 if error > 0 else -1.0
    # The fastest speed that can still stop at the target with whole steps of
    # dt at this deceleration (the discrete form of sqrt(2 a d)).
    a_dt = accel_tps2 * dt_s
    stop_speed = a_dt * (-0.5 + math.sqrt(0.25 + 2.0 * abs(error) / (a_dt * dt_s)))
    wanted = direction * min(speed_max_tps, stop_speed)
    change = max(-a_dt, min(a_dt, wanted - velocity))
    velocity += change
    step = velocity * dt_s
    if abs(step) >= abs(error):
        return target, 0.0
    return position + step, velocity


def summary(values: Sequence[float]) -> Dict[str, float]:
    if not values:
        return {}
    ordered = sorted(values)
    return {"n": len(ordered), "min": ordered[0], "median": ordered[len(ordered) // 2],
            "max": ordered[-1]}


SPEECH_FILE = "speech.wav"
SPEECH_TAIL_S = 2.5              # after the speech: smile ramp-out and rest


def main(argv: Optional[List[str]] = None, audio_player_factory=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    fs.add_session_arguments(parser)
    # Jaw current cap 650 mA (operator, 2026-11-10: "near 650 mA for short
    # amounts of time"). The board profile kept 750 from a 2026-10-23
    # session, and run 202232 read 703 mA once; this caps the servo itself.
    parser.set_defaults(line_speed=1000.0, jaw_speed=4000.0, jaw_current=JAW_CURRENT_MA,
                        jaw_open=JAW_OPEN_GOAL_TICK, jaw_release=list(JAW_RELEASE_DEFAULT),
                        retry_enable_bus_fault=True)
    parser.add_argument("--seconds", type=float,
                        help="length of the show (10-120 s; default 40, or just long enough for --say)")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--smile", type=float, default=0.6,
                        help="fake smile up activation (0-1); below 0: the corners are held DOWN "
                             "by that much instead (down drives, -1 to 0), to test the jaw against it")
    parser.add_argument("--smile-diag", type=float, default=0.5, help="fake smile diag activation (0-1)")
    parser.add_argument("--no-talk", action="store_true", help="keep the jaw closed")
    parser.add_argument("--eye-speed", type=float, default=EYE_SPEED_TPS,
                        help=f"eyes 0/1/2, ticks/s (at most {fs.SPEED_MAX_TPS})")
    parser.add_argument("--lid-speed", type=float, default=LID_SPEED_TPS,
                        help=f"lids 5/6, ticks/s (at most {fs.SPEED_MAX_TPS})")
    parser.add_argument("--lid-open-speed", type=float, default=LID_OPEN_SPEED_TPS,
                        help=f"lids reopening, ticks/s (default {LID_OPEN_SPEED_TPS:g})")
    parser.add_argument("--blink-depth", action="append", default=[], metavar="[LID=]FRACTION",
                        help="a blink closes the lid to this fraction of its closed mark, 0.3-1 "
                             "(repeat; LID 5 or 6; default " + " ".join(
                                 f"{k}={v}" for k, v in BLINK_DEPTH.items()) + ")")
    parser.add_argument("--validate", type=int, metavar="N",
                        help="N held, stratified poses over the whole action space (no blinks); "
                             "then: face_workspace.py --report RUN")
    parser.add_argument("--explore", action="store_true",
                        help="random actions over the whole ML action space (face_workspace.project): "
                             "both sides incl. down, jaw (budget), brows, oculi, lips, gaze, lids")
    parser.add_argument("--lid-ladder", action="store_true",
                        help="only step the lids to 0.3, 0.4 ... 1.0 of the closed mark and hold "
                             "(stills): to see where the eye is covered")
    parser.add_argument("--say", metavar="TEXT",
                        help="speak this text (eSpeak NG): the jaw and lip corners follow its "
                             "visemes, gaze and blinks its phrases; the audio plays (aplay) and, "
                             "with a camera, is added to the video (video_with_audio.webm)")
    parser.add_argument("--voice", default="en-us", help="eSpeak NG voice (default en-us)")
    parser.add_argument("--wpm", type=int, default=150, help="speech rate, words per minute")
    parser.add_argument("--pitch", type=int, default=50, help="eSpeak pitch 0-100")
    parser.add_argument("--speech-at", type=float,
                        help="script time the speech starts (default: once the smile is up)")
    parser.add_argument("--audio-advance-ms", type=float, default=0.0,
                        help="start the audio this much earlier than the mouth plan (-500 to 500; "
                             "the audio-vs-motion delay is not measured yet)")
    parser.add_argument("--no-audio", action="store_true", help="do not play the speech audio")
    parser.add_argument("--lips", action="store_true",
                        help="also press the lower (3/4) and upper (13/18) lips for p/b/m")
    parser.add_argument("--eye-accel", type=float, default=EYE_ACCEL_TPS2, help="ticks/s^2")
    parser.add_argument("--lid-accel", type=float, default=LID_ACCEL_TPS2, help="ticks/s^2")
    a = parser.parse_args(argv)
    speech = None
    if a.say is not None:
        if a.validate is not None or a.explore or a.lid_ladder or a.no_talk:
            print("--say does not go with --validate, --explore, --lid-ladder or --no-talk."); return 2
        if not -500.0 <= a.audio_advance_ms <= 500.0:
            print("--audio-advance-ms must be -500 to 500."); return 2
        try:
            speech = speech_face.synthesize(a.say, a.voice, a.wpm, a.pitch)
        except (ValueError, RuntimeError, OSError) as error:
            print(f"Speech: {error}"); return 2
        defaults = NaturalSettings()
        speech_at = (a.speech_at if a.speech_at is not None
                     else defaults.smile_in_s + defaults.ramp_s + 0.5)
        if speech_at < 1.0:
            print("--speech-at must be at least 1 s."); return 2
        needed = speech_at + speech.duration_s + defaults.ramp_s + SPEECH_TAIL_S
        if a.seconds is None:
            a.seconds = max(10.0, needed)
        elif a.seconds < needed:
            print(f"--seconds must be at least {needed:.1f} for this text."); return 2
    if a.seconds is None:
        a.seconds = 40.0
    if not 10.0 <= a.seconds <= 120.0:
        print("--seconds must be 10-120."); return 2
    if not (-1.0 <= a.smile <= 1.0 and 0.0 <= a.smile_diag <= 1.0):
        print("--smile must be -1 to 1 and --smile-diag 0-1."); return 2
    if not (0 < a.eye_speed <= fs.SPEED_MAX_TPS and 0 < a.lid_speed <= fs.SPEED_MAX_TPS
            and (a.lid_open_speed is None or 0 < a.lid_open_speed <= fs.SPEED_MAX_TPS)):
        print(f"--eye-speed, --lid-speed and --lid-open-speed must be 1 to {fs.SPEED_MAX_TPS}."); return 2
    depth = dict(BLINK_DEPTH)
    try:
        for item in a.blink_depth:
            lid, _, value = item.rpartition("=")
            lids = [int(lid)] if lid else list(LIDS)
            if any(i not in LIDS for i in lids) or not 0.3 <= float(value) <= 1.0:
                raise ValueError(item)
            for i in lids:
                depth[i] = float(value)
    except ValueError:
        print("--blink-depth: [5=|6=]FRACTION, 0.3-1."); return 2
    ladder = (0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0) if a.lid_ladder else ()
    if a.preview and a.jaw_open == JAW_OPEN_GOAL_TICK:
        a.jaw_open = None            # the simulated neutrals carry their own jaw slice
    settings = NaturalSettings(seconds=a.seconds, seed=a.seed, smile_up=0.0 if ladder else a.smile,
                               smile_diag=a.smile_diag, talk=not (a.no_talk or ladder),
                               blink_depth=depth, ladder=ladder)
    if ladder:
        settings.seconds = 0.5 + len(ladder) * settings.ladder_hold_s + 1.0
    if a.validate is not None:
        if not 5 <= a.validate <= 80:
            print("--validate must be 5-80 poses."); return 2
        plan = ValidatePlan.make_validate(a.validate, a.seed)
        settings = plan.settings
    elif a.explore:
        plan = ExplorePlan.make(settings)
    elif speech is not None:
        track = speech_face.SpeechTrack.make(speech, speech_at, seed=a.seed)
        plan = SpeechPlan.make_speech(settings, track, lips=a.lips)
    else:
        plan = NaturalPlan.make(settings)
    say = lambda text: print(text, flush=True)  # noqa: E731
    records = a.records / "preview" if a.preview else a.records
    device_for_marks = None
    try:
        if a.preview:
            device_for_marks = fs.face_preview_controller()
            marks = eye_tour.preview_marks(device_for_marks)
        else:
            marks = tuning_core.load_earlier_marks(records, since=tuning_core.HEAD_MARKS_SINCE)[0]
        limits = eye_tour.tour_limits(marks)
        if not (1000.0 <= a.eye_accel <= 200000.0 and 1000.0 <= a.lid_accel <= 200000.0):
            raise ValueError("--eye-accel and --lid-accel must be 1000-200000 ticks/s^2")
        extra = EyeLids(plan, limits, a.eye_speed, a.lid_speed, a.eye_accel, a.lid_accel,
                        a.lid_open_speed)
    except (eye_tour.SetupError, ValueError) as error:
        print(error); return 2
    try:
        setup = fs.open_session(a, "face", "natural", say, extra=extra)
    except fs.SetupError as error:
        print(error); return 2
    session = setup.session
    if isinstance(plan, ValidatePlan):
        session.log["validate"] = {"poses": plan.poses,
                                   "actions": [w[1] for w in plan.waypoints[1:-1]]}
    session.log["natural"] = {"settings": vars(settings), "syllables": plan.syllables,
                              "phrases": plan.phrases, "saccades": plan.saccades,
                              "blinks": plan.blinks,
                              "eye_limits": {str(i): limits[i] for i in EXTRA_SERVOS},
                              "eye_speed_tps": a.eye_speed, "lid_speed_tps": a.lid_speed}
    recorder = setup.recorder
    hooks = []
    if recorder is not None:
        stills = setup.run / records_layout.STILLS
        recorder.still(stills / "00_before_enable.jpg")
        marks_t = plan.marks()
        taken = [0]

        def take_stills(t_s: float) -> None:
            while marks_t and t_s >= marks_t[0][0]:
                _t, label = marks_t.pop(0)
                taken[0] += 1
                recorder.still(stills / f"{taken[0]:02d}_{label}.jpg")
        hooks.append(take_stills)
    player = None
    if speech is not None:
        wav = speech.write_wav(setup.run / SPEECH_FILE)
        audio_at = max(0.0, speech_at - a.audio_advance_ms / 1000.0)
        session.log["natural"]["speech"] = {
            **speech_face.summary(speech), "voice": a.voice, "wpm": a.wpm, "pitch": a.pitch,
            "wav": str(wav), "start_s": speech_at, "audio_at_s": audio_at,
            "audio_advance_ms": a.audio_advance_ms, "lips": a.lips, "played": not a.no_audio,
            "phrases": plan.track.phrases,
            "phones": [(round(p.start_s, 3), round(p.end_s, 3), p.ipa) for p in speech.phones],
            "visemes": {"anticipate_s": speech_face.ANTICIPATE_S, "blend_s": speech_face.BLEND_S,
                        "loudness_mix": speech_face.LOUDNESS_MIX}}
        if not a.no_audio:
            player = (audio_player_factory or fs.AudioPlayer)(wav)

            def start_audio(t_s: float) -> None:
                if player.process is None and t_s >= audio_at:
                    player.start()
                    session.log["natural"]["speech"]["started_utc"] = player.started_utc
                    session.log["natural"]["speech"]["started_script_s"] = t_s
            hooks.append(start_audio)
    if hooks:
        session.on_update = lambda t_s: [hook(t_s) for hook in hooks]
    try:
        try:
            session.start()
        except RuntimeError as error:
            session.finish(f"not started: {error}")
            return 2
        say(f"Enabled; natural face for {settings.seconds:.0f} s: {len(plan.blinks)} blinks, "
            f"{len(plan.saccades)} saccades, {len(plan.syllables)} syllables.")
        # The jaw goes through the ML workspace's jaw budget (face_workspace.py):
        # pulses above the static limit only, with a duty limit.
        budget = face_workspace.JawBudget(face_workspace.WorkspaceSpec())
        limited = [0]

        def script(t: float):
            wanted = plan.face(t)
            if wanted is None:
                return None
            actions, j = wanted
            # Every action goes through the ML workspace layer (clipping,
            # known keys, jaw budget), as a policy's would.
            projected = face_workspace.project({**actions, "jaw": j}, budget, t)
            allowed = projected.pop("jaw")
            actions = projected
            if allowed < j - 1e-9:
                limited[0] += 1
            return actions, allowed
        reason = session.run(script)
        session.log["natural"]["jaw_budget_limited_updates"] = limited[0]
        return 0 if reason == "finished" else 1
    finally:
        session.log["natural"]["blink_log"] = extra.blink_log
        session.log["natural"]["saccade_log"] = extra.saccade_log
        closes = [b["closed_s"] for b in extra.blink_log]
        totals = [b["total_s"] for b in extra.blink_log if "total_s" in b]
        reached = [s["reached_s"] for s in extra.saccade_log]
        session.log["natural"]["blink_close_s"] = summary(closes)
        session.log["natural"]["blink_total_s"] = summary(totals)
        session.log["natural"]["saccade_s"] = summary(reached)
        session.log["natural"]["eye_lid_peak_ma"] = {str(i): v for i, v in sorted(extra.peak_ma.items())}
        session.log["natural"]["accel_tps2"] = {"eye": a.eye_accel, "lid": a.lid_accel}
        session.log["natural"]["lid_open_speed_tps"] = extra.lid_open_speed_tps
        session.log["natural"]["blink_depth"] = settings.blink_depth
        say(f"Blinks: close {summary(closes)}, close+open {summary(totals)}; "
            f"saccades reached in {summary(reached)}; eye/lid peak mA {extra.peak_ma}; max temperature "
            f"{max(session.max_temperature_c.values(), default=0)} C.")
        if player is not None:
            player.stop()
        fs.close_session(setup)
        started = (session.log.get("natural", {}).get("speech") or {}).get("started_utc")
        if started and recorder is not None and recorder.frames:
            try:
                _w, _h, fps = fs.parse_camera_mode(a.camera_mode)
                info = speech_face.mux_run(setup.run, setup.run / SPEECH_FILE, started, fps,
                                           video_name=Path(recorder.video_path).name)
                say(f"Video with audio: {info['video']} (audio at {info['offset_s']} s, "
                    f"camera {info['measured_fps']} fps)")
                session.log["natural"]["speech"]["mux"] = info
                tuning_core.write_json_atomically(setup.run / records_layout.RECORD, session.log)
            except (ValueError, RuntimeError, OSError) as error:
                say(f"No video with audio: {error}")


if __name__ == "__main__":
    sys.exit(main())
