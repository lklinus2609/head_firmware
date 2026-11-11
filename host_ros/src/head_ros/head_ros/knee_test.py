"""Find where each modiolus line just becomes tight at the skin's rest pose.

The silicone puts each action point (modiolus) at its own rest pose when no
line pulls. So, per side, the test pays all six lines out by --slack ticks
(both points at rest), then for each line of that side, one at a time with
the other two still slack: takes it in slowly (--step ticks, each once the
horn has caught up), learns the servo's no-load current from the first
BASELINE_TICKS of travel, and stops at --stop-ma or --max-travel. The knee is
where the load first stays above the baseline (find_knee). The line is then
paid back out to where it started, so the point is at rest again for the
next line.

The three knees of one point should describe the same rest pose. The check:
all three lines are taken in together, from slack, to their knee ticks and
held for HOLD_S; each then reads its load against its own baseline. A line
far over its baseline means the knees do not agree (or friction holds it).

With --camera realsense the camera finds the knee instead (operator,
2026-10-21: the current only shows a line once it already pulls the corner
up, so a current-based neutral is not emotionless). The slack picture of each
mouth corner is the reference; while a line comes in, every new frame of that
corner is compared with it (optical flow, 90th percentile, CornerWatch). The
camera knee is where the corner first moves more than CAM_KNEE_PX for
CAM_KNEE_FRAMES frames in a row (camera_knee), and the take-in stops soon
after, at CAM_STOP_PX. The check then compares the corner with all three
lines at their camera knees against the slack picture. When every line has a
camera knee, neutrals.json is written for head-face (kind
"camera_knee", used without a neutral offset). The knees are found with the
jaw closed only (holding it open on slack lines for a minute would take about
300 mA, near where it ran hot), so the open-jaw neutrals are a copy.

The jaw holds where it is. Same session and stops as head-ranges (panel
closed; Ctrl-C, SIGTERM or the stop file turn torque off, as do back-off, the
jaw guard, a board fault, or --abort-ma on any line). Torque goes off at the
end. Writes record.json in its run folder (runs/<date>/<time>_knee/); with
--camera realsense also video.webm and stills/.

    scripts/head-knee --limit 650 --camera realsense
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import statistics
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

try:
    from . import headctl, records_layout, tuning_core, workspace_guard as wg
    from .sweep_runner import DEFAULT_RECORDS, EXIT_FINISHED, EXIT_NOT_STARTED, \
        EXIT_TORQUE_OFF, Runner, add_jaw_arguments, session_limits
    from .jaw_probe import Probe
    from .range_finder import SESSION_NEUTRAL_FORMAT, SESSION_NEUTRAL_SUFFIX
except ImportError:  # run as a plain script from this directory
    import headctl  # type: ignore[no-redef]
    import records_layout  # type: ignore[no-redef]
    import tuning_core  # type: ignore[no-redef]
    import workspace_guard as wg  # type: ignore[no-redef]
    from sweep_runner import DEFAULT_RECORDS, EXIT_FINISHED, EXIT_NOT_STARTED, \
        EXIT_TORQUE_OFF, Runner, add_jaw_arguments, session_limits  # type: ignore[no-redef]
    from jaw_probe import Probe  # type: ignore[no-redef]
    from range_finder import SESSION_NEUTRAL_FORMAT, SESSION_NEUTRAL_SUFFIX  # type: ignore[no-redef]

KNEE_FORMAT = "head-knee-test-1"
# Order on each side: down first, then diag, then up.
ROLE_ORDER = ("down", "diag", "up")
# Knee rule (chosen 2026-10-21, to be judged on the first curves): the
# baseline is the median load over the first BASELINE_TICKS of travel; the
# knee is the first of KNEE_HOLD_SAMPLES samples in a row above baseline +
# max(KNEE_MIN_RISE_MA, KNEE_NOISE_FACTOR x noise), noise = 1.4826 x MAD.
BASELINE_TICKS = 60
KNEE_MIN_RISE_MA = 8
KNEE_NOISE_FACTOR = 4.0
KNEE_HOLD_SAMPLES = 5
# The baseline window is not slack when its load already rises by this much
# from its first to its last third.
BASELINE_RISE_MA = 8
STOP_SAMPLES = 3
HOLD_S = 1.0                  # about 100 samples at 100 Hz (2 s until 2026-10-21)
TAKE_IN_TIMEOUT_S = 60.0
# A take-in whose goal has not moved for this long stops (line 16 crawled for
# the whole 60 s timeout on 2026-10-21).
STALL_S = 3.0
# Camera knee (chosen 2026-10-21 from the knee run at 20:39Z: a corner whose
# lines did not move read about 0.2-0.3 px, the forehead 0.03 px; the first
# clear motion of a good line, 19, came at about 124 ticks).
CAM_KNEE_PX = 0.5
CAM_KNEE_FRAMES = 3
CAM_STOP_PX = 1.5
CAM_STOP_FRAMES = 2
# With the camera on, a take-in stops when the corner has not moved after
# the slack plus this many ticks: on 2026-10-22 the right magnet was off and
# lines 11 and 14 pulled the bare action point up 1500 ticks.
CAM_NO_MOTION_TICKS = 800
# Mouth-corner boxes [y0, y1, x0, x1] in the reference picture (1280 x 720,
# camera as aimed on 2026-10-21; the face's right corner is on the picture's
# left). Each run finds the face again (locate_corners: the eyes-and-nose
# band FACE_TEMPLATE matched at a few scales) and moves the boxes with it: on
# 2026-10-22 the face had moved about 140 px and the fixed boxes were on the
# background. The slack still is saved with the boxes drawn on it.
FACE_REFERENCE = (Path(__file__).resolve().parents[4] / "tuning_runs" / records_layout.CONFIG /
                  "face_reference.jpg")        # new head, torque off, 2026-11-09
# New head 2026-11-09 (old head's: right (430, 600, 440, 580), left (430, 600,
# 600, 740), template (270, 470, 440, 770); face_reference_20261021_old_head.jpg).
# The new face is smaller in the picture: mouth line y ~557, corners x ~705/835.
CORNER_REGIONS = {"right": (515, 605, 655, 765), "left": (515, 605, 775, 880)}
FACE_TEMPLATE = (330, 500, 620, 910)
FACE_SCALES = tuple(round(0.80 + 0.02 * k, 2) for k in range(21))      # 0.80 .. 1.20
# Normalised correlation the face must reach (the 2026-10-22 picture: 0.71).
FACE_MATCH_MIN = 0.6
# A corner is still once it moves less than STILL_PX between frames for
# STILL_FRAMES frames in a row (the silicone takes time to settle).
STILL_PX = 0.3
STILL_FRAMES = 5
STILL_TIMEOUT_S = 3.0
# Finding slack by load (2026-10-22: a fixed 600-tick pay-out wound 11, 14,
# 9 and 16 back onto their horns the other way, -20 to -57 mA holding, and
# pulled the corner up). Each line moves SLACK_STEP ticks at a time and its
# holding load (pull direction, mean over SLACK_HOLD_S) decides: over
# +SLACK_BAND_MA still tight, pay out; under -SLACK_BAND_MA wound the other
# way, take in; inside the band it is slack.
SLACK_STEP = 50
SLACK_BAND_MA = 10
SLACK_HOLD_S = 0.3
SLACK_MAX_TICKS = 1500
# The open-jaw tick written into the session file (the session's jaw open
# limit). New head 2026-11-09: the operator's panel open mark for the jaw
# (2026-11-09_152443_tuning.json, end_1 3939); 3900 on the old head.
JAW_OPEN_TICK = 3939


def find_knee(samples: Sequence[Tuple[int, int]], baseline_ticks: int = BASELINE_TICKS,
              min_rise_ma: float = KNEE_MIN_RISE_MA, noise_factor: float = KNEE_NOISE_FACTOR,
              hold: int = KNEE_HOLD_SAMPLES) -> dict:
    """samples: (travel ticks from the start, load mA in the pull direction).
    Returns the baseline, noise, threshold, whether the baseline window was
    slack, and the knee's travel (None when the load never rose)."""
    window = [load for travel, load in samples if travel <= baseline_ticks]
    if len(window) < 6:
        return {"knee_travel": None, "reason": "too few samples before the baseline ends"}
    baseline = statistics.median(window)
    noise = 1.4826 * statistics.median(abs(load - baseline) for load in window)
    threshold = baseline + max(min_rise_ma, noise_factor * noise)
    third = max(1, len(window) // 3)
    rise = statistics.mean(window[-third:]) - statistics.mean(window[:third])
    result = {"baseline_ma": round(baseline, 1), "noise_ma": round(noise, 1),
              "threshold_ma": round(threshold, 1), "baseline_slack": rise < BASELINE_RISE_MA,
              "knee_travel": None, "reason": "the load never stayed over the threshold"}
    run = 0
    for number, (travel, load) in enumerate(samples):
        if travel <= baseline_ticks:
            continue
        run = run + 1 if load > threshold else 0
        if run >= hold:
            first = samples[number - hold + 1]
            result.update(knee_travel=first[0], reason="knee")
            break
    return result


def camera_knee(samples: Sequence[Tuple[float, float]], threshold_px: float = CAM_KNEE_PX,
                frames: int = CAM_KNEE_FRAMES) -> Optional[float]:
    """samples: (travel ticks, corner motion px) per camera frame. The travel
    of the first of `frames` frames in a row over threshold_px, or None."""
    run = 0
    for number, (_travel, px) in enumerate(samples):
        run = run + 1 if px > threshold_px else 0
        if run >= frames:
            return samples[number - frames + 1][0]
    # Stick-slip: the corner can jump straight past the stop level, and the
    # take-in then stops after CAM_STOP_FRAMES frames, before `frames` frames
    # over threshold_px exist (line 19 on the new head, 2026-11-09: 0.32 px at
    # 110 ticks, 1.70 at 117, 2.07 at 124). The first frame of that jump is
    # the knee.
    run = 0
    for number, (_travel, px) in enumerate(samples):
        run = run + 1 if px > CAM_STOP_PX else 0
        if run >= CAM_STOP_FRAMES:
            return samples[number - CAM_STOP_FRAMES + 1][0]
    return None


def corner_motion(reference, image) -> float:
    """90th percentile of the optical flow (px) from reference to image,
    both grey crops of one corner box."""
    import cv2  # optional dependency (bench venv)
    import numpy

    flow = cv2.calcOpticalFlowFarneback(reference, image, None, 0.5, 3, 21, 3, 5, 1.1, 0)
    return float(numpy.percentile(numpy.hypot(flow[..., 0], flow[..., 1]), 90))


def locate_corners(image, reference=None) -> Tuple[Dict[str, Tuple[int, int, int, int]], float, float]:
    """(corner boxes in this picture, match score, scale): the reference's
    FACE_TEMPLATE found in the RGB image at FACE_SCALES, CORNER_REGIONS moved
    and scaled with it."""
    import cv2

    if reference is None:
        reference = cv2.imread(str(FACE_REFERENCE), cv2.IMREAD_GRAYSCALE)
        if reference is None:
            raise RuntimeError(f"no face reference picture at {FACE_REFERENCE}")
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) if image.ndim == 3 else image
    y0, y1, x0, x1 = FACE_TEMPLATE
    template = reference[y0:y1, x0:x1]
    best = None
    for scale in FACE_SCALES:
        scaled = cv2.resize(template, None, fx=scale, fy=scale)
        if scaled.shape[0] >= gray.shape[0] or scaled.shape[1] >= gray.shape[1]:
            continue
        _low, score, _at, (lx, ly) = cv2.minMaxLoc(cv2.matchTemplate(gray, scaled, cv2.TM_CCOEFF_NORMED))
        if best is None or score > best[0]:
            best = (score, scale, lx, ly)
    score, scale, lx, ly = best
    boxes = {side: (round(ly + (a - y0) * scale), round(ly + (b - y0) * scale),
                    round(lx + (c - x0) * scale), round(lx + (d - x0) * scale))
             for side, (a, b, c, d) in CORNER_REGIONS.items()}
    return boxes, float(score), float(scale)


class CornerWatch:
    """One mouth corner on the recorder's frames: reset() takes the newest
    frame as the reference, poll() gives (frame time, motion px) once per new
    frame, else None."""

    def __init__(self, recorder, region: Tuple[int, int, int, int]) -> None:
        self.recorder = recorder
        self.region = region
        self.reference = None
        self._seen_s = float("-inf")

    def _crop(self, image):
        import cv2

        y0, y1, x0, x1 = self.region
        return cv2.cvtColor(image[y0:y1, x0:x1], cv2.COLOR_RGB2GRAY)

    def reset(self, timeout_s: float = 2.0) -> None:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            frame = self.recorder.latest()
            if frame is not None and frame[0] > self._seen_s:
                self._seen_s = frame[0]
                self.reference = self._crop(frame[1])
                return
            time.sleep(0.01)
        raise RuntimeError("no camera frame for the corner reference")

    def wait_still(self, pump) -> Optional[str]:
        """Wait (pumping the board) until the corner is still; a stop reason
        from pump, or None (also when it times out: the knee then shows it)."""
        previous, still = None, 0
        end = time.monotonic() + STILL_TIMEOUT_S
        while time.monotonic() < end and still < STILL_FRAMES:
            reason = pump()
            if reason is not None:
                return reason
            frame = self.recorder.latest()
            if frame is None or frame[0] <= self._seen_s:
                continue
            self._seen_s = frame[0]
            crop = self._crop(frame[1])
            if previous is not None:
                still = still + 1 if corner_motion(previous, crop) < STILL_PX else 0
            previous = crop
        return None

    def poll(self) -> Optional[Tuple[float, float]]:
        frame = self.recorder.latest()
        if frame is None or frame[0] <= self._seen_s or self.reference is None:
            return None
        self._seen_s = frame[0]
        return frame[0], corner_motion(self.reference, self._crop(frame[1]))


class KneeTest:
    def __init__(self, probe: Probe, step: int, stop_ma: int, max_travel: int,
                 say=print, slack: int = 0, no_motion_ticks: int = CAM_NO_MOTION_TICKS) -> None:
        self.probe, self.link = probe, probe.link
        self.step, self.stop_ma, self.max_travel = step, stop_ma, max_travel
        self.say = say
        # The camera gives up after slack + CAM_NO_MOTION_TICKS without motion.
        self.no_motion_ticks = slack + no_motion_ticks
        # Servo -> its own take-in stop (mA), for a line with more friction
        # (line 9: about 40-45 mA while moving, 2026-10-22).
        self.stop_ma_by_servo: Dict[int, int] = {}

    def take_in(self, index: int, watch: Optional[CornerWatch] = None,
                max_travel: Optional[int] = None, no_motion_ticks: Optional[int] = None) -> dict:
        """Take one line in slowly from where it is; the curve and its knee
        (and, with a watch, the camera curve and camera knee). max_travel and
        no_motion_ticks override the test's own for this line (the down
        lines, which can have much more slack)."""
        max_travel = self.max_travel if max_travel is None else max_travel
        no_motion_ticks = self.no_motion_ticks if no_motion_ticks is None else no_motion_ticks
        pull = wg.PULL[index]
        start = self.link.state.servos[index]
        start_present, start_goal = start.present_tick, start.goal_tick
        curve: List[List[int]] = []
        camera: List[List[float]] = []          # [frame time, travel, motion px]
        seen = float("-inf")
        over = 0
        moved = 0
        deadline = time.monotonic() + TAKE_IN_TIMEOUT_S
        stopped_by = None
        last_goal, last_move_s = start_goal, time.monotonic()
        while stopped_by is None:
            reason = self.probe.pump()
            if reason is not None:
                return {"stopped": reason, "curve": curve}
            if time.monotonic() > deadline:
                stopped_by = f"timeout {TAKE_IN_TIMEOUT_S:.0f} s"
                break
            state = self.link.state
            servo = state.servos[index]
            if servo.goal_tick != last_goal:
                last_goal, last_move_s = servo.goal_tick, time.monotonic()
            elif time.monotonic() - last_move_s > STALL_S:
                stopped_by = f"no progress for {STALL_S:g} s"
                break
            if state.received_monotonic_s > seen:
                seen = state.received_monotonic_s
                load = servo.current_ma * pull
                curve.append([round(seen, 3), servo.goal_tick, servo.present_tick, servo.current_ma])
                stop_ma = self.stop_ma_by_servo.get(index, self.stop_ma)
                over = over + 1 if load >= stop_ma else 0
                if over >= STOP_SAMPLES:
                    stopped_by = f"{stop_ma} mA"
                elif pull * (servo.goal_tick - start_goal) >= max_travel:
                    stopped_by = f"{max_travel} ticks"
            sample = watch.poll() if watch is not None else None
            if sample is not None:
                frame_s, px = sample
                camera.append([round(frame_s, 3), travel_at(curve, frame_s, start_present, pull),
                               round(px, 3)])
                moved = moved + 1 if px > CAM_STOP_PX else 0
                if stopped_by is None and moved >= CAM_STOP_FRAMES:
                    stopped_by = f"camera {CAM_STOP_PX:g} px"
                elif (stopped_by is None and camera[-1][1] >= no_motion_ticks and
                      camera_knee([(t, x) for _s, t, x in camera]) is None):
                    stopped_by = (f"no corner motion in {no_motion_ticks} ticks "
                                  "(magnet off?)")
            # Caught up: with Position I = 0 the horn trails its goal by its
            # load / LAG_MA_PER_TICK, so only the trail beyond that counts.
            trail = pull * (servo.goal_tick - servo.present_tick)
            expected = max(0, servo.current_ma * pull) / tuning_core.LAG_MA_PER_TICK
            if stopped_by is None and trail - expected <= self.step and self.probe.may_step(index):
                self.link.request_step(index, self.step * pull)
        samples = [(pull * (present - start_present), current * pull)
                   for _t, _goal, present, current in curve]
        knee = find_knee(samples)
        if knee["knee_travel"] is not None:
            knee["knee_tick"] = start_present + pull * knee["knee_travel"]
        result = {"start_present": start_present, "start_goal": start_goal,
                  "stopped_by": stopped_by, "knee": knee, "curve": curve}
        if watch is not None:
            travel = camera_knee([(t, px) for _s, t, px in camera])
            result["camera"] = camera
            result["camera_knee"] = {"travel": None if travel is None else round(travel),
                                     "tick": None if travel is None else
                                     start_present + pull * round(travel),
                                     "threshold_px": CAM_KNEE_PX, "frames": CAM_KNEE_FRAMES}
        return result

    def find_slack(self, indexes: Sequence[int], margin: int) -> Tuple[Optional[str], dict]:
        """Bring each line, one at a time, to where its holding load is inside
        +-SLACK_BAND_MA. One that came from the tight side then pays out
        `margin` more if it stays inside (else that is taken back); one that
        was wound the other way goes SLACK_STEP further in. Returns (stop
        reason, per-line report)."""
        report: dict = {}
        for index in indexes:
            pull = wg.PULL[index]
            start = self.link.state.servos[index].goal_tick
            line = {"start_goal": start, "loads": [], "found": False}
            report[str(index)] = line
            while True:
                reason, loads = self.hold_loads([index], SLACK_HOLD_S)
                if reason is not None:
                    return reason, report
                load = loads[index]
                goal = self.link.state.servos[index].goal_tick
                line["loads"].append([goal, load])
                if abs(load) <= SLACK_BAND_MA:
                    line["found"] = True
                    break
                if abs(goal - start) >= SLACK_MAX_TICKS:
                    break
                if load < 0:
                    line["wound_the_other_way"] = True
                out = load > 0                          # tight: pay out; reversed: take in
                reason = self.probe.lines_to({index: goal + (-pull if out else pull) * SLACK_STEP},
                                             paying_out=out)
                if reason is not None:
                    return reason, report
            if line["found"] and line.get("wound_the_other_way"):
                # It entered the band from the reversed side, so it may still
                # be wound back a little: one step further in clears it.
                goal = self.link.state.servos[index].goal_tick
                reason = self.probe.lines_to({index: goal + pull * SLACK_STEP}, paying_out=False)
                if reason is not None:
                    return reason, report
            elif line["found"] and margin > 0:
                slack_goal = self.link.state.servos[index].goal_tick
                reason = self.probe.lines_to({index: slack_goal - pull * margin}, paying_out=True)
                if reason is not None:
                    return reason, report
                reason, loads = self.hold_loads([index], SLACK_HOLD_S)
                if reason is not None:
                    return reason, report
                line["loads"].append([self.link.state.servos[index].goal_tick, loads[index]])
                if loads[index] < -SLACK_BAND_MA:       # the margin wound it the other way
                    reason = self.probe.lines_to({index: slack_goal}, paying_out=False)
                    if reason is not None:
                        return reason, report
                    line["margin_taken_back"] = True
            line["end_goal"] = self.link.state.servos[index].goal_tick
            self.say(f"  {index}: " + (f"slack at {line['end_goal']} ({line['end_goal'] - start:+d})"
                                        if line["found"] else
                                        f"NO slack found within {SLACK_MAX_TICKS} ticks")
                     + (" (was wound the other way)" if line.get("wound_the_other_way") else ""))
        return None, report

    def clear_for_down(self, indexes: Sequence[int], clearance: int) -> Tuple[Optional[str], dict]:
        """Pay the up and diag lines of a side out up to `clearance` ticks
        more, SLACK_STEP at a time, so the down line can pull the corner down
        (operator, 2026-11-09: they block it right away when only just slack).
        A line whose load turns the other way (wound back onto its horn:
        under -SLACK_BAND_MA in its pull direction) takes that step back and
        stops there; Probe.lines_to also leaves a line that lags while paying
        out. Returns (stop reason, {"restore": servo -> goal before, "paid_out":
        servo -> ticks})."""
        restore = {i: self.link.state.servos[i].goal_tick for i in indexes}
        paid = {i: 0 for i in indexes}
        for index in indexes:
            pull = wg.PULL[index]
            while paid[index] + SLACK_STEP <= clearance:
                goal = self.link.state.servos[index].goal_tick
                reason = self.probe.lines_to({index: goal - pull * SLACK_STEP}, paying_out=True)
                if reason is not None:
                    return reason, {"restore": restore, "paid_out": paid}
                reason, loads = self.hold_loads([index], SLACK_HOLD_S)
                if reason is not None:
                    return reason, {"restore": restore, "paid_out": paid}
                if loads.get(index, 0.0) < -SLACK_BAND_MA:
                    reason = self.probe.lines_to({index: goal}, paying_out=False)
                    if reason is not None:
                        return reason, {"restore": restore, "paid_out": paid}
                    self.say(f"  {index}: wound the other way after {paid[index] + SLACK_STEP} "
                             f"ticks ({loads[index]:+.0f} mA); kept at {paid[index]}")
                    break
                paid[index] += SLACK_STEP
        return None, {"restore": restore, "paid_out": paid}

    def hold_loads(self, indexes: Sequence[int], hold_s: float = HOLD_S) -> Tuple[Optional[str], Dict[int, float]]:
        """Mean load per line (pull direction) over hold_s."""
        loads: Dict[int, List[int]] = {i: [] for i in indexes}
        end = time.monotonic() + hold_s
        seen = float("-inf")
        while time.monotonic() < end:
            reason = self.probe.pump()
            if reason is not None:
                return reason, {}
            state = self.link.state
            if state.received_monotonic_s > seen:
                seen = state.received_monotonic_s
                for i in indexes:
                    loads[i].append(state.servos[i].current_ma * wg.PULL[i])
        return None, {i: round(statistics.mean(v), 1) for i, v in loads.items() if v}


def travel_at(curve: Sequence[Sequence[float]], time_s: float, start_present: int,
              pull: int) -> float:
    """The line's travel (present tick, pull direction) at time_s, between
    the STATE samples around it."""
    if not curve:
        return 0.0
    before = [row for row in curve if row[0] <= time_s]
    after = [row for row in curve if row[0] > time_s]
    if not before:
        return float(pull * (curve[0][2] - start_present))
    a = before[-1]
    if not after:
        return float(pull * (a[2] - start_present))
    b = after[0]
    u = (time_s - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
    return pull * (a[2] + u * (b[2] - a[2]) - start_present)


def camera_session_neutral(log: dict, session_id: Optional[int], preload_ma: int) -> Optional[dict]:
    """The head-face input from the camera knees (None unless every line of
    every side tested has one)."""
    closed: Dict[str, int] = {}
    for side in log["sides"].values():
        for index, line in side["lines"].items():
            tick = (line.get("camera_knee") or {}).get("tick")
            if tick is None:
                return None
            closed[index] = int(tick)
    if set(closed) != {str(i) for i in wg.PULL}:
        return None
    return {"format": SESSION_NEUTRAL_FORMAT, "made_utc": datetime.now(timezone.utc).isoformat(),
            "source": log.get("source"), "controller_session_id": session_id, "jaw": wg.JAW,
            "neutral_kind": "camera_knee", "open_copied_from_closed": True,
            "preload_ma": preload_ma, "jaw_closed_tick": log["jaw_tick"],
            "jaw_open_tick": JAW_OPEN_TICK, "jaw_open_target": JAW_OPEN_TICK,
            "closed": closed, "open": dict(closed)}


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=headctl.AUTO_PORT)
    parser.add_argument("--records", type=Path, default=DEFAULT_RECORDS)
    parser.add_argument("--preview", action="store_true", help="simulated board")
    parser.add_argument("--limit", type=int, required=True)
    parser.add_argument("--side", action="append", choices=sorted(wg.SIDES),
                        help="repeat; default both")
    parser.add_argument("--slack", type=int, default=50,
                        help="extra pay-out after each line's slack is found by its load (ticks)")
    parser.add_argument("--step", type=int, default=3, help="take-in step (ticks)")
    parser.add_argument("--stop-ma", type=int, default=80)
    parser.add_argument("--max-travel", type=int, default=900,
                        help="take-in stops after this many ticks without --stop-ma")
    parser.add_argument("--abort-ma", type=int, default=280)
    parser.add_argument("--stop-ma-servo", action="append", default=[], metavar="SERVO=MA",
                        help="a line's own take-in stop, below --abort-ma (repeat)")
    parser.add_argument("--camera", choices=("none", "realsense"), default="none")
    parser.add_argument("--no-motion-ticks", type=int, default=CAM_NO_MOTION_TICKS,
                        help="camera: a take-in stops when the corner has not moved after the "
                             "slack plus this many ticks")
    parser.add_argument("--down-clearance", type=int, default=0,
                        help="before each down line: pay that side's up and diag lines out this "
                             "many ticks more (they block the corner when only just slack); "
                             "they come back after the down line")
    parser.add_argument("--down-max-travel", type=int,
                        help="the down lines' own take-in travel limit (default --max-travel)")
    parser.add_argument("--other-sides-from", type=Path,
                        help="an earlier knee run's record.json (same boot) whose camera knees fill in "
                             "the sides not tested now, for the session neutral file")
    add_jaw_arguments(parser)
    a = parser.parse_args(argv)
    one_turn = tuning_core.headctl_one_turn_ticks()
    if not (0 <= a.slack <= 300 and 1 <= a.step <= 10 and 0 < a.stop_ma < a.abort_ma
            and a.slack < a.max_travel <= 2000 and 0 <= a.down_clearance <= 2000
            and (a.down_max_travel is None or a.slack < a.down_max_travel <= 2000)):
        print("Need 0 <= slack <= 300, step 1-10, stop-ma < abort-ma, "
              "slack < max-travel <= 2000, down-clearance 0-2000, "
              "slack < down-max-travel <= 2000."); return EXIT_NOT_STARTED
    sides = a.side or ["right", "left"]
    records = a.records / "preview" if a.preview else a.records
    stop_file = records / "STOP"
    if stop_file.exists():
        print(f"Remove the stop file first: {stop_file}"); return EXIT_NOT_STARTED
    run = records_layout.run_dir(records, "knee")
    print(f"Run folder: {run}", flush=True)
    out_path = run / records_layout.RECORD
    stills = run / records_layout.STILLS
    recorder = None
    if a.camera == "realsense":
        from face_camera import RealSenseCamera, Recorder  # optional
        recorder = Recorder(RealSenseCamera(), run / records_layout.VIDEO)
    taken = [0]

    def still(name: str) -> None:
        if recorder is not None:
            taken[0] += 1
            recorder.still(stills / f"{taken[0]:02d}_{name}.jpg")

    if a.preview:
        from face_stream import face_preview_controller
        device = face_preview_controller()
    else:
        device = headctl.Device(a.port)
    link = tuning_core.TuningLink(device)
    link.connect()
    info = headctl.decode_info(device.get(headctl.GET_INFO, headctl.INFO))
    branches = headctl.read_servo_branches(device, info["active_servo_mask"])
    caps, guard = session_limits(device, info, a.limit, wg.JAW, a)
    say = lambda text: print(text, flush=True)
    runner = Runner(link, branches, run, a.limit, dict(tuning_core.SWEEP_BOX_DEFAULTS),
                    stop_file, say=say, caps=caps, jaw_guard=guard,
                    jaw_abort_ma=a.jaw_abort_ma)
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda signum, _f: runner.request_stop(signal.Signals(signum).name))
    probe = Probe(runner, wg.JAW, dict(wg.PULL), a.abort_ma, None, stills,
                  a.jaw_abort_ma or 600)
    test = KneeTest(probe, a.step, a.stop_ma, a.max_travel, say, slack=a.slack,
                    no_motion_ticks=a.no_motion_ticks)
    for item in a.stop_ma_servo:
        servo, _, ma = item.partition("=")
        if not (servo.isdigit() and int(servo) in wg.PULL and ma.isdigit() and
                0 < int(ma) < a.abort_ma):
            print(f"bad --stop-ma-servo {item!r} (a modiolus line, under --abort-ma)")
            return EXIT_NOT_STARTED
        test.stop_ma_by_servo[int(servo)] = int(ma)
    log = {"format": KNEE_FORMAT, "started_utc": datetime.now(timezone.utc).isoformat(),
           "limit_ma": a.limit, "slack": a.slack, "step": a.step, "stop_ma": a.stop_ma,
           "max_travel": a.max_travel, "abort_ma": a.abort_ma,
           "rule": {"baseline_ticks": BASELINE_TICKS, "min_rise_ma": KNEE_MIN_RISE_MA,
                    "noise_factor": KNEE_NOISE_FACTOR, "hold_samples": KNEE_HOLD_SAMPLES},
           "video": str(recorder.video_path) if recorder else None,
           "sides": {}, "stopped": None, "source": out_path.name,
           "camera_rule": {"knee_px": CAM_KNEE_PX, "knee_frames": CAM_KNEE_FRAMES,
                           "stop_px": CAM_STOP_PX, "regions": CORNER_REGIONS}}
    log["stop_ma_by_servo"] = {str(k): v for k, v in test.stop_ma_by_servo.items()}
    log["down_clearance"] = a.down_clearance
    log["down_max_travel"] = a.down_max_travel
    code = EXIT_FINISHED
    watches = ({side: CornerWatch(recorder, CORNER_REGIONS[side]) for side in wg.SIDES}
               if recorder is not None else {})

    def stop(reason: str) -> int:
        log["stopped"] = reason
        say(f"  stopped: {reason}")
        return EXIT_TORQUE_OFF

    try:
        if not runner.start():
            return EXIT_NOT_STARTED
        state = link.state
        log["jaw_tick"] = state.servos[wg.JAW].present_tick
        say("Finding each line's slack by its load...")
        reason, log["slack_found"] = test.find_slack(list(wg.PULL), a.slack)
        if reason is not None:
            return stop(reason)
        missing = [i for i, line in log["slack_found"].items() if not line["found"]]
        if missing:
            return stop(f"no slack found for lines {missing}")
        reason = probe.wait(0.5)                      # let the skin settle at rest
        if reason is not None:
            return stop(reason)
        still("all_slack")
        if recorder is not None:
            frame = recorder.latest()
            if frame is None:
                return stop("no camera frame")
            regions, score, scale = locate_corners(frame[1])
            log["camera_regions"] = {"boxes": regions, "match_score": round(score, 3),
                                     "scale": scale}
            save_regions_still(recorder, stills / "00_corner_regions.jpg", regions)
            say(f"Face found (match {score:.2f}, scale {scale:.2f}); corner boxes {regions}")
            if score < FACE_MATCH_MIN:
                return stop(f"the camera does not find the face (match {score:.2f} < "
                            f"{FACE_MATCH_MIN}); aim it at the face and run again")
            for side, watch in watches.items():
                watch.region = regions[side]
        for side in sides:
            lines = wg.SIDES[side]
            side_log: dict = {"lines": {}}
            log["sides"][side] = side_log
            watch = watches.get(side)
            if watch is not None:
                reason = watch.wait_still(probe.pump)
                if reason is not None:
                    return stop(reason)
                watch.reset()
                slack_reference = watch.reference
            for role in ROLE_ORDER:
                index = lines[role]
                say(f"{side} {role} ({index}): take in...")
                if watch is not None:
                    reason = watch.wait_still(probe.pump)
                    if reason is not None:
                        return stop(reason)
                    watch.reset()                     # the corner as it is now, at rest
                clear = None
                if role == "down" and a.down_clearance > 0:
                    say(f"  clearance: paying {lines['diag']} and {lines['up']} out up to "
                        f"{a.down_clearance} ticks...")
                    reason, clear = test.clear_for_down([lines["diag"], lines["up"]],
                                                        a.down_clearance)
                    side_log["down_clearance"] = clear
                    if reason is not None:
                        return stop(reason)
                    say(f"  paid out: {clear['paid_out']}")
                    if watch is not None:
                        reason = watch.wait_still(probe.pump)
                        if reason is not None:
                            return stop(reason)
                        watch.reset()
                if role == "down" and a.down_max_travel is not None:
                    result = test.take_in(index, watch, max_travel=a.down_max_travel,
                                          no_motion_ticks=a.slack + a.down_max_travel - 50)
                else:
                    result = test.take_in(index, watch)
                side_log["lines"][str(index)] = dict(result, role=role)
                tuning_core.write_json_atomically(out_path, log)
                if "stopped" in result:
                    return stop(result["stopped"])
                knee = result["knee"]
                say(f"  baseline {knee.get('baseline_ma')} mA (noise {knee.get('noise_ma')}), "
                    f"knee {knee.get('knee_tick')} = {knee.get('knee_travel')} ticks in "
                    f"({knee['reason']}; {'slack' if knee.get('baseline_slack') else 'NOT slack'} "
                    f"at start); stopped by {result['stopped_by']}")
                if "camera_knee" in result:
                    cam = result["camera_knee"]
                    say(f"  camera knee {cam['tick']} = {cam['travel']} ticks in"
                        if cam["tick"] is not None else "  no camera knee (the corner did not move)")
                still(f"{side}_{role}_pulled")
                reason = probe.lines_to({index: result["start_goal"]}, True)
                if reason is not None:
                    return stop(reason)
                if clear is not None:                 # up and diag back to just slack
                    reason = probe.lines_to(clear["restore"], False)
                    if reason is not None:
                        return stop(reason)
            if watch is not None:
                knees = {lines[r]: (side_log["lines"][str(lines[r])].get("camera_knee") or {}).get("tick")
                         for r in ROLE_ORDER}
            else:
                knees = {lines[r]: side_log["lines"][str(lines[r])]["knee"].get("knee_tick")
                         for r in ROLE_ORDER}
            if any(tick is None for tick in knees.values()):
                side_log["check"] = {"skipped": "a line has no knee"}
                say(f"  {side}: no co-actuation check (a line has no knee)")
                continue
            say(f"  {side}: all three to their knees together...")
            reason = probe.lines_to(knees, False)
            if reason is not None:
                return stop(reason)
            reason, loads = test.hold_loads(list(knees))
            if reason is not None:
                return stop(reason)
            baselines = {i: side_log["lines"][str(i)]["knee"]["baseline_ma"] for i in knees}
            side_log["check"] = {"knees": knees, "loads_ma": loads,
                                 "over_baseline_ma": {i: round(loads[i] - baselines[i], 1)
                                                      for i in loads}}
            say("  at the knees: " + " ".join(
                f"{i}:{loads[i]:+.0f} mA ({loads[i] - baselines[i]:+.0f} over baseline)"
                for i in knees))
            if watch is not None:
                watch.reference = slack_reference     # compare with the side at rest
                sample = None
                end = time.monotonic() + 1.0
                while sample is None and time.monotonic() < end:
                    sample = watch.poll()
                    time.sleep(0.01)
                if sample is not None:
                    side_log["check"]["corner_px_vs_slack"] = round(sample[1], 2)
                    say(f"  corner vs the slack picture: {sample[1]:.2f} px "
                        f"(knee threshold {CAM_KNEE_PX:g})")
            still(f"{side}_at_knees")
            tuning_core.write_json_atomically(out_path, log)
        if watches and a.other_sides_from is not None:
            earlier = json.loads(a.other_sides_from.read_text())
            for side in wg.SIDES:
                if side not in log["sides"] and side in earlier.get("sides", {}):
                    log["sides"][side] = dict(earlier["sides"][side],
                                              merged_from=a.other_sides_from.name)
                    say(f"  {side} side taken from {a.other_sides_from.name}")
        if watches:
            data = camera_session_neutral(log, getattr(device, "session_id", None),
                                          wg.PRELOAD_MA)
            if data is None:
                say("No session neutral file: a line has no camera knee.")
            else:
                session_path = run / records_layout.NEUTRALS
                tuning_core.write_json_atomically(session_path, data)
                log["session_neutral"] = str(session_path)
                say(f"Session neutrals (camera knees): {session_path}")
        return code
    finally:
        still("end_torque_on")
        runner.disable()
        say(f"Torque off. Knees: {out_path}")
        tuning_core.write_json_atomically(out_path, log)
        runner.close()
        if recorder is not None:
            time.sleep(1.0)
            still("torque_off_1s")
            time.sleep(1.0)
            recorder.close()
            say(f"Video: {recorder.video_path} ({recorder.frames} frames)")
        try:
            device.close()
        except Exception:
            pass


def save_regions_still(recorder, path: Path, regions=None) -> None:
    """The newest frame with the corner boxes drawn, to check the aim."""
    import cv2

    regions = regions or CORNER_REGIONS

    frame = recorder.latest()
    if frame is None:
        return
    image = cv2.cvtColor(frame[1], cv2.COLOR_RGB2BGR)
    for side, (y0, y1, x0, x1) in regions.items():
        cv2.rectangle(image, (x0, y0), (x1, y1), (0, 0, 255), 2)
        cv2.putText(image, side, (x0, y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), image)


if __name__ == "__main__":
    sys.exit(main())
