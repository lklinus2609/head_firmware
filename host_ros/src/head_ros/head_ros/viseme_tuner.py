"""Viseme calibration: make the robot's mouth match photos of a person's mouth.

    scripts/head-viseme targets --photos DIR
    scripts/head-viseme try --viseme M --pose 'jaw=0 lip_right=0.6 lip_left=0.6' [--pose ...]
    scripts/head-viseme search --viseme M [--start POSE] [--max-trials 30] [--save]
    scripts/head-viseme save --viseme M --pose 'jaw=0.2 ...'
    scripts/head-viseme show

targets: DIR holds one photo per viseme, named <viseme>.jpg (or .jpeg/.png),
and rest.jpg (mouth relaxed and closed). MediaPipe finds the landmarks; the
mouth measures (mouth_shape.py) of each photo, as changes from rest.jpg, are
the targets (tuning_runs/config/viseme_targets.json). Crops of each mouth go
to tuning_runs/config/viseme_targets/ for a look.

try / search: one ENABLED session (face_stream: the same neutrals, envelope,
speed limits, jaw guard, live checks and stops as head-face). It first
measures the robot's own rest and a few extreme poses (calibration: the
robot's span per measure), then holds each pose until the targets settle,
saves a still, measures the dots and scores the pose against the viseme.
Every pose gets a side-by-side picture (person | robot) in composites/.
search moves one value at a time (jaw, corner diag, corner up/down, lower
lip, upper lip; both sides the same) and keeps a change when the score drops;
the steps halve when no change helps.

save: put a pose in tuning_runs/config/viseme_library.json (search --save
does it with the best pose). Poses are mouth action vectors (mouth_actions.py):
relative to the session neutral, so they stay valid when the neutrals move.

The camera needs tuning_runs/config/face_markers.json with the eye holes,
modiolus_right/left, lower_lip and (for "opening") upper_lip dots.
--preview runs on the simulated board with a simulated mouth (no camera).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import random
import sys
import time
from typing import Callable, Dict, List, Optional, Sequence, Tuple

try:
    from . import face_stream as fs, mouth_actions as ma, mouth_shape as ms
    from . import records_layout, tuning_core, workspace_guard as wg
except ImportError:  # run as a plain script from this directory
    import face_stream as fs  # type: ignore[no-redef]
    import mouth_actions as ma  # type: ignore[no-redef]
    import mouth_shape as ms  # type: ignore[no-redef]
    import records_layout  # type: ignore[no-redef]
    import tuning_core  # type: ignore[no-redef]
    import workspace_guard as wg  # type: ignore[no-redef]

TARGETS_FILE = "viseme_targets.json"
TARGETS_FORMAT = "head-viseme-targets-1"
TARGET_CROPS_DIR = "viseme_targets"
LIBRARY_FILE = "viseme_library.json"
LIBRARY_FORMAT = "head-viseme-library-1"
TRIALS_FILE = "viseme_trials.json"
LANDMARK_MODEL_FILE = "face_landmarker.task"
REST_NAME = "rest"
PHOTO_SUFFIXES = (".jpg", ".jpeg", ".png")

# Settling (chosen, not measured): a pose counts as reached when no mouth
# servo's target moved more than SETTLE_TICKS over SETTLE_UPDATES updates
# (followers keep adjusting by a tick or two), then the skin gets DWELL_S.
# A pose not settled after SETTLE_TIMEOUT_S is measured anyway and flagged.
SETTLE_TICKS = 3
SETTLE_UPDATES = 10
DWELL_S = 0.5
SETTLE_TIMEOUT_S = 8.0
# Search (chosen): first step per value, smallest step, trial cap.
SEARCH_FIRST_STEP = 0.25
SEARCH_MIN_STEP = 0.05
SEARCH_MAX_TRIALS = 30
SEARCH_VALUES = ("jaw", "d", "v", "lip", "ulip")
SEARCH_BOUNDS = {"jaw": (0.0, 1.0), "d": (0.0, 1.0), "v": (-1.0, 1.0), "lip": (0.0, 1.0),
                 "ulip": (0.0, 1.0)}


def search_vector(p: Dict[str, float]) -> ma.Vector:
    """The mouth vector for symmetric search values."""
    return ma.check_vector({"jaw": p.get("jaw", 0.0), "right_d": p.get("d", 0.0),
                            "left_d": p.get("d", 0.0), "right_v": p.get("v", 0.0),
                            "left_v": p.get("v", 0.0), "lip_right": p.get("lip", 0.0),
                            "lip_left": p.get("lip", 0.0), "upper_lip_right": p.get("ulip", 0.0),
                            "upper_lip_left": p.get("ulip", 0.0)})


def search_values(v: ma.Vector) -> Dict[str, float]:
    """Symmetric search values of a vector (the mean of the two sides)."""
    return {"jaw": v["jaw"], "d": (v["right_d"] + v["left_d"]) / 2,
            "v": (v["right_v"] + v["left_v"]) / 2, "lip": (v["lip_right"] + v["lip_left"]) / 2,
            "ulip": (v["upper_lip_right"] + v["upper_lip_left"]) / 2}


def calibration_poses(upper_lips: bool) -> List[Tuple[str, ma.Vector]]:
    """The robot's rest, then one extreme per action (chosen): their measures
    give the robot's span per measure for this session."""
    poses = [("rest", ma.rest()),
             ("cal_jaw", search_vector({"jaw": 1.0})),
             ("cal_wide", search_vector({"d": 1.0})),
             ("cal_up", search_vector({"v": 1.0})),
             ("cal_down", search_vector({"v": -1.0})),
             ("cal_lower_lip", search_vector({"lip": 1.0}))]
    if upper_lips:
        poses.append(("cal_upper_lip", search_vector({"ulip": 1.0})))
    return poses


# Files ------------------------------------------------------------------------------
def write_json(path: Path, data: dict) -> Path:
    tuning_core.write_json_atomically(Path(path), data)
    return Path(path)


def load_targets(config: Path) -> dict:
    path = Path(config) / TARGETS_FILE
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("format") != TARGETS_FORMAT:
        raise ValueError(f"{path} is not a {TARGETS_FORMAT} file")
    return data


def load_library(config: Path) -> dict:
    path = Path(config) / LIBRARY_FILE
    if not path.exists():
        return {"format": LIBRARY_FORMAT, "visemes": {}}
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("format") != LIBRARY_FORMAT:
        raise ValueError(f"{path} is not a {LIBRARY_FORMAT} file")
    for name, entry in data["visemes"].items():
        entry["pose"] = ma.check_vector(entry["pose"])
    return data


def save_to_library(config: Path, name: str, pose: ma.Vector, **info) -> Path:
    library = load_library(config)
    library["visemes"][name] = {"pose": ma.check_vector(pose),
                                "saved_utc": datetime.now(timezone.utc).isoformat(), **info}
    return write_json(Path(config) / LIBRARY_FILE, library)


def find_photos(directory: Path) -> Dict[str, Path]:
    photos: Dict[str, Path] = {}
    for path in sorted(Path(directory).iterdir()):
        if path.suffix.lower() in PHOTO_SUFFIXES and path.is_file():
            name = path.stem.lower()
            if name in photos:
                raise ValueError(f"two photos for viseme {name!r}: {photos[name].name}, {path.name}")
            photos[name] = path
    if REST_NAME not in photos:
        raise ValueError(f"{directory} needs a {REST_NAME}.jpg (mouth relaxed and closed)")
    if len(photos) < 2:
        raise ValueError(f"{directory} needs {REST_NAME}.jpg and at least one viseme photo")
    return photos


def build_targets(photos: Dict[str, Path], find_landmarks: Callable, crops_dir: Optional[Path],
                  connections=None) -> dict:
    """The targets file content. find_landmarks(rgb) -> N x 2 pixels or None."""
    import numpy
    from PIL import Image

    shapes: Dict[str, ms.MouthShape] = {}
    entries: Dict[str, dict] = {}
    missing = []
    for name, path in photos.items():
        rgb = numpy.asarray(Image.open(path).convert("RGB"))
        landmarks = find_landmarks(rgb)
        if landmarks is None:
            missing.append(path.name)
            continue
        shape = ms.human_shape(landmarks, connections=connections)
        shapes[name] = shape
        box = ms.human_mouth_box(landmarks)
        entry = {"photo": str(path), "shape": shape.as_dict(), "mouth_box": list(box)}
        if crops_dir is not None:
            crop = Path(crops_dir) / f"{name}.jpg"
            crop.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(rgb).crop(box).save(crop, quality=90)
            entry["crop"] = str(crop)
        entries[name] = entry
    if missing:
        raise ValueError(f"no face found in {', '.join(missing)}")
    rest = shapes[REST_NAME]
    span = ms.spans(rest, shapes.values())
    for name, entry in entries.items():
        entry["normalized"] = ms.normalized(shapes[name], rest, span)
    return {"format": TARGETS_FORMAT, "made_utc": datetime.now(timezone.utc).isoformat(),
            "rest": rest.as_dict(), "spans": span,
            "visemes": {n: e for n, e in entries.items() if n != REST_NAME},
            "rest_entry": entries[REST_NAME]}


def _face_markers():
    try:
        import face_markers
    except ImportError:
        from . import face_markers  # type: ignore[no-redef]
    return face_markers


# Measuring the robot -------------------------------------------------------------------
@dataclass
class Measurement:
    shape: Optional[ms.MouthShape]
    rgb: object = None              # numpy array, or None (preview)
    found: Optional[dict] = None    # tracked markers
    note: str = ""


class CameraMeasurer:
    """Marker dots on the newest camera frame that came after a given time."""

    def __init__(self, recorder, markers: Dict[str, dict], clock=time.monotonic) -> None:
        try:
            import face_markers
        except ImportError:
            from . import face_markers  # type: ignore[no-redef]

        self.recorder = recorder
        self.tracker = face_markers.MarkerTracker(markers)
        self.clock = clock

    def ready(self, after_s: float) -> bool:
        latest = self.recorder.latest()
        return latest is not None and latest[0] > after_s

    def measure(self, _pose: ma.Vector) -> Measurement:
        latest = self.recorder.latest()
        if latest is None:
            return Measurement(None, note="no camera frame")
        _t, rgb = latest
        found = self.tracker.track(rgb)
        shape = ms.robot_shape(found, rgb)
        lost = [name for name, entry in found.items() if entry.get("lost")]
        return Measurement(shape, rgb, found, ("lost: " + ", ".join(lost)) if lost else "")


class SimulatedMeasurer:
    """--preview and tests: a made-up mouth that answers each pose with
    measures (not a model of the head), plus a little noise."""

    def __init__(self, seed: int = 1, noise: float = 0.002) -> None:
        self.random = random.Random(seed)
        self.noise = noise

    @staticmethod
    def shape_of(v: ma.Vector) -> ms.MouthShape:
        d = (v["right_d"] + v["left_d"]) / 2
        up = (max(0.0, v["right_v"]) + max(0.0, v["left_v"])) / 2
        down = (max(0.0, -v["right_v"]) + max(0.0, -v["left_v"])) / 2
        lip = (v["lip_right"] + v["lip_left"]) / 2
        ulip = (v["upper_lip_right"] + v["upper_lip_left"]) / 2
        width = 1.00 + 0.08 * d + 0.03 * up - 0.02 * down - 0.04 * ulip * (1 - d)
        opening = 0.30 + 0.10 * v["jaw"] + 0.03 * ulip - 0.03 * lip + 0.01 * down
        gap = max(0.0, 0.002 + 0.03 * v["jaw"] - 0.015 * lip - 0.015 * ulip)
        return ms.MouthShape(width, opening, gap, 200.0)

    def ready(self, _after_s: float) -> bool:
        return True

    def measure(self, pose: ma.Vector) -> Measurement:
        s = self.shape_of(pose)
        jitter = lambda x: x + self.random.gauss(0.0, self.noise)  # noqa: E731
        return Measurement(ms.MouthShape(jitter(s.width), jitter(s.opening), max(0.0, jitter(s.gap)),
                                         s.eye_px))


# Planners -----------------------------------------------------------------------------
Step = Optional[Tuple[str, ma.Vector]]


class ListPlanner:
    """Fixed poses, in order."""

    def __init__(self, poses: Sequence[Tuple[str, ma.Vector]]) -> None:
        self.poses = list(poses)
        self.results: List[Tuple[str, ma.Vector, Measurement]] = []

    def first(self) -> Step:
        return self.poses[0] if self.poses else None

    def observe(self, label: str, pose: ma.Vector, measurement: Measurement) -> Step:
        self.results.append((label, pose, measurement))
        index = len(self.results)
        return self.poses[index] if index < len(self.poses) else None


class SearchPlanner:
    """Coordinate search on the symmetric values (SEARCH_VALUES): try each
    free value one step up and down, keep the best if it lowers the score,
    halve the steps after a round with no improvement."""

    def __init__(self, score_fn: Callable[[Measurement], float], start: ma.Vector,
                 free: Sequence[str] = SEARCH_VALUES, first_step: float = SEARCH_FIRST_STEP,
                 min_step: float = SEARCH_MIN_STEP, max_trials: int = SEARCH_MAX_TRIALS) -> None:
        unknown = sorted(set(free) - set(SEARCH_VALUES))
        if unknown or not free:
            raise ValueError(f"free values {list(free)}: choose from {', '.join(SEARCH_VALUES)}")
        self.score_fn = score_fn
        self.current = search_values(start)
        self.free = list(free)
        self.step = first_step
        self.min_step = min_step
        self.max_trials = max_trials
        self.trials = 0
        self.best_score = math.inf
        self.best_pose = search_vector(self.current)
        self._param = 0
        self._queue: List[Dict[str, float]] = []
        self._round_improved = False
        self._started = False

    def first(self) -> Step:
        return ("start", search_vector(self.current))

    def _candidates(self, name: str) -> List[Dict[str, float]]:
        low, high = SEARCH_BOUNDS[name]
        out = []
        for sign in (1, -1):
            value = min(high, max(low, self.current[name] + sign * self.step))
            if abs(value - self.current[name]) > 1e-9:
                out.append({**self.current, name: round(value, 4)})
        return out

    def _next(self) -> Step:
        while True:
            if self.trials >= self.max_trials or self.step < self.min_step - 1e-9:
                return None
            if self._queue:
                p = self._queue.pop(0)
                return (f"try_{self.free[self._param]}", search_vector(p))
            # The candidates of the last value are done: move to the next.
            if self._started:
                self._param += 1
                if self._param >= len(self.free):
                    self._param = 0
                    if not self._round_improved:
                        self.step /= 2.0
                        if self.step < self.min_step - 1e-9:
                            return None
                    self._round_improved = False
            self._started = True
            self._queue = self._candidates(self.free[self._param])

    def observe(self, label: str, pose: ma.Vector, measurement: Measurement) -> Step:
        self.trials += 1
        value = self.score_fn(measurement)
        if value < self.best_score - 1e-12:
            self.best_score = value
            self.best_pose = pose
            if label != "start":
                self.current = search_values(pose)
                self._round_improved = True
                self._queue = []            # restart around the new best
        return self._next()


class ChainPlanner:
    """One planner after another (calibration, then the search or the list)."""

    def __init__(self, planners: Sequence, on_switch: Optional[Callable[[int], None]] = None) -> None:
        self.planners = list(planners)
        self.index = 0
        self.on_switch = on_switch

    def first(self) -> Step:
        return self._advance(self.planners[0].first())

    def _advance(self, step: Step) -> Step:
        while step is None and self.index + 1 < len(self.planners):
            self.index += 1
            if self.on_switch is not None:
                self.on_switch(self.index)
            step = self.planners[self.index].first()
        return step

    def observe(self, label: str, pose: ma.Vector, measurement: Measurement) -> Step:
        return self._advance(self.planners[self.index].observe(label, pose, measurement))


# The closed-loop script ---------------------------------------------------------------
class PoseSequencer:
    """A face_stream Script with on_command/on_update hooks: holds each pose
    from the planner until it settles, measures it, asks for the next; ends
    (None) after the planner's last pose. approach="rest" goes back to rest
    (not measured) before each pose."""

    def __init__(self, planner, measurer, on_measured: Callable[[str, ma.Vector, Measurement, dict], None],
                 approach: str = "direct", clock=time.monotonic,
                 settle_ticks: int = SETTLE_TICKS, settle_updates: int = SETTLE_UPDATES,
                 dwell_s: float = DWELL_S, timeout_s: float = SETTLE_TIMEOUT_S) -> None:
        if approach not in ("direct", "rest"):
            raise ValueError("approach must be 'direct' or 'rest'")
        self.planner, self.measurer, self.on_measured = planner, measurer, on_measured
        self.approach, self.clock = approach, clock
        self.settle_ticks, self.settle_updates = settle_ticks, settle_updates
        self.dwell_s, self.timeout_s = dwell_s, timeout_s
        self.label: Optional[str] = None
        self.pose: Optional[ma.Vector] = None
        self._next: Step = None
        self._via_rest = False
        self._pose_since_s = 0.0
        self._settled_since_s: Optional[float] = None
        self._history: List[Dict[int, int]] = []
        self.done = False
        self.started = False

    def _set(self, step: Step, now_s: float) -> None:
        if step is None:
            self.done = True
            return
        if self.approach == "rest" and self.pose is not None and any(step[1].values()):
            self._via_rest, self._next = True, step
            self.label, self.pose = "approach_rest", ma.rest()
        else:
            self._via_rest = False
            self.label, self.pose = step
        self._pose_since_s = now_s
        self._settled_since_s = None
        self._history = []

    def __call__(self, t_s: float):
        if not self.started:
            self.started = True
            self._set(self.planner.first(), self.clock())
        if self.done or self.pose is None:
            return None
        return ma.to_actions(self.pose)

    def on_command(self, _t_s: float, _actions, _j: float, targets: Dict[int, int]) -> None:
        mouth = {i: targets[i] for i in fs.COMMAND_LOG_SERVOS if i in targets}
        self._history.append(mouth)
        if len(self._history) > self.settle_updates:
            self._history.pop(0)

    def _is_settled(self) -> bool:
        if len(self._history) < self.settle_updates:
            return False
        first = self._history[0]
        return all(abs(h[i] - first[i]) <= self.settle_ticks for h in self._history for i in first)

    def on_update(self, _t_s: float) -> None:
        if self.done or self.pose is None:
            return
        now = self.clock()
        timed_out = now - self._pose_since_s > self.timeout_s
        if self._settled_since_s is None:
            if self._is_settled() or timed_out:
                self._settled_since_s = now
            return
        if now - self._settled_since_s < self.dwell_s and not timed_out:
            return
        if not self.measurer.ready(self._settled_since_s + self.dwell_s):
            if not timed_out:
                return
        if self._via_rest:
            step, self._via_rest = self._next, False
            self.label, self.pose = step
            self._pose_since_s, self._settled_since_s, self._history = now, None, []
            return
        started = time.perf_counter()
        measurement = self.measurer.measure(self.pose)
        info = {"measure_ms": round(1000 * (time.perf_counter() - started), 1),
                "settled": not timed_out, "held_s": round(now - self._pose_since_s, 2)}
        self.on_measured(self.label, self.pose, measurement, info)
        self._set(self.planner.observe(self.label, self.pose, measurement), now)


# One tuning run -----------------------------------------------------------------------
@dataclass
class TuningRun:
    """Calibration + trials, the files they write and the scores."""
    target_name: str
    targets: dict
    run: Path
    target_rgb: object = None
    calibration_labels: Tuple[str, ...] = ()
    robot_rest: Optional[ms.MouthShape] = None
    robot_span: Dict[str, Optional[float]] = field(default_factory=dict)
    calibration: List[Tuple[str, ms.MouthShape]] = field(default_factory=list)
    trials: List[dict] = field(default_factory=list)
    weights: Dict[str, float] = field(default_factory=lambda: dict(ms.DEFAULT_WEIGHTS))
    say: Callable[[str], None] = print

    @property
    def target_normalized(self) -> Dict[str, float]:
        return self.targets["visemes"][self.target_name]["normalized"]

    def score_of(self, measurement: Measurement) -> float:
        if measurement.shape is None or self.robot_rest is None:
            return math.inf
        robot = ms.normalized(measurement.shape, self.robot_rest, self.robot_span)
        return ms.score(robot, self.target_normalized, self.weights)[0]

    def finish_calibration(self) -> None:
        rest = dict(self.calibration).get("rest")
        if rest is None:
            raise RuntimeError("calibration: the robot's rest could not be measured (markers lost?)")
        self.robot_rest = rest
        self.robot_span = ms.spans(rest, (s for _n, s in self.calibration))
        self.say("Robot rest " + " ".join(f"{k}={v:.4f}" for k, v in rest.as_dict().items()
                                          if isinstance(v, float)) +
                 "; span " + " ".join(f"{k}={'-' if v is None else f'{v:.4f}'}"
                                      for k, v in self.robot_span.items()))

    def record(self, label: str, pose: ma.Vector, measurement: Measurement, info: dict) -> None:
        number = len(self.calibration) + len(self.trials) + 1
        entry = {"number": number, "label": label, "pose": pose, "note": measurement.note, **info,
                 "shape": None if measurement.shape is None else measurement.shape.as_dict()}
        still = composite = None
        if measurement.rgb is not None:
            from PIL import Image
            import numpy

            still = self.run / records_layout.STILLS / f"{number:02d}_{label}.jpg"
            still.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(numpy.ascontiguousarray(measurement.rgb)).save(still, quality=90)
            entry["still"] = str(still)
        if label in self.calibration_labels:
            if measurement.shape is not None:
                self.calibration.append((label, measurement.shape))
            self.say(f"  calibration {label}: " + ("markers lost" if measurement.shape is None else
                     " ".join(f"{k}={v:.4f}" for k, v in measurement.shape.as_dict().items()
                              if isinstance(v, float) and k != "eye_px")))
            entry["calibration"] = True
            self.trials.append(entry)
            return
        if self.robot_rest is None:
            self.finish_calibration()
        robot = ({} if measurement.shape is None else
                 ms.normalized(measurement.shape, self.robot_rest, self.robot_span))
        value, used = ms.score(robot, self.target_normalized, self.weights)
        entry.update({"normalized": robot, "target": self.target_normalized, "score": value,
                      "measures": used})
        if measurement.rgb is not None and self.target_rgb is not None and measurement.found:
            box = ms.robot_mouth_box(measurement.found)
            if box is not None:
                composite = self.run / "composites" / f"{number:02d}_{label}.jpg"
                ms.side_by_side(self.target_rgb, tuple(self.targets["visemes"][self.target_name]["mouth_box"]),
                                f"target {self.target_name}", measurement.rgb, box,
                                f"robot {label} score {value:.3f}", composite)
                entry["composite"] = str(composite)
        self.trials.append(entry)
        self.say(f"  {number:02d} {label}: score {value:.4f} ({', '.join(used) or 'no shared measure'})"
                 f"  {ma.format_pose(pose) or 'rest'}" + (f"  [{measurement.note}]" if measurement.note else ""))

    def best(self) -> Optional[dict]:
        scored = [t for t in self.trials if not t.get("calibration") and math.isfinite(t.get("score", math.inf))]
        return min(scored, key=lambda t: t["score"]) if scored else None

    def summary(self) -> dict:
        return {"format": "head-viseme-trials-1", "viseme": self.target_name,
                "robot_rest": None if self.robot_rest is None else self.robot_rest.as_dict(),
                "robot_span": self.robot_span, "weights": self.weights, "trials": self.trials,
                "best": self.best()}


def run_tuning(a: argparse.Namespace, mode: str, poses: Sequence[ma.Vector] = (),
               start: Optional[ma.Vector] = None, say=print, measurer_factory=None,
               session_factory=None) -> int:
    """try / search: one session. Returns the exit code."""
    config = records_layout.config_dir(a.records / "preview" if a.preview else a.records)
    try:
        targets = load_targets(config)
    except (OSError, ValueError) as error:
        say(f"No viseme targets ({error}): run 'head-viseme targets --photos DIR' first.")
        return 2
    if a.viseme not in targets["visemes"]:
        say(f"No target {a.viseme!r}; targets: {', '.join(sorted(targets['visemes']))}")
        return 2
    if not a.preview:
        a.camera = "realsense"                 # the tool measures with the camera
        try:
            markers = _face_markers().load_markers(config)
        except (OSError, ValueError) as error:
            say(f"Face markers: {error}"); return 2
        needed = {ms.ROBOT_MARKERS[k] for k in ("corner_right", "corner_left", "eye_right", "eye_left",
                                                "lower")}
        missing = sorted(needed - set(markers))
        if missing:
            say(f"face_markers.json needs {', '.join(missing)}"); return 2
        if ms.ROBOT_MARKERS["upper"] not in markers:
            say("Note: no upper_lip marker, so the 'opening' measure is not used.")
    try:
        setup = (session_factory or fs.open_session)(a, "viseme", f"{mode}_{a.viseme}", say)
    except fs.SetupError as error:
        say(str(error)); return 2
    session = setup.session
    target_rgb = None
    photo = targets["visemes"][a.viseme].get("photo")
    if photo and Path(photo).exists():
        import numpy
        from PIL import Image
        target_rgb = numpy.asarray(Image.open(photo).convert("RGB"))
    cal = calibration_poses(wg.upper_lips_configured())
    tuning = TuningRun(a.viseme, targets, setup.run, target_rgb, tuple(n for n, _p in cal), say=say)
    if getattr(a, "weight", None):
        for item in a.weight:
            name, _, value = item.partition("=")
            if name not in ms.MEASURES:
                say(f"bad --weight {item!r}"); fs.close_session(setup); return 2
            tuning.weights[name] = float(value)
    if measurer_factory is not None:
        measurer = measurer_factory(setup)
    elif a.preview:
        measurer = SimulatedMeasurer()
    else:
        measurer = CameraMeasurer(setup.recorder, _face_markers().load_markers(config))

    def switched(_index: int) -> None:
        tuning.finish_calibration()
    if mode == "search":
        free = [v for v in (a.free.split(",") if a.free else SEARCH_VALUES)
                if v != "ulip" or wg.upper_lips_configured()]
        main_planner = SearchPlanner(tuning.score_of, start or ma.rest(), free,
                                     max_trials=a.max_trials)
    else:
        main_planner = ListPlanner([(f"pose{i + 1}", p) for i, p in enumerate(poses)])
    planner = ChainPlanner([ListPlanner(cal), main_planner], on_switch=switched)
    sequencer = PoseSequencer(planner, measurer, tuning.record, approach=a.approach)
    command_log = session.on_command

    def on_command(t_s, actions, j, targets_sent):
        if command_log is not None:
            command_log(t_s, actions, j, targets_sent)
        sequencer.on_command(t_s, actions, j, targets_sent)
    session.on_command = on_command
    session.on_update = sequencer.on_update
    session.log["viseme"] = {"mode": mode, "name": a.viseme, "approach": a.approach}
    code = 1
    try:
        try:
            session.start()
        except RuntimeError as error:
            session.finish(f"not started: {error}")
            return 2
        say(f"Enabled; viseme {mode} for {a.viseme!r} ({len(cal)} calibration poses first).")
        try:
            reason = session.run(sequencer)
        except RuntimeError as error:          # e.g. the robot's rest could not be measured
            session.finish(str(error))
            reason = str(error)
        code = 0 if reason == "finished" else 1
    finally:
        summary = tuning.summary()
        write_json(setup.run / TRIALS_FILE, summary)
        session.log["viseme"]["trials"] = str(setup.run / TRIALS_FILE)
        fs.close_session(setup)
    best = tuning.best()
    if best is None:
        say("No pose was scored.")
        return code or 1
    say(f"Best for {a.viseme!r}: score {best['score']:.4f}: {ma.format_pose(best['pose']) or 'rest'}"
        + (f"\n  picture: {best['composite']}" if best.get("composite") else ""))
    if mode == "search" and a.save and code == 0:
        path = save_to_library(config, a.viseme, best["pose"], score=best["score"], run=str(setup.run))
        say(f"Saved to {path}")
    return code


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)
    targets_cmd = commands.add_parser("targets", help="measure the viseme photos")
    targets_cmd.add_argument("--photos", type=Path, required=True)
    targets_cmd.add_argument("--records", type=Path, default=fs.DEFAULT_RECORDS)
    targets_cmd.add_argument("--model", type=Path,
                             help=f"MediaPipe face landmarker model (default: config/{LANDMARK_MODEL_FILE})")
    targets_cmd.add_argument("--preview", action="store_true",
                             help="write into the preview records (for trying search --preview)")
    for name, text in (("try", "measure given poses"), ("search", "search for the best pose")):
        cmd = commands.add_parser(name, help=text)
        fs.add_session_arguments(cmd)
        cmd.add_argument("--viseme", required=True)
        cmd.add_argument("--approach", choices=("direct", "rest"), default="direct",
                         help="go to each pose directly, or through rest (default direct)")
        cmd.add_argument("--weight", action="append", default=[], metavar="MEASURE=W",
                         help="score weight per measure (width, opening, gap; default 1)")
        if name == "try":
            cmd.add_argument("--pose", action="append", default=[], required=True,
                             help="a mouth pose, e.g. 'jaw=0.3 lip_right=0.5 lip_left=0.5' (repeat)")
        else:
            cmd.add_argument("--start", help="start pose (default rest)")
            cmd.add_argument("--free", help=f"values to search (default {','.join(SEARCH_VALUES)})")
            cmd.add_argument("--max-trials", type=int, default=SEARCH_MAX_TRIALS)
            cmd.add_argument("--save", action="store_true", help="save the best pose to the library")
    save_cmd = commands.add_parser("save", help="put a pose in the viseme library")
    save_cmd.add_argument("--viseme", required=True)
    save_cmd.add_argument("--pose", required=True)
    save_cmd.add_argument("--note", default="")
    save_cmd.add_argument("--records", type=Path, default=fs.DEFAULT_RECORDS)
    save_cmd.add_argument("--preview", action="store_true")
    show_cmd = commands.add_parser("show", help="print the targets and the library")
    show_cmd.add_argument("--records", type=Path, default=fs.DEFAULT_RECORDS)
    show_cmd.add_argument("--preview", action="store_true")
    a = parser.parse_args(argv)
    say = lambda text: print(text, flush=True)  # noqa: E731
    records = a.records / "preview" if a.preview else a.records
    config = records_layout.config_dir(records)
    if a.command == "targets":
        model = a.model or config / LANDMARK_MODEL_FILE
        try:
            photos = find_photos(a.photos)
            finder = ms.LandmarkFinder(model)
        except (OSError, ValueError) as error:
            say(str(error)); return 2
        try:
            data = build_targets(photos, finder.find, config / TARGET_CROPS_DIR)
        except ValueError as error:
            say(str(error)); return 2
        finally:
            finder.close()
        path = write_json(config / TARGETS_FILE, data)
        for name, entry in sorted(data["visemes"].items()):
            say(f"  {name}: " + " ".join(f"{k}={v:+.2f}" for k, v in entry["normalized"].items()))
        say(f"Targets: {path} ({len(data['visemes'])} visemes); crops in {config / TARGET_CROPS_DIR}")
        return 0
    if a.command == "save":
        try:
            pose = ma.parse_pose(a.pose)
        except ValueError as error:
            say(str(error)); return 2
        path = save_to_library(config, a.viseme, pose, note=a.note)
        say(f"Saved {a.viseme!r} to {path}")
        return 0
    if a.command == "show":
        try:
            targets = load_targets(config)
            for name, entry in sorted(targets["visemes"].items()):
                say(f"target {name}: " + " ".join(f"{k}={v:+.2f}" for k, v in entry["normalized"].items()))
        except (OSError, ValueError) as error:
            say(f"No targets ({error})")
        for name, entry in sorted(load_library(config)["visemes"].items()):
            say(f"library {name}: {ma.format_pose(entry['pose']) or 'rest'}"
                + (f"  score {entry['score']:.4f}" if "score" in entry else ""))
        return 0
    try:
        if a.command == "try":
            poses = [ma.parse_pose(text) for text in a.pose]
            return run_tuning(a, "try", poses=poses, say=say)
        start = ma.parse_pose(a.start) if a.start else None
        if not 1 <= a.max_trials <= 200:
            say("--max-trials must be 1-200"); return 2
        return run_tuning(a, "search", start=start, say=say)
    except ValueError as error:
        say(str(error)); return 2


if __name__ == "__main__":
    sys.exit(main())
