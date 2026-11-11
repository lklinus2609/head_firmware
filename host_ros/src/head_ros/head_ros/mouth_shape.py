"""Mouth shape measures, for the robot (ink dots) and a person (photos).

Both sides give the same three measures, each divided by the distance
between the eyes so that camera distance does not matter:

    width    mouth corner to mouth corner
             robot: modiolus_right .. modiolus_left dots
             person: MediaPipe landmarks 61 .. 291 (outer lip corners)
    opening  upper lip to lower lip, on the outside of the lips
             robot: upper_lip .. lower_lip dots (None without an upper_lip dot)
             person: landmarks 0 .. 17 (outer lip centres)
    gap      how far the lips are apart
             robot: fraction of dark pixels (mouth cavity) in the box between
                    the lips; 0 when they touch
             person: landmarks 13 .. 14 (inner lip centres)

The landmark numbers are from the FACE_LANDMARKS_LIPS table of the installed
mediapipe 1.1.1 (FaceLandmarksConnections): 61 and 291 end both outer lip
chains, 0 and 17 are the middle of the upper and lower outer chains, 13 and
14 the middle of the upper and lower inner chains. mouth_shape_test.py checks
this against the installed table when mediapipe is importable.

The robot and the person have different mouths, so the measures are compared
as changes from each one's own rest, divided by each one's own largest change
(normalized(): about -1..1). A viseme target is then a set of normalized
changes, and score() is the weighted squared difference.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy

MEASURES = ("width", "opening", "gap")
# MediaPipe face mesh landmark numbers (see the module docstring).
LIP_CORNER_RIGHT, LIP_CORNER_LEFT = 61, 291
OUTER_UPPER_CENTRE, OUTER_LOWER_CENTRE = 0, 17
INNER_UPPER_CENTRE, INNER_LOWER_CENTRE = 13, 14
# Robot marker names (tuning_runs/config/face_markers.json).
ROBOT_MARKERS = {"corner_right": "modiolus_right", "corner_left": "modiolus_left",
                 "upper": "upper_lip", "lower": "lower_lip",
                 "eye_right": "eye_right", "eye_left": "eye_left"}
# Dark-pixel threshold: a pixel is "mouth cavity" when darker than this
# fraction of the median brightness of the box. Chosen, not measured: on two
# stills of 2026-10-23 (jaw 4000 vs 3880, 193741_face_duchenne) 0.4 gave
# 0.0000 closed and 0.0094 open.
DARK_FRACTION_OF_MEDIAN = 0.4
# Without an upper_lip dot, the gap box reaches this many eye distances up
# from the lower_lip dot (on 2026-10-23 the mouth line sat about 0.17 eye
# distances above that dot).
GAP_BOX_HEIGHT_EYES = 0.25
# The gap box covers the middle of the mouth: this fraction of the corner
# distance is left out at each end.
GAP_BOX_SIDE_MARGIN = 0.2
# The ink dots are dark too: a circle of this many eye distances around each
# tracked dot is left out of the gap (chosen: about 6 px at the 2026-10-23
# eye distance of about 200 px; the dots are about 1 mm).
DOT_EXCLUDE_EYES = 0.03


@dataclass
class MouthShape:
    width: Optional[float]
    opening: Optional[float]
    gap: Optional[float]
    eye_px: float

    def as_dict(self) -> Dict[str, Optional[float]]:
        return asdict(self)

    @staticmethod
    def from_dict(data: Dict) -> "MouthShape":
        return MouthShape(data.get("width"), data.get("opening"), data.get("gap"), float(data["eye_px"]))


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def gray(rgb: numpy.ndarray) -> numpy.ndarray:
    return rgb[..., :3].astype(numpy.float64) @ numpy.array([0.299, 0.587, 0.114])


def dark_fraction(gray_image: numpy.ndarray, box: Tuple[int, int, int, int],
                  threshold: float = DARK_FRACTION_OF_MEDIAN,
                  exclude: Sequence[Tuple[float, float, float]] = ()) -> Optional[float]:
    """Fraction of pixels in box (x0, y0, x1, y1) darker than threshold x the
    box's median, leaving out the circles (x, y, radius) in exclude (the ink
    dots); None for an empty box."""
    height, width = gray_image.shape
    x0, y0, x1, y1 = (max(0, box[0]), max(0, box[1]), min(width, box[2]), min(height, box[3]))
    if x1 - x0 < 2 or y1 - y0 < 2:
        return None
    patch = gray_image[y0:y1, x0:x1]
    keep = numpy.ones(patch.shape, dtype=bool)
    if exclude:
        yy, xx = numpy.mgrid[y0:y1, x0:x1]
        for x, y, radius in exclude:
            keep &= (xx - x) ** 2 + (yy - y) ** 2 > radius ** 2
    values = patch[keep]
    if values.size < 4:
        return None
    median = float(numpy.median(values))
    if median <= 0:
        return None
    return float(numpy.mean(values < threshold * median))


def robot_gap_box(points: Dict[str, Sequence[float]], eye_px: float) -> Optional[Tuple[int, int, int, int]]:
    right, left = points.get("corner_right"), points.get("corner_left")
    lower = points.get("lower")
    if right is None or left is None or lower is None:
        return None
    x_low, x_high = sorted((right[0], left[0]))
    margin = GAP_BOX_SIDE_MARGIN * (x_high - x_low)
    upper = points.get("upper")
    y_top = upper[1] if upper is not None else lower[1] - GAP_BOX_HEIGHT_EYES * eye_px
    return (int(round(x_low + margin)), int(round(min(y_top, lower[1]))),
            int(round(x_high - margin)), int(round(max(y_top, lower[1]))))


def robot_shape(found: Dict[str, dict], rgb: Optional[numpy.ndarray] = None) -> Optional[MouthShape]:
    """The robot's mouth shape from MarkerTracker.track() output (marker ->
    {"xy", ...} or {"lost": True}) and the picture (for the gap). None when
    the eye holes or the corner dots are lost."""
    points = {}
    for role, name in ROBOT_MARKERS.items():
        entry = found.get(name)
        if entry is not None and not entry.get("lost"):
            points[role] = entry["xy"]
    if "eye_right" not in points or "eye_left" not in points:
        return None
    eye_px = _distance(points["eye_right"], points["eye_left"])
    if eye_px <= 0:
        return None
    width = (_distance(points["corner_right"], points["corner_left"]) / eye_px
             if "corner_right" in points and "corner_left" in points else None)
    if width is None:
        return None
    opening = (_distance(points["upper"], points["lower"]) / eye_px
               if "upper" in points and "lower" in points else None)
    gap = None
    if rgb is not None:
        box = robot_gap_box(points, eye_px)
        if box is not None:
            radius = DOT_EXCLUDE_EYES * eye_px
            dots = [(p[0], p[1], radius) for role, p in points.items() if not role.startswith("eye")]
            gap = dark_fraction(gray(rgb), box, exclude=dots)
    return MouthShape(width, opening, gap, eye_px)


def robot_mouth_box(found: Dict[str, dict], scale: float = 1.6) -> Optional[Tuple[int, int, int, int]]:
    """A square box around the robot's mouth (for crops): centred between the
    corner dots and the lower lip dot, scale x the corner distance wide."""
    right, left = found.get("modiolus_right"), found.get("modiolus_left")
    if not right or not left or right.get("lost") or left.get("lost"):
        return None
    cx = (right["xy"][0] + left["xy"][0]) / 2.0
    cy = (right["xy"][1] + left["xy"][1]) / 2.0
    lower = found.get("lower_lip")
    if lower and not lower.get("lost"):
        cy = (cy + lower["xy"][1]) / 2.0
    half = scale * _distance(right["xy"], left["xy"]) / 2.0
    return (int(round(cx - half)), int(round(cy - half)), int(round(cx + half)), int(round(cy + half)))


# The person's side -----------------------------------------------------------------
def eye_centres(landmarks_px: numpy.ndarray, connections=None) -> Tuple[numpy.ndarray, numpy.ndarray]:
    """(right eye centre, left eye centre) in pixels: the mean of each eye's
    landmarks in MediaPipe's FACE_LANDMARKS_RIGHT_EYE / _LEFT_EYE tables."""
    if connections is None:
        from mediapipe.tasks.python.vision import FaceLandmarksConnections as connections
    right = sorted({i for c in connections.FACE_LANDMARKS_RIGHT_EYE for i in (c.start, c.end)})
    left = sorted({i for c in connections.FACE_LANDMARKS_LEFT_EYE for i in (c.start, c.end)})
    return landmarks_px[right].mean(axis=0), landmarks_px[left].mean(axis=0)


def human_shape(landmarks_px: numpy.ndarray, eyes: Optional[Tuple[numpy.ndarray, numpy.ndarray]] = None,
                connections=None) -> MouthShape:
    """A person's mouth shape from MediaPipe face landmarks in pixels (N x 2)."""
    right_eye, left_eye = eyes if eyes is not None else eye_centres(landmarks_px, connections)
    eye_px = _distance(right_eye, left_eye)
    if eye_px <= 0:
        raise ValueError("eye centres coincide")
    p = landmarks_px
    return MouthShape(_distance(p[LIP_CORNER_RIGHT], p[LIP_CORNER_LEFT]) / eye_px,
                      _distance(p[OUTER_UPPER_CENTRE], p[OUTER_LOWER_CENTRE]) / eye_px,
                      _distance(p[INNER_UPPER_CENTRE], p[INNER_LOWER_CENTRE]) / eye_px,
                      eye_px)


def human_mouth_box(landmarks_px: numpy.ndarray, scale: float = 1.6) -> Tuple[int, int, int, int]:
    right, left = landmarks_px[LIP_CORNER_RIGHT], landmarks_px[LIP_CORNER_LEFT]
    centre = (landmarks_px[OUTER_UPPER_CENTRE] + landmarks_px[OUTER_LOWER_CENTRE]) / 2.0
    half = scale * _distance(right, left) / 2.0
    return (int(round(centre[0] - half)), int(round(centre[1] - half)),
            int(round(centre[0] + half)), int(round(centre[1] + half)))


class LandmarkFinder:
    """MediaPipe FaceLandmarker (image mode, one face) on RGB arrays."""

    def __init__(self, model_path: Path) -> None:
        import mediapipe
        from mediapipe.tasks.python import vision
        from mediapipe.tasks.python.core.base_options import BaseOptions

        if not Path(model_path).is_file():
            raise FileNotFoundError(f"no MediaPipe face landmarker model at {model_path}")
        self._mp = mediapipe
        options = vision.FaceLandmarkerOptions(base_options=BaseOptions(model_asset_path=str(model_path)),
                                               num_faces=1)
        self._landmarker = vision.FaceLandmarker.create_from_options(options)

    def find(self, rgb: numpy.ndarray) -> Optional[numpy.ndarray]:
        """N x 2 landmark pixels, or None when no face is found."""
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB,
                               data=numpy.ascontiguousarray(rgb[..., :3]))
        result = self._landmarker.detect(image)
        if not result.face_landmarks:
            return None
        height, width = rgb.shape[:2]
        return numpy.array([[point.x * width, point.y * height] for point in result.face_landmarks[0]])

    def close(self) -> None:
        self._landmarker.close()


# Comparing shapes ------------------------------------------------------------------
def spans(rest: MouthShape, shapes: Iterable[MouthShape]) -> Dict[str, Optional[float]]:
    """Per measure, the largest |shape - rest| over shapes (None when no shape
    has the measure or it never changes)."""
    out: Dict[str, Optional[float]] = {}
    shapes = list(shapes)
    for name in MEASURES:
        base = getattr(rest, name)
        changes = [abs(getattr(s, name) - base) for s in shapes
                   if base is not None and getattr(s, name) is not None]
        largest = max(changes, default=0.0)
        out[name] = largest if largest > 1e-9 else None
    return out


def normalized(shape: MouthShape, rest: MouthShape, span: Dict[str, Optional[float]]) -> Dict[str, float]:
    """measure -> (shape - rest) / span, for the measures all three have."""
    out = {}
    for name in MEASURES:
        value, base, size = getattr(shape, name), getattr(rest, name), span.get(name)
        if value is not None and base is not None and size:
            out[name] = (value - base) / size
    return out


DEFAULT_WEIGHTS = {"width": 1.0, "opening": 1.0, "gap": 1.0}


def score(robot: Dict[str, float], target: Dict[str, float],
          weights: Optional[Dict[str, float]] = None) -> Tuple[float, List[str]]:
    """(weighted sum of squared differences over the measures both have,
    measures used). inf when none is shared."""
    weights = DEFAULT_WEIGHTS if weights is None else weights
    used = [name for name in MEASURES if name in robot and name in target and weights.get(name, 0) > 0]
    if not used:
        return math.inf, []
    return sum(weights[name] * (robot[name] - target[name]) ** 2 for name in used), used


def side_by_side(left_rgb: numpy.ndarray, left_box: Tuple[int, int, int, int], left_label: str,
                 right_rgb: numpy.ndarray, right_box: Tuple[int, int, int, int], right_label: str,
                 path: Path, height: int = 320) -> Path:
    """Save two mouth crops next to each other with a label above each."""
    from PIL import Image, ImageDraw

    def crop(rgb, box):
        image = Image.fromarray(numpy.ascontiguousarray(rgb[..., :3]))
        image = image.crop(box)
        scale = height / max(1, image.height)
        return image.resize((max(1, round(image.width * scale)), height))

    a, b = crop(left_rgb, left_box), crop(right_rgb, right_box)
    band = 24
    canvas = Image.new("RGB", (a.width + b.width + 8, height + band), (255, 255, 255))
    canvas.paste(a, (0, band))
    canvas.paste(b, (a.width + 8, band))
    draw = ImageDraw.Draw(canvas)
    draw.text((4, 4), left_label, fill=(0, 0, 0))
    draw.text((a.width + 12, 4), right_label, fill=(0, 0, 0))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(path, quality=90)
    return path
