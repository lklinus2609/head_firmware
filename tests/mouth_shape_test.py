"""Tests for the mouth measures (mouth_shape.py), on drawn pictures."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import tempfile

import numpy

FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))

import face_markers  # noqa: E402
import mouth_shape as ms  # noqa: E402


def face(gap_rows: int = 0, corner_dx: int = 0, lower_dy: int = 0) -> numpy.ndarray:
    """A grey face with eye holes, four ink dots and a dark mouth slit of
    gap_rows rows between the lip dots."""
    rgb = numpy.full((400, 600, 3), 180, dtype=numpy.uint8)
    yy, xx = numpy.mgrid[0:400, 0:600]

    def spot(x, y, radius, value):
        rgb[(xx - x) ** 2 + (yy - y) ** 2 <= radius ** 2] = value
    spot(200, 100, 14, 20)                       # eye holes, 200 px apart
    spot(400, 100, 14, 20)
    spot(220 - corner_dx, 260, 2, 60)            # modiolus dots
    spot(380 + corner_dx, 260, 2, 60)
    spot(300, 240, 2, 60)                        # upper lip dot
    spot(300, 290 + lower_dy, 2, 60)             # lower lip dot
    if gap_rows:
        rgb[262:262 + gap_rows, 270:331] = 10
    return rgb


MARKERS = {"modiolus_right": {"xy": [220, 260], "kind": "dot", "reference": False},
           "modiolus_left": {"xy": [380, 260], "kind": "dot", "reference": False},
           "upper_lip": {"xy": [300, 240], "kind": "dot", "reference": False},
           "lower_lip": {"xy": [300, 290], "kind": "dot", "reference": False},
           "eye_right": {"xy": [200, 100], "kind": "hole", "reference": True},
           "eye_left": {"xy": [400, 100], "kind": "hole", "reference": True}}


def measure(rgb):
    tracker = face_markers.MarkerTracker(MARKERS)
    found = tracker.track(rgb)
    return ms.robot_shape(found, rgb), found


def test_robot_measures_follow_the_dots_and_the_gap() -> None:
    rest, found = measure(face())
    assert not any(entry.get("lost") for entry in found.values())
    assert abs(rest.eye_px - 200) < 0.5
    assert abs(rest.width - 160 / 200) < 0.01 and abs(rest.opening - 50 / 200) < 0.01
    assert rest.gap == 0.0
    wide, _ = measure(face(corner_dx=10))
    assert abs(wide.width - 180 / 200) < 0.01
    open_mouth, _ = measure(face(gap_rows=6, lower_dy=8))
    assert open_mouth.gap > 0.03 and abs(open_mouth.opening - 58 / 200) < 0.01
    box = ms.robot_mouth_box(found)
    assert box[2] - box[0] == box[3] - box[1] and box[0] < 220 < 380 < box[2]


def test_lost_markers() -> None:
    rgb = face()
    found = {"eye_right": {"xy": [200, 100]}, "eye_left": {"lost": True, "xy": [400, 100]}}
    assert ms.robot_shape(found, rgb) is None
    tracker = face_markers.MarkerTracker({k: v for k, v in MARKERS.items() if k != "upper_lip"})
    shape = ms.robot_shape(tracker.track(rgb), rgb)
    assert shape.opening is None and shape.width is not None and shape.gap is not None


def test_dark_fraction() -> None:
    gray = numpy.full((20, 20), 100.0)
    assert ms.dark_fraction(gray, (0, 0, 20, 20)) == 0.0
    gray[0:4, :] = 10.0
    assert abs(ms.dark_fraction(gray, (0, 0, 20, 20)) - 0.2) < 1e-9
    assert ms.dark_fraction(gray, (5, 5, 6, 6)) is None


class Connections:
    """Stand-in for mediapipe's FaceLandmarksConnections (eyes only)."""
    class C:
        def __init__(self, start, end):
            self.start, self.end = start, end
    FACE_LANDMARKS_RIGHT_EYE = [C(33, 133)]
    FACE_LANDMARKS_LEFT_EYE = [C(362, 263)]


def test_human_measures_from_landmarks() -> None:
    points = numpy.zeros((478, 2))
    points[[33, 133]] = [[90, 100], [110, 100]]          # right eye centre (100, 100)
    points[[362, 263]] = [[290, 100], [310, 100]]        # left eye centre (300, 100)
    points[ms.LIP_CORNER_RIGHT], points[ms.LIP_CORNER_LEFT] = [140, 300], [260, 300]
    points[ms.OUTER_UPPER_CENTRE], points[ms.OUTER_LOWER_CENTRE] = [200, 280], [200, 330]
    points[ms.INNER_UPPER_CENTRE], points[ms.INNER_LOWER_CENTRE] = [200, 295], [200, 305]
    shape = ms.human_shape(points, connections=Connections)
    assert shape.eye_px == 200 and shape.width == 0.6 and shape.opening == 0.25 and shape.gap == 0.05


def test_normalized_and_score() -> None:
    rest = ms.MouthShape(0.8, 0.25, 0.0, 200)
    shapes = [rest, ms.MouthShape(0.9, 0.25, 0.02, 200), ms.MouthShape(0.7, 0.35, 0.01, 200)]
    span = ms.spans(rest, shapes)
    assert abs(span["width"] - 0.1) < 1e-12 and abs(span["opening"] - 0.1) < 1e-12 and span["gap"] == 0.02
    n = ms.normalized(shapes[2], rest, span)
    assert abs(n["width"] + 1) < 1e-9 and abs(n["opening"] - 1) < 1e-9 and abs(n["gap"] - 0.5) < 1e-9
    value, used = ms.score(n, {"width": -1.0, "opening": 0.5, "gap": 0.5})
    assert abs(value - 0.25) < 1e-9 and used == ["width", "opening", "gap"]
    assert ms.score({}, {"width": 1})[0] == math.inf
    no_change = ms.spans(rest, [rest])
    assert no_change == {"width": None, "opening": None, "gap": None}
    assert ms.normalized(rest, rest, no_change) == {}


def test_side_by_side_picture() -> None:
    path = Path(tempfile.mkdtemp()) / "pair.jpg"
    ms.side_by_side(face(), (200, 200, 400, 340), "target", face(gap_rows=4), (200, 200, 400, 340),
                    "robot", path)
    from PIL import Image
    image = Image.open(path)
    assert image.height == 320 + 24 and image.width > 600


def test_landmark_numbers_match_the_installed_mediapipe() -> None:
    try:
        from mediapipe.tasks.python.vision import FaceLandmarksConnections as connections
    except ImportError:
        print("  (mediapipe not importable: landmark table not checked)")
        return

    def chain(start, end):
        links = {}
        for c in connections.FACE_LANDMARKS_LIPS:
            links.setdefault(c.start, []).append(c.end)
        paths = [[start]]
        while paths:
            path = paths.pop()
            if path[-1] == end and len(path) > 1:
                yield path
                continue
            for nxt in links.get(path[-1], []):
                paths.append(path + [nxt])
    outer = [p for p in chain(ms.LIP_CORNER_RIGHT, ms.LIP_CORNER_LEFT)]
    middles = {p[len(p) // 2] for p in outer}
    assert middles == {ms.OUTER_UPPER_CENTRE, ms.OUTER_LOWER_CENTRE}, middles
    inner = [p for p in chain(78, 308)]
    assert {p[len(p) // 2] for p in inner} == {ms.INNER_UPPER_CENTRE, ms.INNER_LOWER_CENTRE}


def main() -> None:
    test_robot_measures_follow_the_dots_and_the_gap()
    test_lost_markers()
    test_dark_fraction()
    test_human_measures_from_landmarks()
    test_normalized_and_score()
    test_side_by_side_picture()
    test_landmark_numbers_match_the_installed_mediapipe()
    print("mouth shape: all tests passed")


if __name__ == "__main__":
    main()
