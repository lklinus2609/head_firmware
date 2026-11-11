"""Regression tests for the face marker tracker (needs numpy: run with
bench/.venv/bin/python, see tests/README.md)."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile

import numpy

FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))

import face_markers  # noqa: E402


def picture(spots, size=(240, 320), noise=2.5, seed=0):
    """Grey skin with dark Gaussian spots: (x, y, sigma, depth) each."""
    rng = numpy.random.default_rng(seed)
    yy, xx = numpy.mgrid[0:size[0], 0:size[1]]
    image = 200.0 + rng.normal(0, noise, size)
    for x, y, sigma, depth in spots:
        image -= depth * numpy.exp(-((xx - x) ** 2 + (yy - y) ** 2) / (2 * sigma ** 2))
    return numpy.repeat(numpy.clip(image, 0, 255)[..., None], 3, axis=2).astype(numpy.uint8)


def test_subpixel_motion_and_reference_correction() -> None:
    markers = {"dot": {"xy": [97, 103], "kind": "dot", "reference": False},
               "eye": {"xy": [200, 60], "kind": "hole", "reference": True}}
    tracker = face_markers.MarkerTracker(markers)
    tracker.track(picture([(100.0, 100.0, 1.0, 60), (200, 60, 8, 120)]))
    # The dot moves +2.4/-1.3 px; the whole picture also shifts +3/+1 px
    # (camera nudge), which the eye reference must take off.
    found = tracker.track(picture([(105.4, 99.7, 1.0, 60), (203, 61, 8, 120)], seed=1))
    moved = tracker.displacement(found)
    dx, dy = moved["dot"]["px"]
    assert abs(dx - 2.4) < 0.2 and abs(dy + 1.3) < 0.2, moved
    assert moved["dot"]["mm"] is None


def test_lost_marker_and_file_format() -> None:
    tracker = face_markers.MarkerTracker({"dot": {"xy": [100, 100], "kind": "dot",
                                                  "reference": False}})
    tracker.track(picture([(100, 100, 1.0, 60)]))
    found = tracker.track(picture([]))          # the dot is gone
    assert found["dot"].get("lost") and tracker.displacement(found)["dot"]["px"] is None
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / face_markers.MARKERS_FILE
        path.write_text(json.dumps({"format": face_markers.MARKERS_FORMAT, "markers": {
            "a": {"xy": [1, 2]}, "b": {"xy": [3, 4], "kind": "hole", "reference": True}}}))
        loaded = face_markers.load_markers(Path(directory))
        assert loaded["a"] == {"xy": [1.0, 2.0], "kind": "dot", "reference": False}
        assert loaded["b"]["reference"] and loaded["b"]["kind"] == "hole"


def main() -> None:
    test_subpixel_motion_and_reference_correction()
    test_lost_marker_and_file_format()
    print("face markers: all tests passed")


if __name__ == "__main__":
    main()
