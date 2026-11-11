"""Regression test for finding the face in the camera picture (knee_test.locate_corners).
Needs OpenCV, so it runs with the bench venv."""

from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace

sys.modules.setdefault("serial", SimpleNamespace(Serial=object))
FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))

import cv2  # noqa: E402
import numpy  # noqa: E402

import knee_test as kt  # noqa: E402


def test_boxes_follow_the_face() -> None:
    reference = cv2.imread(str(kt.FACE_REFERENCE), cv2.IMREAD_GRAYSCALE)
    assert reference is not None, kt.FACE_REFERENCE
    boxes, score, scale = kt.locate_corners(reference, reference)
    assert score > 0.99 and scale == 1.0 and boxes == kt.CORNER_REGIONS
    for dx, dy, s in ((140, 36, 0.92), (-60, 20, 1.08), (30, -25, 1.0)):
        moved = cv2.warpAffine(reference, numpy.float32([[s, 0, dx], [0, s, dy]]),
                               (reference.shape[1], reference.shape[0]))
        boxes, score, scale = kt.locate_corners(moved, reference)
        assert score > 0.8 and abs(scale - s) <= 0.021, (dx, dy, s, score, scale)
        for side, (a, b, c, d) in kt.CORNER_REGIONS.items():
            want = (a * s + dy, b * s + dy, c * s + dx, d * s + dx)
            assert all(abs(got - w) <= 6 for got, w in zip(boxes[side], want)), (side, boxes[side], want)


def main() -> None:
    test_boxes_follow_the_face()
    print("knee camera: all tests passed")


if __name__ == "__main__":
    main()
