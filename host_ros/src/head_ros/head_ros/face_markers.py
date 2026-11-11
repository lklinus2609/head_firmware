"""Track marker dots and skin-visible magnets in face pictures.

A marker is found as the strongest dark spot near where it was last time: the
picture is blurred at a fine and a coarse scale, and the coarse minus the fine
blur peaks on a dark spot of about the fine scale. Searching only near the
last position matters: across the whole lower face the mouth line, the holes
and the edges of the white piece look just as strong (2026-10-18). The centre
is the weighted mean of the peak, so it is finer than one pixel.

With an aligned depth picture, each marker also gets a camera-frame position
in mm, so displacements are in mm without a ruler.

Markers start from tuning_runs/config/face_markers.json:

    {"format": "head-face-markers-1",
     "markers": {"modiolus_right": {"xy": [550, 553], "kind": "dot"},
                 "eye_right": {"xy": [568, 298], "kind": "hole",
                               "reference": true}}}

"right" and "left" are the face's own sides. Reference markers do not move
with the face; their mean shift is taken off the others, so a small camera or
head movement does not show up as face motion.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional

import numpy

MARKERS_FILE = "face_markers.json"
MARKERS_FORMAT = "head-face-markers-1"
# kind -> (fine sigma px, coarse sigma px, search half-window px, min contrast)
# Measured 2026-10-18: ~1 mm ink dots gave a peak of 34-36 against skin noise
# of sd 2.6-2.8 (fine 0.8, coarse 4); the magnets are soft ~25 px grey discs.
# Eye holes ("hole"): large and very dark; in the upper face, which the jaw
# does not move, so they serve as references (forehead magnets were too faint
# in brighter light, contrast 9-12, 2026-10-18).
KINDS = {"dot": (0.8, 4.0, 25, 12.0), "magnet": (3.0, 10.0, 30, 4.0),
         "hole": (5.0, 15.0, 30, 15.0)}
# Depth is the median of this many pixels around the marker, zeros (no
# depth) left out.
DEPTH_HALF_WINDOW = 3


def _gauss_kernel(sigma: float) -> numpy.ndarray:
    radius = max(1, int(3 * sigma + 0.5))
    x = numpy.arange(-radius, radius + 1, dtype=numpy.float64)
    kernel = numpy.exp(-0.5 * (x / sigma) ** 2)
    return kernel / kernel.sum()


def _blur(image: numpy.ndarray, sigma: float) -> numpy.ndarray:
    kernel = _gauss_kernel(sigma)
    pad = len(kernel) // 2
    padded = numpy.pad(image, pad, mode="reflect")
    rows = numpy.apply_along_axis(lambda r: numpy.convolve(r, kernel, mode="valid"), 1, padded)
    return numpy.apply_along_axis(lambda c: numpy.convolve(c, kernel, mode="valid"), 0, rows)


def gray(rgb: numpy.ndarray) -> numpy.ndarray:
    return rgb[..., :3].astype(numpy.float64) @ numpy.array([0.299, 0.587, 0.114])


def locate(gray_image: numpy.ndarray, xy: List[float], kind: str) -> Optional[dict]:
    """The marker near xy: {"xy": [x, y], "contrast": c} or None if too weak."""
    fine, coarse, half, min_contrast = KINDS[kind]
    height, width = gray_image.shape
    margin = int(3 * coarse) + 1
    x0 = max(0, int(round(xy[0])) - half - margin)
    y0 = max(0, int(round(xy[1])) - half - margin)
    x1 = min(width, int(round(xy[0])) + half + margin + 1)
    y1 = min(height, int(round(xy[1])) + half + margin + 1)
    window = gray_image[y0:y1, x0:x1]
    if window.shape[0] < 2 * margin + 3 or window.shape[1] < 2 * margin + 3:
        return None
    dog = _blur(window, coarse) - _blur(window, fine)
    inner = dog[margin:-margin, margin:-margin]
    py, px = numpy.unravel_index(numpy.argmax(inner), inner.shape)
    peak = inner[py, px]
    if peak < min_contrast:
        return None
    radius = max(1, int(round(2 * fine)))
    py, px = py + margin, px + margin
    patch = dog[py - radius:py + radius + 1, px - radius:px + radius + 1]
    weights = numpy.clip(patch, 0, None)
    ys, xs = numpy.mgrid[-radius:radius + 1, -radius:radius + 1]
    total = weights.sum()
    cx = px + (xs * weights).sum() / total
    cy = py + (ys * weights).sum() / total
    return {"xy": [float(x0 + cx), float(y0 + cy)], "contrast": float(peak)}


def depth_at(depth_m: numpy.ndarray, xy: List[float]) -> Optional[float]:
    x, y = int(round(xy[0])), int(round(xy[1]))
    h = DEPTH_HALF_WINDOW
    patch = depth_m[max(0, y - h):y + h + 1, max(0, x - h):x + h + 1]
    valid = patch[patch > 0]
    return float(numpy.median(valid)) if valid.size else None


def load_markers(directory: Path) -> Dict[str, dict]:
    data = json.loads((Path(directory) / MARKERS_FILE).read_text(encoding="utf-8"))
    if data.get("format") != MARKERS_FORMAT:
        raise ValueError(f"{MARKERS_FILE} is not a {MARKERS_FORMAT} file")
    markers = {}
    for name, entry in data["markers"].items():
        if entry.get("kind", "dot") not in KINDS:
            raise ValueError(f"marker {name}: unknown kind {entry.get('kind')!r}")
        markers[name] = {"xy": [float(v) for v in entry["xy"]],
                         "kind": entry.get("kind", "dot"),
                         "reference": bool(entry.get("reference", False))}
    return markers


class MarkerTracker:
    """Follows the markers from picture to picture; the first picture tracked
    is the zero for displacements."""

    def __init__(self, markers: Dict[str, dict]) -> None:
        self.markers = {name: dict(entry) for name, entry in markers.items()}
        self.last = {name: list(entry["xy"]) for name, entry in markers.items()}
        self.first: Optional[Dict[str, dict]] = None

    def track(self, rgb: numpy.ndarray, depth_m: Optional[numpy.ndarray] = None,
              deproject=None) -> Dict[str, dict]:
        """Marker -> {"xy", "contrast", "mm" (camera frame, or None)} or
        {"lost": True}; lost markers keep their last position."""
        image = gray(rgb)
        found = {}
        for name, entry in self.markers.items():
            spot = locate(image, self.last[name], entry["kind"])
            if spot is None:
                found[name] = {"lost": True, "xy": list(self.last[name])}
                continue
            self.last[name] = spot["xy"]
            spot["mm"] = None
            if depth_m is not None and deproject is not None:
                depth = depth_at(depth_m, spot["xy"])
                if depth is not None:
                    spot["mm"] = [1000.0 * v for v in deproject(*spot["xy"], depth)]
            found[name] = spot
        if self.first is None:
            self.first = found
        return found

    def displacement(self, found: Dict[str, dict]) -> Dict[str, dict]:
        """Marker -> {"px": [dx, dy], "mm": [dx, dy, dz] or None} since the
        first picture, minus the mean shift of the reference markers."""
        first = self.first or found

        def shift(name, key):
            a, b = first.get(name, {}), found.get(name, {})
            if a.get("lost") or b.get("lost") or a.get(key) is None or b.get(key) is None:
                return None
            return [bv - av for av, bv in zip(a[key], b[key])]

        def reference_shift(key, size):
            shifts = [shift(name, key) for name, entry in self.markers.items()
                      if entry["reference"]]
            shifts = [s for s in shifts if s is not None]
            if not shifts:
                return [0.0] * size
            return [sum(s[i] for s in shifts) / len(shifts) for i in range(size)]

        ref_px, ref_mm = reference_shift("xy", 2), reference_shift("mm", 3)
        out = {}
        for name, entry in self.markers.items():
            px, mm = shift(name, "xy"), shift(name, "mm")
            if not entry["reference"]:
                px = None if px is None else [v - r for v, r in zip(px, ref_px)]
                mm = None if mm is None else [v - r for v, r in zip(mm, ref_mm)]
            out[name] = {"px": px, "mm": mm}
        return out
