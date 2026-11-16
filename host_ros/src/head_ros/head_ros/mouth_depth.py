"""Mouth opening in millimetres with the RealSense depth camera.

    scripts/head-mouthdepth --neutrals tuning_runs/runs/<date>/<time>_knee/neutrals.json \
        --jaw-release 8=1250 --jaw-release 9=1250
    scripts/head-mouthdepth --analyze RUN [--x PX] [--y0 PY --y1 PY]

run: one face session (face_stream.FaceSession: envelope, guards, the 52 C
stop) that holds the jaw closed, then fully open (j = 1, --jaw-open), three
times (HOLDS), corners at rest. A background thread reads the camera (color
with depth aligned to it) all the time; at the end of every closed and every
open hold the newest frame set is saved as snapshots/<n>_<closed|open>.npz
(rgb, depth in metres, camera intrinsics, host time) and a JPEG.

--analyze: along a vertical band at the mouth centre (--x, default: the
middle of the darkest band found in the open frame), the dark lip slit is
found by brightness (rows darker than DARK_FRACTION of the band's median).
Its top and bottom rows become 3-D points with the depth of the lips just
above and below the slit (median over a small window), so the gap is in mm.
The lower lip's drop is the 3-D distance between the bottom edge of the slit
when open and the lip line when closed. Writes mouth_depth.json.

The brightness rule and the window sizes are chosen; check the JPEGs.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

try:
    from . import face_stream as fs, records_layout, tuning_core
except ImportError:  # run as a plain script from this directory
    import face_stream as fs  # type: ignore[no-redef]
    import records_layout  # type: ignore[no-redef]
    import tuning_core  # type: ignore[no-redef]

CLOSED_S, OPEN_S, HOLDS = 2.0, 0.9, 3
LEAD_S = 1.0
SNAP_BEFORE_END_S = 0.15        # a snapshot this long before a hold ends
MAX_FRAME_AGE_S = 0.4           # older frames are not used for a snapshot
DARK_FRACTION = 0.55            # slit rows: darker than this x the band median
BAND_HALF_PX = 6                # half width of the vertical band (pixels)
DEPTH_WINDOW_PX = 4             # depth median window around an edge point
LIP_OFFSET_PX = 6               # lips sampled this far above/below the slit
SAME_SURFACE_M = 0.010          # edge depths further apart: one is not on the lip


def schedule() -> List[Tuple[float, float, str]]:
    """(start s, end s, 'closed'|'open') for every hold, after LEAD_S."""
    out, t = [], LEAD_S
    for _n in range(HOLDS):
        out.append((t, t + CLOSED_S, "closed")); t += CLOSED_S
        out.append((t, t + OPEN_S, "open")); t += OPEN_S
    out.append((t, t + CLOSED_S, "closed"))
    return out


class DepthGrabber:
    """Reads color + aligned depth in a thread; latest() gives the newest set."""

    def __init__(self, camera) -> None:
        self.camera = camera
        self._latest = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.error: Optional[str] = None
        self._thread = threading.Thread(target=self._run, daemon=True, name="depth-grabber")
        self._thread.start()

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                rgb, depth = self.camera.capture()
                with self._lock:
                    self._latest = (time.monotonic(), datetime.now(timezone.utc).isoformat(), rgb, depth)
        except Exception as error:          # the session goes on without depth
            self.error = str(error)

    def latest(self):
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)
        self.camera.close()


def save_snapshot(path: Path, frame, camera) -> None:
    """Write one snapshot (slow: several MB). Never call it from the control
    loop: on 2026-11-11 (run 175728) a save there stopped the command stream
    and the board latched a WATCHDOG fault. SnapshotWriter does it in a thread."""
    import numpy
    from PIL import Image

    _mono, utc, rgb, depth = frame
    k = camera.intrinsics
    path.parent.mkdir(parents=True, exist_ok=True)
    depth_mm = numpy.round(depth * 1000.0).astype(numpy.uint16)
    numpy.savez(path.with_suffix(".npz"), rgb=rgb, depth_mm=depth_mm, utc=utc,
                intrinsics=numpy.array([k.width, k.height, k.ppx, k.ppy, k.fx, k.fy]))
    Image.fromarray(rgb).save(path.with_suffix(".jpg"), quality=92)


class SnapshotWriter:
    """Saves snapshots in its own thread; put() only queues the frame."""

    def __init__(self, camera) -> None:
        import queue

        self.camera = camera
        self._queue: "queue.Queue" = queue.Queue()
        self.errors: List[str] = []
        self._thread = threading.Thread(target=self._run, daemon=True, name="snapshot-writer")
        self._thread.start()

    def put(self, path: Path, frame) -> None:
        self._queue.put((path, frame))

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            if item is None:
                return
            try:
                save_snapshot(item[0], item[1], self.camera)
            except Exception as error:
                self.errors.append(f"{item[0].name}: {error}")

    def close(self) -> None:
        self._queue.put(None)
        self._thread.join(timeout=30.0)


def deproject(intr, x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Camera-frame point (metres) of pixel (x, y) at depth z with the saved
    pinhole intrinsics (width, height, ppx, ppy, fx, fy); lens distortion is
    not applied."""
    _w, _h, ppx, ppy, fx, fy = intr
    return ((x - ppx) / fx * z, (y - ppy) / fy * z, z)


def depth_at(depth, x: int, y: int, half: int = DEPTH_WINDOW_PX) -> Optional[float]:
    import numpy

    patch = depth[max(0, y - half):y + half + 1, max(0, x - half):x + half + 1]
    vals = patch[patch > 0]
    return float(numpy.median(vals)) if vals.size else None


def find_slit(rgb, x: int, y0: int, y1: int) -> Optional[Tuple[int, int]]:
    """(top row, bottom row) of the dark slit in the band around column x
    between rows y0 and y1, or None."""
    import numpy

    band = rgb[y0:y1, max(0, x - BAND_HALF_PX):x + BAND_HALF_PX + 1].astype(numpy.float32).mean(axis=(1, 2))
    med = float(numpy.median(band))
    dark = numpy.where(band < DARK_FRACTION * med)[0]
    if dark.size == 0:
        return None
    # The longest run of dark rows.
    runs, start = [], dark[0]
    for a, b in zip(dark, dark[1:]):
        if b != a + 1:
            runs.append((start, a)); start = b
    runs.append((start, dark[-1]))
    top, bottom = max(runs, key=lambda r: r[1] - r[0])
    return int(y0 + top), int(y0 + bottom)


def analyze(run: Path, x: Optional[int], y0: Optional[int], y1: Optional[int]) -> dict:
    import numpy

    snaps = sorted((run / "snapshots").glob("*.npz"))
    if not snaps:
        raise ValueError(f"no snapshots in {run}")
    data = {p.stem: numpy.load(p, allow_pickle=False) for p in snaps}
    first_open = next(d for n, d in data.items() if n.endswith("open"))
    h, w = first_open["rgb"].shape[:2]
    y0 = y0 if y0 is not None else int(h * 0.45)
    y1 = y1 if y1 is not None else int(h * 0.85)
    if x is None:
        # The column whose band is darkest in the open frame (the slit centre).
        gray = first_open["rgb"].astype(numpy.float32).mean(axis=2)[y0:y1]
        cols = gray.min(axis=0)
        x = int(numpy.argmin(numpy.convolve(cols, numpy.ones(25) / 25, mode="same")))
    out = {"run": str(run), "x": x, "rows": [y0, y1], "snapshots": {}}
    closed_lines = []
    for name, d in data.items():
        rgb, intr = d["rgb"], d["intrinsics"]
        depth = d["depth_mm"].astype(numpy.float32) / 1000.0
        slit = find_slit(rgb, x, y0, y1)
        entry: Dict[str, object] = {"slit_rows": slit}
        if slit is not None:
            top, bottom = slit
            z_up = depth_at(depth, x, top - LIP_OFFSET_PX)
            z_lo = depth_at(depth, x, bottom + LIP_OFFSET_PX)
            if z_up and z_lo:
                # Both edges lie on the lip surface. When one sample lands in
                # the mouth opening or a depth hole it reads deeper (run
                # 180418: 0.362 vs 0.305 m, a 60 mm "gap"); then the nearer
                # (lip) depth is used for both.
                if abs(z_up - z_lo) > SAME_SURFACE_M:
                    z_up = z_lo = min(z_up, z_lo)
                p_top = deproject(intr, x, top, z_up)
                p_bot = deproject(intr, x, bottom, z_lo)
                entry["gap_mm"] = round(1000 * math.dist(p_top, p_bot), 1)
                entry["upper_edge_m"] = [round(v, 5) for v in p_top]
                entry["lower_edge_m"] = [round(v, 5) for v in p_bot]
                entry["depth_m"] = [round(z_up, 4), round(z_lo, 4)]
                if name.endswith("closed"):
                    closed_lines.append(p_bot)
        out["snapshots"][name] = entry
    if closed_lines:
        ref = tuple(float(numpy.mean([p[i] for p in closed_lines])) for i in range(3))
        for name, entry in out["snapshots"].items():
            if name.endswith("open") and "lower_edge_m" in entry:
                entry["lower_lip_drop_mm"] = round(1000 * math.dist(ref, entry["lower_edge_m"]), 1)
    opens = [e for n, e in out["snapshots"].items() if n.endswith("open") and "gap_mm" in e]
    closes = [e for n, e in out["snapshots"].items() if n.endswith("closed") and "gap_mm" in e]
    out["open_gap_mm"] = [e["gap_mm"] for e in opens]
    out["closed_gap_mm"] = [e["gap_mm"] for e in closes]
    out["lower_lip_drop_mm"] = [e.get("lower_lip_drop_mm") for e in opens]
    return out


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    fs.add_session_arguments(parser)
    parser.set_defaults(line_speed=1000.0, jaw_speed=4000.0, jaw_current=650, jaw_open=3700,
                        jaw_peak_s=1.0, retry_enable_bus_fault=True)
    parser.add_argument("--lip", type=float, default=0.0,
                        help="lower-lip action (servos 3/4, 0-1 of their session span) held "
                             "during the whole run, e.g. 0.125 = 50 of 400 ticks")
    parser.add_argument("--analyze", type=Path, metavar="RUN", help="measure a finished run")
    parser.add_argument("--x", type=int, help="mouth centre column (pixels) for --analyze")
    parser.add_argument("--y0", type=int, help="first row searched for the slit")
    parser.add_argument("--y1", type=int, help="last row searched for the slit")
    a = parser.parse_args(argv)
    if a.analyze:
        try:
            result = analyze(a.analyze, a.x, a.y0, a.y1)
        except (ValueError, OSError) as error:
            print(error); return 2
        (a.analyze / "mouth_depth.json").write_text(json.dumps(result, indent=1) + "\n")
        print(json.dumps({k: result[k] for k in ("x", "rows", "open_gap_mm", "closed_gap_mm",
                                                  "lower_lip_drop_mm")}))
        return 0
    if a.preview:
        print("mouth_depth needs the real camera; use --analyze on a recorded run."); return 2
    if not 0.0 <= a.lip <= 1.0:
        print("--lip must be 0 to 1."); return 2
    say = lambda text: print(text, flush=True)  # noqa: E731
    a.camera = "none"                       # this script owns the camera (with depth)
    try:
        from .face_camera import RealSenseCamera
    except ImportError:
        from face_camera import RealSenseCamera  # type: ignore[no-redef]
    try:
        setup = fs.open_session(a, "face", "mouthdepth", say)
    except fs.SetupError as error:
        print(error); return 2
    camera = RealSenseCamera(depth=True)
    grabber = DepthGrabber(camera)
    writer = SnapshotWriter(camera)
    session = setup.session
    plan = schedule()
    end_s = plan[-1][1]
    marks = [(b - SNAP_BEFORE_END_S, f"{n:02d}_{kind}") for n, (a_, b, kind) in enumerate(plan)]
    session.log["mouth_depth"] = {"schedule": plan, "snapshots": [], "lip": a.lip}
    snaps_dir = setup.run / "snapshots"

    def take(t_s: float) -> None:
        while marks and t_s >= marks[0][0]:
            _t, label = marks.pop(0)
            frame = grabber.latest()
            if frame is None or time.monotonic() - frame[0] > MAX_FRAME_AGE_S:
                session.log["mouth_depth"]["snapshots"].append({"label": label, "missing": True})
                continue
            writer.put(snaps_dir / label, frame)
            session.log["mouth_depth"]["snapshots"].append({"label": label, "utc": frame[1],
                                                            "script_s": round(t_s, 3)})
    session.on_update = take

    def script(t: float):
        if t > end_s:
            return None
        j = next((1.0 if kind == "open" else 0.0 for s, e, kind in plan if s <= t < e), 0.0)
        actions = {"right": (0.0, 0.0), "left": (0.0, 0.0)}
        if a.lip > 0.0:
            actions["lip"] = (a.lip, a.lip)
        return actions, j
    try:
        try:
            session.start()
        except RuntimeError as error:
            session.finish(f"not started: {error}")
            return 2
        say(f"Enabled; {HOLDS} jaw holds (open {OPEN_S} s), snapshots with depth.")
        reason = session.run(script)
        return 0 if reason == "finished" else 1
    finally:
        writer.close()
        grabber.close()
        if writer.errors:
            session.log["mouth_depth"]["save_errors"] = writer.errors
        if grabber.error:
            session.log["mouth_depth"]["camera_error"] = grabber.error
        fs.close_session(setup)


if __name__ == "__main__":
    sys.exit(main())
