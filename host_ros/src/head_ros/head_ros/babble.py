"""Make a motor-babbling trajectory around the viseme library, for lip-sync data.

    scripts/head-babble --minutes 5 --seed 1 --out babble_5min.csv
    scripts/head-face --trajectory babble_5min.csv --camera realsense --camera-mode WxH@FPS \
        --line-speed 1200 --jaw-speed 480

Each target is a library viseme (or rest) plus Gaussian noise on every value
(each side on its own), clipped to the action bounds. The move to it lasts
long enough for every value at the given speeds, then it holds a random
dwell. Blocks of motion are separated by rest pauses (cooling), and the jaw
is kept closed while its open time in the block is over --jaw-duty. The
trajectory starts and ends at rest. Use the same speeds as head-face's
--line-speed/--jaw-speed: the generator only plans for them, the session
enforces them.

This is the speech-relevant babbling of Hu et al. 2026 (Sci. Robot. 11,
eadx3017), around this head's own visemes. The defaults are chosen, not
measured; set them from tests T3/T4 in docs/LIP_SYNC_TEST_PLAN.md.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import random
import sys
from typing import Dict, List, Optional, Sequence, Tuple

try:
    from . import face_stream as fs, mouth_actions as ma, workspace_guard as wg
except ImportError:  # run as a plain script from this directory
    import face_stream as fs  # type: ignore[no-redef]
    import mouth_actions as ma  # type: ignore[no-redef]
    import workspace_guard as wg  # type: ignore[no-redef]

STEP_S = 1.0 / fs.STREAM_HZ
# Ticks for a full unit of each action (the session's ranges by default), so
# that speeds in ticks/s become units/s. v spans up (> 0) and down (< 0): the
# larger of the two ranges is used, so the plan is never faster than allowed.
DEFAULT_UNIT_TICKS = {
    "jaw": wg.JAW_CLOSED_TICK - wg.JAW_OPEN_TICK,
    "right_d": fs.SESSION_RANGES["diag"], "left_d": fs.SESSION_RANGES["diag"],
    "right_v": max(fs.SESSION_RANGES["up"], fs.SESSION_RANGES["down"]),
    "left_v": max(fs.SESSION_RANGES["up"], fs.SESSION_RANGES["down"]),
    "lip_right": fs.UPPER_SESSION_RANGES["lip"], "lip_left": fs.UPPER_SESSION_RANGES["lip"],
    # Upper lips: their span comes from the panel marks; this is a stand-in
    # until they are marked (--unit-ticks upper_lip_right=N overrides it).
    "upper_lip_right": 400, "upper_lip_left": 400,
}
JAW_OPEN_THRESHOLD = 0.3     # the jaw counts as open above this action (chosen)
# The planner uses this fraction of the speed limits, so the session's own
# limit never has to slow a move down (chosen).
SPEED_MARGIN = 0.9


@dataclass
class BabbleSettings:
    minutes: float = 5.0
    seed: int = 1
    sigma: float = 0.15
    dwell_s: Tuple[float, float] = (0.08, 0.30)
    block_s: float = 60.0
    pause_s: float = 30.0
    jaw_duty: float = 0.5
    line_speed_tps: float = fs.LINE_SPEED_TPS
    jaw_speed_tps: float = fs.JAW_SPEED_TPS
    rest_weight: float = 1.0
    upper_lips: bool = False


def move_time_s(a: ma.Vector, b: ma.Vector, settings: BabbleSettings,
                unit_ticks: Dict[str, float]) -> float:
    """The shortest move from a to b at SPEED_MARGIN x the speed limits."""
    longest = 0.0
    for name in ma.MOUTH_COLUMNS:
        speed = settings.jaw_speed_tps if name == "jaw" else settings.line_speed_tps
        ticks = abs(b[name] - a[name]) * unit_ticks[name]
        longest = max(longest, ticks / (SPEED_MARGIN * speed))
    return longest


def open_part(a: float, b: float, threshold: float = JAW_OPEN_THRESHOLD) -> float:
    """The part of a linear jaw move from a to b spent over the threshold."""
    low, high = min(a, b), max(a, b)
    if high <= threshold:
        return 0.0
    if low > threshold:
        return 1.0
    return (high - threshold) / (high - low)


def generate(library: Dict[str, ma.Vector], settings: BabbleSettings,
             unit_ticks: Optional[Dict[str, float]] = None) -> List[ma.Row]:
    """The trajectory rows (one per STEP_S)."""
    unit_ticks = {**DEFAULT_UNIT_TICKS, **(unit_ticks or {})}
    if not 0 < settings.minutes <= ma.MAX_DURATION_S / 60:
        raise ValueError(f"minutes must be over 0 and at most {ma.MAX_DURATION_S / 60:.0f}")
    if not 0 <= settings.sigma <= 1 or not 0 < settings.dwell_s[0] <= settings.dwell_s[1]:
        raise ValueError("need 0 <= sigma <= 1 and 0 < dwell min <= dwell max")
    if not 0 < settings.jaw_duty <= 1 or settings.block_s <= 0 or settings.pause_s < 0:
        raise ValueError("need 0 < jaw_duty <= 1, block_s > 0, pause_s >= 0")
    rng = random.Random(settings.seed)
    names = sorted(library)
    choices = names + ["rest"]
    weights = [1.0] * len(names) + [settings.rest_weight]
    keys: List[Tuple[float, ma.Vector]] = [(0.0, ma.rest()), (1.0, ma.rest())]   # 1 s rest first
    t = 1.0
    total_motion_s = settings.minutes * 60.0
    motion_s = 0.0
    while motion_s < total_motion_s:
        block_end = min(settings.block_s, total_motion_s - motion_s)
        block_t = 0.0
        jaw_open_s = 0.0
        current = keys[-1][1]
        while block_t < block_end:
            name = rng.choices(choices, weights)[0]
            base = ma.rest() if name == "rest" else library[name]
            target = ma.clip_vector({k: base[k] + rng.gauss(0.0, settings.sigma) for k in ma.MOUTH_COLUMNS})
            if not settings.upper_lips:
                target["upper_lip_right"] = target["upper_lip_left"] = 0.0
            if block_t > 0 and jaw_open_s / block_t > settings.jaw_duty:
                target["jaw"] = 0.0
            move = max(STEP_S, move_time_s(current, target, settings, unit_ticks))
            dwell = rng.uniform(*settings.dwell_s)
            t += move
            keys.append((t, target))
            t += dwell
            keys.append((t, target))
            jaw_open_s += move * open_part(current["jaw"], target["jaw"])
            if target["jaw"] > JAW_OPEN_THRESHOLD:
                jaw_open_s += dwell
            block_t += move + dwell
            current = target
        motion_s += block_t
        # Back to rest, then the pause.
        move = max(STEP_S, move_time_s(current, ma.rest(), settings, unit_ticks))
        t += move
        keys.append((t, ma.rest()))
        t += settings.pause_s if motion_s < total_motion_s else 1.0
        keys.append((t, ma.rest()))
    return resample(keys)


def resample(keys: Sequence[Tuple[float, ma.Vector]]) -> List[ma.Row]:
    """Rows every STEP_S, linear between the keys."""
    unique: List[Tuple[float, ma.Vector]] = []
    for t_s, values in keys:
        if unique and t_s <= unique[-1][0] + 1e-9:
            unique[-1] = (unique[-1][0], values)
        else:
            unique.append((t_s, values))
    rows: List[ma.Row] = []
    end = unique[-1][0]
    steps = int(math.floor(end / STEP_S + 1e-9))
    for k in range(steps + 1):
        t_s = round(k * STEP_S, 6)
        rows.append((t_s, ma.clip_vector(ma.sample(unique, t_s) or unique[-1][1])))
    return rows


def jaw_open_fraction(rows: Sequence[ma.Row]) -> float:
    return sum(1 for _t, v in rows if v["jaw"] > JAW_OPEN_THRESHOLD) / max(1, len(rows))


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--records", type=Path, default=fs.DEFAULT_RECORDS)
    parser.add_argument("--preview", action="store_true", help="use the preview records' library")
    parser.add_argument("--library", type=Path, help="viseme library (default: config/viseme_library.json)")
    parser.add_argument("--out", type=Path, required=True, help="trajectory CSV to write")
    parser.add_argument("--minutes", type=float, default=5.0, help="motion time, pauses not counted")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--sigma", type=float, default=0.15, help="noise per value (action units)")
    parser.add_argument("--dwell-ms", default="80:300", metavar="MIN:MAX")
    parser.add_argument("--block-s", type=float, default=60.0)
    parser.add_argument("--pause-s", type=float, default=30.0)
    parser.add_argument("--jaw-duty", type=float, default=0.5,
                        help=f"most time per block with the jaw over {JAW_OPEN_THRESHOLD}")
    parser.add_argument("--line-speed", type=float, default=fs.LINE_SPEED_TPS)
    parser.add_argument("--jaw-speed", type=float, default=fs.JAW_SPEED_TPS)
    parser.add_argument("--rest-weight", type=float, default=1.0,
                        help="how often rest is picked compared with one viseme")
    parser.add_argument("--upper-lips", action="store_true",
                        help="babble the upper lips too (only once they are marked)")
    parser.add_argument("--unit-ticks", action="append", default=[], metavar="NAME=TICKS",
                        help="ticks per action unit (default: the session ranges)")
    a = parser.parse_args(argv)
    try:
        low, _, high = a.dwell_ms.partition(":")
        dwell = (float(low) / 1000.0, float(high) / 1000.0)
        unit_ticks = {}
        for item in a.unit_ticks:
            name, _, ticks = item.partition("=")
            if name not in ma.MOUTH_COLUMNS or float(ticks) <= 0:
                raise ValueError(f"bad --unit-ticks {item!r}")
            unit_ticks[name] = float(ticks)
        records = a.records / "preview" if a.preview else a.records
        path = a.library or records / "config" / "viseme_library.json"
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if data.get("format") != "head-viseme-library-1":
            raise ValueError(f"{path} is not a viseme library")
        library = {name: ma.check_vector(entry["pose"]) for name, entry in data["visemes"].items()}
        if not library:
            raise ValueError(f"{path} has no visemes")
        settings = BabbleSettings(a.minutes, a.seed, a.sigma, dwell, a.block_s, a.pause_s, a.jaw_duty,
                                  a.line_speed, a.jaw_speed, a.rest_weight, a.upper_lips)
        rows = generate(library, settings, unit_ticks)
    except (OSError, ValueError, KeyError) as error:
        print(error); return 2
    metadata = {"made_by": "babble", "seed": a.seed, "sigma": a.sigma, "library": str(path),
                "line_speed_tps": a.line_speed, "jaw_speed_tps": a.jaw_speed}
    ma.write_trajectory(a.out, rows, metadata)
    print(f"{a.out}: {rows[-1][0]:.0f} s, {len(rows)} rows, {len(library)} visemes, jaw open "
          f"{100 * jaw_open_fraction(rows):.0f} % of the time. Run it with head-face --trajectory "
          f"{a.out} --line-speed {a.line_speed:g} --jaw-speed {a.jaw_speed:g}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
