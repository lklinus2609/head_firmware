"""Jaw x lip-corner coupling probe: a grid of corner poses at jaw openings.

    scripts/head-jawcorner run --neutrals <knee>/neutrals.json --camera realsense [head-face args]
    scripts/head-jawcorner report tuning_runs/runs/<date>/<time>_face_frames

`run` starts head-face with keyframes: for each corner pose (both sides the
same), the jaw goes 0 -> 0.5 -> 1 -> 0 with a hold at each step, then the
face rests. Every head-face guard applies (line abort 280 mA, magnet-drop
stop, jaw guard, temperature stop, session envelope); nothing new moves.

`report` reads the run's commands.csv, telemetry.csv and record.json and
gives, for every hold, the mean over its last REPORT_WINDOW_S of each line's
position (ticks from its closed-jaw neutral, in its pull direction: + is
pulled in, - is paid out) and load, and the jaw's position and load. It then
derives what the ML workspace needs:
- passive pay-out: how far each down line (a follower in a smile) is let out
  by load when the jaw opens -> the --jaw-release value for that line;
- conflict: how much a DRIVING down line's load rises when the jaw opens
  (the jaw pulling against it), and the jaw's own load, per pose;
- hysteresis: the line positions back at jaw 0 vs before the opening.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import sys
from typing import Dict, List, Optional, Sequence, Tuple

try:
    from . import face_stream as fs, workspace_guard as wg
except ImportError:  # run as a plain script from this directory
    import face_stream as fs  # type: ignore[no-redef]
    import workspace_guard as wg  # type: ignore[no-redef]

# (name, d, v): both sides get the same point (workspace_guard action space).
POSES = (("rest", 0.0, 0.0), ("smile_half", 0.5, 0.5), ("smile_full", 1.0, 1.0),
         ("up_only", 0.0, 1.0), ("diag_only", 1.0, 0.0), ("down_half", 0.0, -0.5),
         ("down_full", 0.0, -1.0))
JAW_STEPS = (0.0, 0.5, 1.0, 0.0)
HOLD_S = 2.0
REST_S = 1.5
REPORT_WINDOW_S = 0.6
MIN_SEGMENT_S = 1.2
ACTION_COLUMNS = ("jaw", "right_d", "right_v", "left_d", "left_v")


def frames(poses: Sequence[Tuple[str, float, float]] = POSES, hold_s: float = HOLD_S,
           rest_s: float = REST_S) -> List[str]:
    out = [f"hold={rest_s}"]
    for _name, d, v in poses:
        for j in JAW_STEPS:
            out.append(f"hold={hold_s} right={d},{v} left={d},{v} jaw={j}")
        out.append(f"hold={rest_s}")
    return out


def pose_name(d: float, v: float) -> str:
    for name, pd, pv in POSES:
        if abs(pd - d) < 1e-3 and abs(pv - v) < 1e-3:
            return name
    return f"d{d:g}_v{v:g}"


def _utc(text: str) -> float:
    return datetime.fromisoformat(text).timestamp()


def segments(commands: Path) -> List[dict]:
    """Holds of constant actions: {actions, start, end} (host UTC seconds)."""
    out: List[dict] = []
    with open(commands, encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            key = tuple(round(float(row[c]), 3) for c in ACTION_COLUMNS)
            t = _utc(row["host_utc"])
            if out and out[-1]["key"] == key:
                out[-1]["end"] = t
            else:
                out.append({"key": key, "start": t, "end": t})
    return [s for s in out if s["end"] - s["start"] >= MIN_SEGMENT_S]


def telemetry_means(path: Path, windows: List[Tuple[float, float]]) -> List[Dict[int, dict]]:
    sums: List[Dict[int, List[float]]] = [dict() for _ in windows]
    with open(path, encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            t = _utc(row["host_utc"])
            for k, (a, b) in enumerate(windows):
                if a <= t <= b:
                    acc = sums[k].setdefault(int(row["servo"]), [0.0, 0.0, 0.0, 0.0, -1e9])
                    acc[0] += float(row["present_tick"]); acc[1] += float(row["current_ma"])
                    acc[2] += 1; acc[3] += float(row["goal_tick"])
                    acc[4] = max(acc[4], float(row["temperature_c"]))
    out = []
    for acc in sums:
        out.append({i: {"present": a[0] / a[2], "current_ma": a[1] / a[2], "goal": a[3] / a[2],
                        "max_temp_c": a[4]} for i, a in acc.items() if a[2]})
    return out


def report(run: Path) -> dict:
    record = json.loads((run / "record.json").read_text())
    neutral = {int(i): t for i, t in record["neutrals_used"]["closed"].items()}
    segs = segments(run / "commands.csv")
    windows = [(s["end"] - REPORT_WINDOW_S, s["end"]) for s in segs]
    means = telemetry_means(run / "telemetry.csv", windows)
    holds = []
    for seg, m in zip(segs, means):
        jaw, rd, rv, ld, lv = seg["key"]
        if wg.JAW not in m:
            continue
        lines = {}
        for i in wg.PULL:
            if i in m:
                lines[str(i)] = {"pulled": round(wg.PULL[i] * (m[i]["present"] - neutral[i]), 1),
                                 "load_ma": round(m[i]["current_ma"] * wg.PULL[i], 1),
                                 "max_temp_c": m[i]["max_temp_c"]}
        holds.append({"pose": pose_name(rd, rv) if (rd, rv) == (ld, lv) else f"R{rd},{rv}_L{ld},{lv}",
                      "jaw_cmd": jaw, "jaw_tick": round(m[wg.JAW]["present"], 1),
                      "jaw_ma": round(m[wg.JAW]["current_ma"], 1),
                      "jaw_temp_c": m[wg.JAW]["max_temp_c"], "lines": lines})
    derived = derive(holds)
    out = {"run": str(run), "stopped": record.get("stopped"),
           "jaw_release_ticks": record.get("jaw_release_ticks"),
           "max_temperature_c": record.get("max_temperature_c"),
           "window_s": REPORT_WINDOW_S, "holds": holds, "derived": derived}
    return out


def derive(holds: List[dict]) -> dict:
    """Per pose: the change from jaw 0 (before) to jaw 1 of each line's
    position and load, the jaw at 1, and the change back at jaw 0 (after)."""
    by_pose: Dict[str, List[dict]] = {}
    for hold in holds:
        by_pose.setdefault(hold["pose"], []).append(hold)
    out = {}
    for pose, hs in by_pose.items():
        zero = [h for h in hs if h["jaw_cmd"] == 0.0]
        full = [h for h in hs if h["jaw_cmd"] == 1.0]
        if not zero or not full:
            continue
        before, opened = zero[0], full[0]
        after = zero[1] if len(zero) > 1 else None
        entry = {"jaw_opening_ticks": round(before["jaw_tick"] - opened["jaw_tick"], 1),
                 "jaw_ma_open": opened["jaw_ma"], "lines": {}}
        for i, line in opened["lines"].items():
            b = before["lines"].get(i)
            if b is None:
                continue
            e = {"moved_with_jaw": round(line["pulled"] - b["pulled"], 1),
                 "load_change_ma": round(line["load_ma"] - b["load_ma"], 1),
                 "load_open_ma": line["load_ma"]}
            if after is not None and i in after["lines"]:
                e["back_at_jaw0"] = round(after["lines"][i]["pulled"] - b["pulled"], 1)
            entry["lines"][i] = e
        out[pose] = entry
    return out


def print_report(data: dict) -> None:
    lines = [str(i) for i in wg.PULL]
    print(f"Run {data['run']} (stopped: {data['stopped']}; jaw release {data['jaw_release_ticks']})")
    print("pose        jaw  jaw_tick jaw_mA | " + " ".join(f"{i:>11}" for i in lines)
          + "   (pulled ticks / load mA)")
    for h in data["holds"]:
        cells = []
        for i in lines:
            c = h["lines"].get(i)
            cells.append(f"{c['pulled']:>6.0f}/{c['load_ma']:>4.0f}" if c else " " * 11)
        print(f"{h['pose']:<11} {h['jaw_cmd']:<4} {h['jaw_tick']:>8.0f} {h['jaw_ma']:>6.0f} | "
              + " ".join(cells))
    print("\nJaw 0 -> 1 per pose: line moved (ticks, - = paid out) / load change (mA); back at jaw 0")
    for pose, e in data["derived"].items():
        cells = []
        for i in lines:
            c = e["lines"].get(i)
            cells.append(f"{i}:{c['moved_with_jaw']:+.0f}/{c['load_change_ma']:+.0f}"
                         + (f"({c['back_at_jaw0']:+.0f})" if "back_at_jaw0" in c else "") if c else "")
        print(f"  {pose:<11} jaw opened {e['jaw_opening_ticks']:.0f} ticks at {e['jaw_ma_open']:.0f} mA: "
              + "  ".join(cells))
    print(f"Max temperature: {data['max_temperature_c']}")


def main(argv: Optional[List[str]] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] not in ("run", "report", "frames"):
        print(__doc__); return 2
    command, rest = argv[0], argv[1:]
    if command == "frames":
        for frame in frames():
            print(frame)
        return 0
    if command == "report":
        parser = argparse.ArgumentParser(prog="head-jawcorner report")
        parser.add_argument("run", type=Path)
        parser.add_argument("--json", action="store_true", help="print JSON only")
        a = parser.parse_args(rest)
        data = report(a.run)
        (a.run / "jaw_corner_report.json").write_text(json.dumps(data, indent=1))
        if a.json:
            print(json.dumps(data, indent=1))
        else:
            print_report(data)
            print(f"Saved: {a.run / 'jaw_corner_report.json'}")
        return 0
    if any(arg.startswith("--frame") or arg.startswith("--demo") or arg.startswith("--trajectory")
           for arg in rest):
        print("run makes its own keyframes; do not give --frame, --demo or --trajectory"); return 2
    face_argv = list(rest)
    for frame in frames():
        face_argv += ["--frame", frame]
    return fs.main(face_argv)


if __name__ == "__main__":
    sys.exit(main())
