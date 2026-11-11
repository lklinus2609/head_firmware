"""The commandable safe workspace of the face, for ML policies and babbling.

    python3 face_workspace.py --write tuning_runs/config/face_workspace.json
    python3 face_workspace.py --show tuning_runs/config/face_workspace.json

A model outputs actions; project() turns any action into one inside the
workspace that was measured safe on the head on 2026-11-10
(firmware/docs/NATURAL_FACE_AND_JAW_WORKSPACE.md). The session
(face_stream.FaceSession) still enforces its own limits on top; this layer
makes the model's commands ones the head can actually carry out, so the
model never learns from clamped or refused commands.

Action dimensions (all unitless; the session maps them to ticks):
    right, left: (d, v) per side   d 0..1 diag, v -1..1 (v < 0: down drives)
    jaw: j 0..1                    0 closed, 1 the session's jaw goal (3700)
    brow, oculi, lip, upper_lip: (right, left) 0..1
    gaze: (x, y) -1..1             natural_face.EyeLids mapping
    lids: (right, left) 0..1       fraction of the closed mark

Rules (hard, not learned):
1. Ranges: each value is clipped to its bounds.
2. Jaw budget (JawBudget): above jaw_static_max the jaw draws more current
   than it can hold (the jaw is P-only and the skin is stiff: goal 3880 held
   3952 at 250-282 mA, under the 400 mA / 2 s guard). Above it the jaw is
   allowed only in pulses of at most jaw_pulse_max_s and a duty of at most
   jaw_duty_max in any jaw_window_s. These are the limits of the tested
   talking (3 x 120 s: pulses <= 0.18 s, 10 s duty <= 0.40; jaw +3-4 C per
   2 min, 46 C at most), not a thermal model; the 52 C stop stays the last
   guard.
3. Down release with the jaw: a down line's neutral moves toward slack by
   jaw_release_ticks[line] x the jaw's measured opening fraction (the session
   does it with --jaw-release LINE=TICKS; release_args() gives the flags).
4. Down never pulls against up/diag: the session's guard does it (v sign).
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
import sys
from typing import Deque, Dict, List, Optional, Tuple

SPEC_FORMAT = "head-face-workspace-1"


@dataclass
class WorkspaceSpec:
    measured: str = "2026-11-10, new head prototype, knee neutrals 191452"
    # Session settings the numbers below hold for.
    session_ranges_ticks: Dict[str, int] = field(
        default_factory=lambda: {"up": 350, "diag": 290, "down": 150})
    jaw_open_goal_tick: int = 3700
    jaw_closed_tick: int = 4000
    jaw_current_cap_ma: int = 650
    # Jaw budget (rule 2).
    jaw_static_max: float = 0.4              # goal 3880 = (4000 - 3880) / (4000 - 3700)
    jaw_pulse_max_s: float = 0.2
    jaw_duty_max: float = 0.4
    jaw_window_s: float = 10.0
    # Jaw goal -> reached opening, for the model's map: (goal tick, opening
    # ticks below closed, current mA). 3939 and 3880: held 2 s (probes 200042,
    # 200431); 3800 and 3700: deepest point of 0.18-0.32 s syllables (201327,
    # 201601-203145). The jaw is P-only: it settles short of its goal by its load.
    jaw_reached: List[Tuple[int, int, int]] = field(default_factory=lambda: [
        (3939, 27, 124), (3880, 48, 260), (3800, 78, 460), (3700, 114, 640)])
    # Rule 3, ticks at the full jaw goal span (corners held down, talking).
    jaw_release_ticks: Dict[str, int] = field(default_factory=lambda: {"8": 30, "9": 15})
    # Speeds (ticks/s) and accelerations (ticks/s^2) tested clean.
    speeds_tps: Dict[str, float] = field(default_factory=lambda: {
        "lines": 1000.0, "lines_max_with_oculi": 2500.0, "jaw": 4000.0, "eyes": 2500.0,
        "lid_close": 5000.0, "lid_open": 3500.0})
    accel_tps2: Dict[str, float] = field(default_factory=lambda: {"eyes": 40000.0, "lids": 80000.0})
    blink_depth: Dict[str, float] = field(default_factory=lambda: {"5": 0.55, "6": 1.0})
    abort_ma: Dict[str, int] = field(default_factory=lambda: {"lines": 280, "eyes_lids": 280})
    temperature_stop_c: int = 52
    # Measured performance on that session.
    measured_performance: Dict[str, str] = field(default_factory=lambda: {
        "blink": "close 0.36 s, close+open 0.82-0.90 s (lid peaks 222/254 mA)",
        "saccade": "0.10-0.63 s, median 0.22-0.29 s",
        "jaw_dynamic": "110-118 ticks open in 0.18-0.32 s syllables at 600-650 mA peaks",
        "heat": "jaw +3-4 C per 120 s talking, 46 C after 3 runs; others <= 43 C"})

    def to_json(self) -> dict:
        return {"format": SPEC_FORMAT, **asdict(self)}

    @classmethod
    def from_json(cls, data: dict) -> "WorkspaceSpec":
        if data.get("format") != SPEC_FORMAT:
            raise ValueError(f"not a {SPEC_FORMAT} file")
        known = {k: v for k, v in data.items() if k in cls.__dataclass_fields__}
        spec = cls(**known)
        spec.jaw_reached = [tuple(x) for x in spec.jaw_reached]
        return spec

    def release_args(self) -> List[str]:
        """--jaw-release flags for head-face / head-natural."""
        out: List[str] = []
        for line, ticks in sorted(self.jaw_release_ticks.items()):
            out += ["--jaw-release", f"{line}={ticks}"]
        return out


def _clip(x: float, low: float, high: float) -> float:
    return low if x < low else high if x > high else x


class JawBudget:
    """Rule 2: lets j above spec.jaw_static_max only in short pulses with a
    limited duty. limit(j, t_s) returns the allowed j; call it once per
    update with increasing t_s."""

    def __init__(self, spec: WorkspaceSpec) -> None:
        self.spec = spec
        self._over_since: Optional[float] = None
        self._samples: Deque[Tuple[float, bool]] = deque()
        self._spent = False

    def duty(self) -> float:
        if len(self._samples) < 2:
            return 0.0
        over = 0.0
        for (t0, flag), (t1, _f) in zip(self._samples, list(self._samples)[1:]):
            if flag:
                over += t1 - t0
        return over / max(1e-9, self.spec.jaw_window_s)

    def limit(self, j: float, t_s: float) -> float:
        s = self.spec
        j = _clip(j, 0.0, 1.0)
        while self._samples and self._samples[0][0] < t_s - s.jaw_window_s:
            self._samples.popleft()
        allowed = j
        if j <= s.jaw_static_max:
            # The command came down: the next pulse may start.
            self._over_since, self._spent = None, False
        elif self._spent or self.duty() >= s.jaw_duty_max:
            allowed = s.jaw_static_max
        else:
            if self._over_since is None:
                self._over_since = t_s
            if t_s - self._over_since >= s.jaw_pulse_max_s:
                # Pulse used up: the static limit until the command comes
                # back down below it (one pulse = one syllable).
                self._spent = True
                allowed = s.jaw_static_max
        self._samples.append((t_s, allowed > s.jaw_static_max))
        return allowed


SIDE_KEYS = ("right", "left")
PAIR_KEYS = ("brow", "oculi", "lip", "upper_lip", "lids")


def project(action: Dict[str, object], budget: JawBudget, t_s: float) -> Dict[str, object]:
    """The action moved into the workspace (rules 1 and 2). Unknown keys are
    refused (ValueError), so a typo never passes silently."""
    out: Dict[str, object] = {}
    for key, value in action.items():
        if key in SIDE_KEYS:
            d, v = value                                      # type: ignore[misc]
            out[key] = (_clip(float(d), 0.0, 1.0), _clip(float(v), -1.0, 1.0))
        elif key in PAIR_KEYS:
            a, b = value                                      # type: ignore[misc]
            out[key] = (_clip(float(a), 0.0, 1.0), _clip(float(b), 0.0, 1.0))
        elif key == "gaze":
            x, y = value                                      # type: ignore[misc]
            out[key] = (_clip(float(x), -1.0, 1.0), _clip(float(y), -1.0, 1.0))
        elif key == "jaw":
            out[key] = budget.limit(float(value), t_s)        # type: ignore[arg-type]
        else:
            raise ValueError(f"unknown action key {key!r}")
    if "jaw" not in out:
        out["jaw"] = budget.limit(0.0, t_s)
    return out


REPORT_WINDOW_S = 0.6
SERVO_NAMES = {0: "eye R", 1: "eye pitch", 2: "eye L", 3: "lower lip R", 4: "lower lip L",
               5: "lid R", 6: "lid L", 7: "jaw", 8: "down R", 9: "down L", 10: "brow R",
               11: "diag R", 12: "oculi R", 13: "upper lip R", 14: "up R", 15: "diag L",
               16: "brow L", 17: "oculi L", 18: "upper lip L", 19: "up L"}


def validation_report(run: Path) -> dict:
    """Per servo over the held poses of a head-natural --validate run: the
    commanded goal range, the steady tracking error |goal - present| and the
    load; the jaw's commanded opening vs reached opening."""
    import csv
    from datetime import datetime
    record = json.loads((run / "record.json").read_text())
    poses = record["validate"]["poses"]
    utc = lambda text: datetime.fromisoformat(text).timestamp()   # noqa: E731
    commands = [(float(r["t_s"]), utc(r["host_utc"])) for r in
                csv.DictReader(open(run / "commands.csv", encoding="utf-8"))]
    if not commands:
        raise ValueError("no commands in the run")

    def host_time(t_s: float) -> float:
        best = min(commands, key=lambda c: abs(c[0] - t_s))
        return best[1] + (t_s - best[0])
    windows = [(host_time(end - REPORT_WINDOW_S), host_time(end)) for _start, end in poses]
    sums: List[Dict[int, List[float]]] = [dict() for _ in windows]
    with open(run / "telemetry.csv", encoding="utf-8") as stream:
        for r in csv.DictReader(stream):
            if r["board_state"] != "ENABLED":
                continue
            t = utc(r["host_utc"])
            for k, (a, b) in enumerate(windows):
                if a <= t <= b:
                    acc = sums[k].setdefault(int(r["servo"]), [0.0, 0.0, 0.0, 0.0, 0.0])
                    acc[0] += float(r["goal_tick"]); acc[1] += float(r["present_tick"])
                    acc[2] = max(acc[2], abs(float(r["current_ma"])))
                    acc[3] += 1; acc[4] = max(acc[4], float(r["temperature_c"]))
    held = [{i: {"goal": a[0] / a[3], "present": a[1] / a[3], "peak_ma": a[2], "temp_c": a[4]}
             for i, a in acc.items() if a[3]} for acc in sums]
    measured = [h for h in held if h]
    servos: Dict[str, dict] = {}
    for i in sorted({i for h in measured for i in h}):
        goals = [h[i]["goal"] for h in measured if i in h]
        errors = [abs(h[i]["goal"] - h[i]["present"]) for h in measured if i in h]
        if max(goals) - min(goals) < 5:
            continue                                      # held only
        errors.sort()
        servos[str(i)] = {"name": SERVO_NAMES.get(i, str(i)),
                          "goal_span_ticks": round(max(goals) - min(goals)),
                          "error_median_ticks": round(errors[len(errors) // 2], 1),
                          "error_p90_ticks": round(errors[min(len(errors) - 1, int(0.9 * len(errors)))], 1),
                          "error_max_ticks": round(errors[-1], 1),
                          "peak_ma": round(max(h[i]["peak_ma"] for h in measured if i in h)),
                          "max_temp_c": max(h[i]["temp_c"] for h in measured if i in h)}
    jaw = []
    closed = 4000.0
    for h in measured:
        if 7 in h:
            jaw.append((round(closed - h[7]["goal"]), round(closed - h[7]["present"]), round(h[7]["peak_ma"])))
    jaw.sort()
    return {"run": str(run), "stopped": record.get("stopped"), "poses": len(poses),
            "poses_measured": len(measured), "clamped": record.get("clamped_by_profile"),
            "enable_retries": record.get("enable_retries"),
            "max_temperature_c": max((record.get("max_temperature_c") or {"-": 0}).values()),
            "servos": servos, "jaw_goal_vs_reached_open_ticks_ma": jaw}


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--write", type=Path, help="write the measured default spec here")
    parser.add_argument("--show", type=Path, help="print a spec file and its session flags")
    parser.add_argument("--report", type=Path, metavar="RUN",
                        help="validation report of a head-natural --validate run folder")
    a = parser.parse_args(argv)
    if a.write:
        a.write.parent.mkdir(parents=True, exist_ok=True)
        a.write.write_text(json.dumps(WorkspaceSpec().to_json(), indent=1) + "\n")
        print(f"Wrote {a.write}")
    if a.show:
        spec = WorkspaceSpec.from_json(json.loads(a.show.read_text()))
        print(json.dumps(spec.to_json(), indent=1))
        print("Session flags: " + " ".join(spec.release_args())
              + f" --jaw-open {spec.jaw_open_goal_tick} --jaw-current {spec.jaw_current_cap_ma}")
    if a.report:
        data = validation_report(a.report)
        (a.report / "workspace_validation.json").write_text(json.dumps(data, indent=1) + "\n")
        print(f"{data['run']}: {data['poses_measured']}/{data['poses']} poses measured, stopped: "
              f"{data['stopped']}, clamped: {data['clamped']}, enable retries: "
              f"{data['enable_retries']}, max {data['max_temperature_c']} C")
        print(f"{'servo':<16}{'span':>6}{'err med':>9}{'p90':>7}{'max':>7}{'peak mA':>9}")
        for i, v in data["servos"].items():
            print(f"{i:>2} {v['name']:<13}{v['goal_span_ticks']:>6}{v['error_median_ticks']:>9}"
                  f"{v['error_p90_ticks']:>7}{v['error_max_ticks']:>7}{v['peak_ma']:>9}")
        jaw = data["jaw_goal_vs_reached_open_ticks_ma"]
        if jaw:
            print("jaw goal open -> reached open (ticks), peak mA:",
                  ", ".join(f"{g}->{r} ({m})" for g, r, m in jaw[::max(1, len(jaw) // 8)]))
    if not (a.write or a.show or a.report):
        parser.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
