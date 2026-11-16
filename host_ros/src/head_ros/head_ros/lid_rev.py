"""Eyelid blink by a full servo revolution versus back-and-forth.

    scripts/head-lidrev --neutrals tuning_runs/runs/<date>/<time>_knee/neutrals.json \
        --camera realsense [--speeds 1000,2000,3500,5000,6000,7000]

Operator, 2026-11-11: the lid mechanism is a crank, so a lid comes back to
its place after a full turn of its servo; a blink by a full turn never has
to stop and turn back. This test compares, at each speed (ticks/s, ramped up
in order), for lids 5 and 6 together:

  bf:   a back-and-forth blink: closing to the natural-face blink depth
        (BLINK_DEPTH of the closed mark) and back to neutral;
  rev:  one full turn (4096 ticks) in the closing direction;
  back: one full turn back (so every servo ends where it started and its
        marks stay valid).

Every move uses the same acceleration limit (--accel). The log has, per lid
and move, the time to reach the blink depth ("covered") and the time to the
end of the move. Operator allowed for this test (2026-11-11): moving past the
lids' closed marks, full-turn moves, and speeds up to 7000 ticks/s (above the
5000 ticks/s session cap; XC330 no-load speed is about 7100).

Stops: face_stream session stops (52 C, board faults), a lid at
EXTRA_ABORT_MA, or a lid lagging its goal by more than LAG_STOP_TICKS for
LAG_STOP_S (it is not following: a blocked crank).
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
from typing import Dict, List, Optional, Tuple

try:
    from . import eye_tour, face_stream as fs, natural_face as nf, tuning_core
except ImportError:  # run as a plain script from this directory
    import eye_tour  # type: ignore[no-redef]
    import face_stream as fs  # type: ignore[no-redef]
    import natural_face as nf  # type: ignore[no-redef]
    import tuning_core  # type: ignore[no-redef]

LIDS = nf.LIDS
TURN_TICKS = 4096
SPEEDS = (1000.0, 2000.0, 3500.0, 5000.0, 6000.0, 7000.0)
# Operator, 2026-11-11: the lids may use full speed as long as their current
# stays under 600 mA (run 181754 at the 300 mA cap lagged 220-280 ticks above
# 5000 ticks/s, i.e. current-limited). --current sets their cap for the
# session; the abort is then 95 % of it (back-off rule).
SPEED_MAX_TPS = 10000.0                # goals faster than the servo can turn: it runs flat out
CURRENT_MAX_MA = 600
ACCEL_TPS2 = nf.LID_ACCEL_TPS2
ARRIVE_TICKS = 40
REST_S = 1.2
LAG_STOP_TICKS, LAG_STOP_S = 400, 0.3
ABORT_MA = nf.EXTRA_ABORT_MA


def plan_moves(speeds) -> List[Tuple[str, float]]:
    out = []
    for v in speeds:
        out += [("bf_close", v), ("bf_open", v), ("rev", v), ("back", v)]
    return out


class LidRev(fs.ExtraServos):
    servos = LIDS

    def __init__(self, marks_limits: Dict[int, dict], speeds, accel: float = ACCEL_TPS2,
                 depth: Optional[Dict[int, float]] = None, current_ma: Optional[int] = None,
                 rev: bool = True) -> None:
        self.current_ma = current_ma
        self.abort_ma = int(0.95 * current_ma) if current_ma else ABORT_MA
        self.rev = rev
        self.neutral = {i: int(marks_limits[i]["neutral"]) for i in LIDS}
        closed = {i: int(marks_limits[i]["ends"][0]) for i in LIDS}
        self.dir = {i: 1 if closed[i] > self.neutral[i] else -1 for i in LIDS}
        depth = depth or dict(nf.BLINK_DEPTH)
        self.covered = {i: round(depth[i] * abs(closed[i] - self.neutral[i])) for i in LIDS}
        self.moves = plan_moves(speeds) if rev else [m for m in plan_moves(speeds)
                                                      if m[0].startswith("bf")]
        self.accel = float(accel)
        self.k = 0
        self.move_start: Optional[float] = None
        self.rest_until: Optional[float] = None
        self._pos: Dict[int, float] = {}
        self._vel = {i: 0.0 for i in LIDS}
        self._lag_since: Dict[int, float] = {}
        self.log: List[dict] = []
        self.peak_ma: Dict[int, int] = {}
        self.done = False

    def currents(self) -> Dict[int, int]:
        return {i: int(self.current_ma) for i in LIDS} if self.current_ma else {}

    def limits(self, present):
        out = {}
        for i in LIDS:
            far = self.neutral[i] + self.dir[i] * TURN_TICKS
            out[i] = (min(self.neutral[i], far), max(self.neutral[i], far), self.neutral[i],
                      SPEED_MAX_TPS)
        return out

    def _offset(self, kind: str, i: int) -> int:
        return {"bf_close": self.covered[i], "bf_open": 0, "rev": TURN_TICKS, "back": 0}[kind]

    def update(self, t_s, state, dt_s, approaching):
        servos = {s.index: s for s in state.servos}
        for i in LIDS:
            c = servos[i].current_ma
            self.peak_ma[i] = max(self.peak_ma.get(i, 0), abs(c))
            if abs(c) >= self.abort_ma:
                return {}, f"servo {i} at {c} mA (lid abort {self.abort_ma} mA)"
        if not self._pos:
            self._pos = {i: float(servos[i].goal_tick) for i in LIDS}
        if approaching or self.done:
            targets = {i: self.neutral[i] for i in LIDS}
            speed = 1000.0
        else:
            if self.rest_until is not None:
                if t_s < self.rest_until:
                    return {i: round(self._pos[i]) for i in LIDS}, None
                self.rest_until = None
            kind, speed = self.moves[self.k]
            targets = {i: self.neutral[i] + self.dir[i] * self._offset(kind, i) for i in LIDS}
            if self.move_start is None:
                self.move_start = t_s
                self.log.append({"move": kind, "speed_tps": speed, "start_s": round(t_s, 3),
                                 "covered_s": {}, "end_s": {}, "peak_ma": {}, "max_lag": {}})
            entry = self.log[-1]
            for i in LIDS:
                p = servos[i].present_tick
                travel = self.dir[i] * (p - self.neutral[i])
                entry["peak_ma"][str(i)] = max(entry["peak_ma"].get(str(i), 0), abs(servos[i].current_ma))
                lag = abs(servos[i].goal_tick - p)
                entry["max_lag"][str(i)] = max(entry["max_lag"].get(str(i), 0), lag)
                if kind in ("bf_close", "rev") and str(i) not in entry["covered_s"] and \
                        travel >= self.covered[i] - ARRIVE_TICKS:
                    entry["covered_s"][str(i)] = round(t_s - self.move_start, 3)
                if str(i) not in entry["end_s"] and abs(p - targets[i]) <= ARRIVE_TICKS and \
                        abs(self._pos[i] - targets[i]) < 1:
                    entry["end_s"][str(i)] = round(t_s - self.move_start, 3)
                if lag > LAG_STOP_TICKS:
                    if t_s - self._lag_since.setdefault(i, t_s) >= LAG_STOP_S:
                        return {}, (f"lid {i} lags its goal by {lag} ticks for {LAG_STOP_S} s "
                                    f"at {p} (blocked?)")
                else:
                    self._lag_since.pop(i, None)
            if len(entry["end_s"]) == len(LIDS):
                self.k += 1
                self.move_start = None
                self.rest_until = t_s + REST_S
                if self.k >= len(self.moves):
                    self.done = True
        out = {}
        for i in LIDS:
            self._pos[i], self._vel[i] = nf.smooth_step(self._pos[i], self._vel[i], targets[i],
                                                        speed, self.accel, dt_s)
            out[i] = round(self._pos[i])
        return out, None


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    fs.add_session_arguments(parser)
    parser.set_defaults(line_speed=1000.0, jaw_speed=1000.0, retry_enable_bus_fault=True)
    parser.add_argument("--speeds", default=",".join(f"{v:g}" for v in SPEEDS),
                        help=f"lid speeds in order, ticks/s, each at most {SPEED_MAX_TPS:g}")
    parser.add_argument("--accel", type=float, default=ACCEL_TPS2, help="ticks/s^2")
    parser.add_argument("--current", type=int,
                        help=f"lid current cap for the session, mA (at most {CURRENT_MAX_MA}); "
                             "abort at 95 %% of it")
    parser.add_argument("--no-rev", action="store_true", help="back-and-forth blinks only")
    a = parser.parse_args(argv)
    try:
        speeds = [float(x) for x in a.speeds.split(",") if x.strip()]
    except ValueError:
        print("--speeds: comma-separated numbers."); return 2
    if not speeds or any(not 100.0 <= v <= SPEED_MAX_TPS for v in speeds):
        print(f"--speeds must be 100 to {SPEED_MAX_TPS:g}."); return 2
    if not 1000.0 <= a.accel <= 400000.0:
        print("--accel must be 1000-400000."); return 2
    if a.current is not None and not 100 <= a.current <= CURRENT_MAX_MA:
        print(f"--current must be 100 to {CURRENT_MAX_MA}."); return 2
    records = a.records / "preview" if a.preview else a.records
    try:
        if a.preview:
            marks = eye_tour.preview_marks(fs.face_preview_controller())
        else:
            marks = tuning_core.load_earlier_marks(records, since=tuning_core.HEAD_MARKS_SINCE)[0]
        limits = eye_tour.tour_limits(marks)
        extra = LidRev({i: limits[i] for i in LIDS}, speeds, a.accel, current_ma=a.current,
                       rev=not a.no_rev)
    except (eye_tour.SetupError, ValueError, KeyError) as error:
        print(error); return 2
    say = lambda text: print(text, flush=True)  # noqa: E731
    try:
        setup = fs.open_session(a, "face", "lidrev", say, extra=extra)
    except fs.SetupError as error:
        print(error); return 2
    session = setup.session
    session.log["lid_rev"] = {"speeds": speeds, "accel": a.accel, "current_ma": a.current,
                              "abort_ma": extra.abort_ma, "neutral": extra.neutral,
                              "dir": extra.dir, "covered_ticks": extra.covered, "moves": extra.log}
    try:
        try:
            session.start()
        except RuntimeError as error:
            session.finish(f"not started: {error}")
            return 2
        say(f"Enabled; lids: {len(extra.moves)} moves at {speeds} ticks/s.")

        def script(t: float):
            if extra.done and extra.rest_until is not None and t > extra.rest_until:
                return None
            return {"right": (0.0, 0.0), "left": (0.0, 0.0)}, 0.0
        reason = session.run(script)
        return 0 if reason == "finished" else 1
    finally:
        session.log["lid_rev"]["peak_ma"] = {str(i): v for i, v in extra.peak_ma.items()}
        for e in extra.log:
            say(f"  {e['move']:<8} {e['speed_tps']:>6g} t/s: covered {e['covered_s']} end {e['end_s']} "
                f"peak {e['peak_ma']} max lag {e['max_lag']}")
        fs.close_session(setup)


if __name__ == "__main__":
    sys.exit(main())
