"""Combined pulls on one modiolus, with the opposing line only following.

Rule (operator, 2026-10-20): the down line (8 right, 9 left) never pulls
against the up and diag lines (14/11 right, 19/16 left), and they never pull
against it. Whichever lines drive, the others follow: each follower pays out
when its load rises over preload + half the band, and takes in slack when it
reads under preload - half the band, so it keeps pretension and no more.

For each side and jaw tick: the lines are slacked, the jaw moved, and every
line pulled back to the preload from below (neutral), as in head-ranges. Then
each combination drives its lines together from neutral, in steps of
ACTIVATION_STEP of each line's single-line range times the line's weight:

    up + diag: weights (up, diag) = (1, 0), (1, 0.5), (1, 1), (0.5, 1), (0, 1)
    down:      weight 1, with up and diag following

until a driver reaches --end-ma, a driver reaches its full single-line range,
or a driver's load falls under DROP_FRACTION of the highest it had while
pulling (the magnet losing grip), or a stop. The activation reached is saved;
every line then goes back to neutral. Same stops as head-ranges.

    bench/.venv/bin/python coact_test.py --limit 650 --side right --side left \\
        --jaw-tick 4000 --jaw-tick 3880 --camera realsense
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import signal
import sys
import time
from typing import Dict, List, Optional

try:
    from . import headctl, records_layout, tuning_core
    from .sweep_runner import DEFAULT_RECORDS, Runner, add_jaw_arguments, session_limits
    from .range_finder import RangeFinder
    from .jaw_probe import Probe
except ImportError:  # run as a plain script from this directory
    import headctl  # type: ignore[no-redef]
    import records_layout  # type: ignore[no-redef]
    import tuning_core  # type: ignore[no-redef]
    from sweep_runner import DEFAULT_RECORDS, Runner, add_jaw_arguments, \
        session_limits  # type: ignore[no-redef]
    from range_finder import RangeFinder  # type: ignore[no-redef]
    from jaw_probe import Probe  # type: ignore[no-redef]

SIDES = {"right": {"down": 8, "diag": 11, "up": 14}, "left": {"down": 9, "diag": 15, "up": 19}}
# Single-line ranges from neutral (ticks), from the 2026-10-20 mapping and the
# operator's cuts: up 700, diag 580 (magnets lose grip beyond), down 800.
RANGES = {"up": 700, "diag": 580, "down": 800}
COMBOS = [("up+diag", {"up": 1.0, "diag": 0.0}), ("up+diag", {"up": 1.0, "diag": 0.5}),
          ("up+diag", {"up": 1.0, "diag": 1.0}), ("up+diag", {"up": 0.5, "diag": 1.0}),
          ("up+diag", {"up": 0.0, "diag": 1.0}), ("down", {"down": 1.0})]
ACTIVATION_STEP = 0.01
DROP_FRACTION = 0.5
DROP_MIN_PEAK_MA = 80
DROP_SAMPLES = 3
COMBO_TIMEOUT_S = 60.0
# A follower moves at most this far from its neutral; past it, it holds and is
# flagged. A line that never takes load (8 slips on its horn, 2026-10-20)
# would otherwise keep taking in line for the whole combination.
FOLLOW_MAX_TICKS = 600


class CoactTest:
    def __init__(self, finder: RangeFinder, pull: Dict[int, int], preload_ma: int, band_ma: int,
                 end_ma: int) -> None:
        self.finder, self.link = finder, finder.link
        self.pull, self.preload_ma, self.band_ma, self.end_ma = pull, preload_ma, band_ma, end_ma

    def tension(self, index: int) -> int:
        return max(0, self.link.state.servos[index].current_ma * self.pull[index])

    def follow_step(self, index: int) -> int:
        """A follower's step: out when over the preload band, in when under."""
        error = self.preload_ma - self.tension(index)
        if abs(error) <= max(1, self.band_ma // 2):
            return 0
        size = max(1, min(tuning_core.TENSION_MAX_STEP_TICKS,
                          round(tuning_core.TENSION_GAIN * abs(error) / tuning_core.LAG_MA_PER_TICK)))
        return size * (1 if error > 0 else -1) * self.pull[index]

    def run_combo(self, side: Dict[str, int], weights: Dict[str, float], watched: List[int]) -> dict:
        state = self.link.state
        drivers = {side[role]: (weight, RANGES[role]) for role, weight in weights.items() if weight > 0}
        # Every line that does not drive follows, also one with weight 0.
        followers = [index for index in side.values() if index not in drivers]
        neutral = {index: state.servos[index].goal_tick for index in side.values()}
        neutral_present = {index: state.servos[index].present_tick for index in side.values()}
        peak = {index: 0 for index in drivers}
        drops = {index: 0 for index in drivers}
        activation = 0.0
        reason, stopped_by = None, None
        deadline = time.monotonic() + COMBO_TIMEOUT_S
        order = [*drivers, *followers]
        turn = 0
        capped_followers: set = set()
        while True:
            reason = self.finder.pump(watched)
            if reason is not None:
                stopped_by = "stop"
                break
            if time.monotonic() > deadline:
                stopped_by, reason = "timeout", f"combination did not finish in {COMBO_TIMEOUT_S:.0f} s"
                break
            state = self.link.state
            if self.finder.new_sample():
                for index in drivers:
                    load = self.tension(index)
                    peak[index] = max(peak[index], load)
                    dropped = peak[index] >= DROP_MIN_PEAK_MA and load < DROP_FRACTION * peak[index]
                    drops[index] = drops[index] + 1 if dropped else 0
                if any(count >= DROP_SAMPLES for count in drops.values()):
                    stopped_by = "load drop"
                    break
                if any(self.tension(index) >= self.end_ma for index in drivers):
                    stopped_by = "end load"
                    break
            targets = {index: neutral[index] + self.pull[index] * round(activation * weight * span)
                       for index, (weight, span) in drivers.items()}
            if all(state.servos[index].goal_tick == targets[index] for index in drivers):
                if activation >= 1.0 - 1e-9:
                    stopped_by = "full range"
                    break
                activation = min(1.0, activation + ACTIVATION_STEP)
                continue
            # One step per call, drivers and followers in turn.
            for offset in range(len(order)):
                index = order[(turn + offset) % len(order)]
                if not self.finder.may_step(index):
                    continue
                if index in drivers:
                    error = targets[index] - state.servos[index].goal_tick
                    step = max(-tuning_core.TENSION_MAX_STEP_TICKS,
                               min(tuning_core.TENSION_MAX_STEP_TICKS, error))
                else:
                    step = self.follow_step(index)
                    moved = (state.servos[index].goal_tick + step - neutral[index]) * self.pull[index]
                    if step and abs(moved) > FOLLOW_MAX_TICKS:
                        capped_followers.add(index)
                        step = 0
                if step:
                    self.link.request_step(index, step)
                    turn = (turn + offset + 1) % len(order)
                    break
        state = self.link.state
        result = {"weights": weights, "activation": round(activation, 3), "stopped_by": stopped_by,
                  "reason": reason, "followers_at_cap": sorted(capped_followers),
                  "lines": {str(index): {
                      "role": next(role for role, value in side.items() if value == index),
                      "driver": index in drivers,
                      "neutral_tick": neutral_present[index],
                      "tick": state.servos[index].present_tick,
                      "moved_ticks": (state.servos[index].present_tick - neutral_present[index]) * self.pull[index],
                      "load_ma": state.servos[index].current_ma,
                      "peak_ma": peak.get(index)} for index in side.values()}}
        return result

    def back_to_neutral(self, goals: Dict[int, int], watched: List[int]) -> Optional[str]:
        for index, goal in goals.items():
            reason = self.finder.go_to_goal(index, goal, watched)
            if reason is not None:
                return reason
        return None


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=headctl.AUTO_PORT)
    parser.add_argument("--records", type=Path, default=DEFAULT_RECORDS)
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--limit", type=int, required=True)
    parser.add_argument("--side", choices=sorted(SIDES), action="append", required=True)
    parser.add_argument("--jaw-tick", type=int, action="append", required=True)
    parser.add_argument("--slack-ticks", type=int, default=300)
    parser.add_argument("--end-ma", type=int, default=150)
    parser.add_argument("--camera", choices=("none", "realsense"), default="none")
    add_jaw_arguments(parser)
    for key in tuning_core.SWEEP_BOX_DEFAULTS:
        parser.add_argument("--" + key.replace("_", "-"), dest="box_" + key)
    a = parser.parse_args(argv)

    records = a.records / "preview" if a.preview else a.records
    boxes = (dict(tuning_core.PREVIEW_SWEEP_BOXES) if a.preview else tuning_core.load_sweep_boxes(records))
    for key in tuning_core.SWEEP_BOX_DEFAULTS:
        value = getattr(a, "box_" + key)
        if value is not None:
            boxes[key] = value
    pull = tuning_core.parse_pull_directions(boxes["pull"])
    sides = {name: SIDES[name] for name in a.side}
    if a.preview:   # the simulated board has only the right side, with 8/11/14
        sides = {"right": SIDES["right"]}
    for side in sides.values():
        for index in side.values():
            if index not in pull:
                print(f"Servo {index} has no pull direction in the boxes."); return 4
    jaw = int(boxes["jaw"])
    stop_file = records / "STOP"
    if stop_file.exists():
        print(f"Remove the stop file first: {stop_file}"); return 4

    camera = None
    if a.camera == "realsense":
        from face_camera import RealSenseCamera  # optional
        camera = RealSenseCamera()
    device = tuning_core.preview_controller() if a.preview else headctl.Device(a.port)
    link = tuning_core.TuningLink(device)
    link.connect()
    info = headctl.decode_info(device.get(headctl.GET_INFO, headctl.INFO))
    branches = headctl.read_servo_branches(device, info["active_servo_mask"])
    caps, guard = session_limits(device, info, a.limit, jaw, a)
    run = records_layout.run_dir(records, "coact")
    print(f"Run folder: {run}", flush=True)
    runner = Runner(link, branches, run, a.limit, boxes, stop_file,
                    say=lambda text: print(text, flush=True), caps=caps, jaw_guard=guard,
                    jaw_abort_ma=a.jaw_abort_ma)
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda signum, _f: runner.request_stop(signal.Signals(signum).name))
    abort = int(boxes["abort_ma"])
    finder = RangeFinder(runner, a.end_ma, abort, 4096, jaw, a.jaw_abort_ma or abort)
    test = CoactTest(finder, pull, int(boxes["preload_ma"]), int(boxes["band_ma"]), a.end_ma)
    out_path = run / "coact.json"
    frames = run / "pictures"
    mover = Probe(runner, jaw, pull, abort, camera, frames, a.jaw_abort_ma or abort)
    log = {"format": "head-coact-test-1", "started_utc": datetime.now(timezone.utc).isoformat(),
           "boxes": boxes, "ranges": RANGES, "end_ma": a.end_ma, "limit_ma": a.limit,
           "caps": {str(k): v for k, v in caps.items()}, "results": [], "stopped": None}
    print("Caps and boxes as above; ranges " + ", ".join(f"{k} {v}" for k, v in RANGES.items()), flush=True)
    reason = None
    try:
        if not runner.start():
            return 4
        watched = [jaw, *pull]
        for target in a.jaw_tick:
            state = link.state
            print(f"Jaw to {target}: slack every line by {a.slack_ticks}, move the jaw...", flush=True)
            reason = mover.lines_to({i: state.servos[i].goal_tick - sign * a.slack_ticks
                                     for i, sign in pull.items()}, True)
            reason = reason or mover.jaw_to(target, int(boxes["jaw_push_ma"]), f"to{target}")
            if reason:
                break
            settings = runner.settings_for(0, 1)
            settings.defer_on_open = []
            row, reason, _torque = finder.slice_neutral(settings)
            if row is None:
                break
            state = link.state
            neutral_goals = {i: state.servos[i].goal_tick for i in pull}
            print(f"  neutral at jaw {state.servos[jaw].present_tick}: "
                  + " ".join(f"{i}:{state.servos[i].present_tick}" for i in pull), flush=True)
            mover.picture(f"jaw{target}_neutral")
            for side_name, side in sides.items():
                for kind, weights in COMBOS:
                    label = "+".join(f"{role}{weight:g}" for role, weight in weights.items())
                    result = test.run_combo(side, weights, watched)
                    result.update({"side": side_name, "jaw_target": target,
                                   "jaw_tick": link.state.servos[jaw].present_tick})
                    log["results"].append(result)
                    tuning_core.write_json_atomically(out_path, log)
                    lines = " ".join(f"{k}:{v['moved_ticks']:+d}t/{v['load_ma']:+d}mA"
                                     + ("*" if v["driver"] else "") for k, v in result["lines"].items())
                    print(f"  {side_name} {label}: activation {result['activation']:.2f} "
                          f"({result['stopped_by']}) | {lines}", flush=True)
                    mover.picture(f"jaw{target}_{side_name}_{label}")
                    if result["stopped_by"] in ("stop", "timeout"):
                        reason = result["reason"]
                        break
                    reason = test.back_to_neutral({i: neutral_goals[i] for i in side.values()}, watched)
                    if reason:
                        break
                if reason:
                    break
            if reason:
                break
        return 3 if reason else 0
    finally:
        log["stopped"] = reason
        runner.disable()
        tuning_core.write_json_atomically(out_path, log)
        print(f"Torque off: {reason or 'end of test'}. {out_path}", flush=True)
        runner.close()
        if camera is not None:
            camera.close()
        try:
            device.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
