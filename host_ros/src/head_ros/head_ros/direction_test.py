"""Find which way each servo tightens its line, one servo at a time.

For a newly assembled head, where the old pull directions and end marks do
not apply. Each servo in turn is moved AMPLITUDE ticks one way, back to where
it started, the same distance the other way, and back again. The other
servos hold where they are. At each end the tool waits for the servo to
settle and records its tick and current, and a camera picture with
--camera realsense. A line that is pulled tight shows load (and the face
moves); a line that is paid out goes slack (current near zero).

Several --amplitude values are tried smallest first. A servo stops at the
first amplitude where one way reaches --load-ma and the other does not: that
is its tightening direction. Nothing is decided for the operator: the result
file lists what was measured and the pictures to compare.

Safety: every firmware tuning check applies (current cap, lag limit, one
turn of travel). On top of it, torque goes off at once when any servo reaches
--abort-ma, on a board fault, on back-off, on Ctrl-C/SIGTERM, or when the stop
file (default <records>/STOP) appears. The jaw is not tested here. An
amplitude may not be over MAX_AMPLITUDE_TICKS. The panel must be closed.

    scripts/head-direction --limit 300 --servo 8 --servo 9 --amplitude 100 --amplitude 250 --camera realsense
    scripts/head-direction --preview --limit 300 --servo 8 --amplitude 100
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import signal
import statistics
import sys
import time
from typing import Dict, List, Optional

try:
    from . import headctl, records_layout, sweep_runner, tuning_core
except ImportError:  # run as a plain script from this directory
    import headctl  # type: ignore[no-redef]
    import records_layout  # type: ignore[no-redef]
    import sweep_runner  # type: ignore[no-redef]
    import tuning_core  # type: ignore[no-redef]

JAW_INDEX = 7
MAX_AMPLITUDE_TICKS = 300
DEFAULT_ABORT_MA = 280
DEFAULT_LOAD_MA = 60
# A move ends early at this load: the test only has to see load, not push.
DEFAULT_STOP_MA = 150
# Goal lead over the servo during a move (ticks). Small, so the load rises
# slowly and the stop load is seen before the abort load.
MOVE_MAX_LEAD_TICKS = 20
MOVE_TIMEOUT_S = 15.0
SETTLE_S = 0.6
EXIT_FINISHED = sweep_runner.EXIT_FINISHED
EXIT_TORQUE_OFF = sweep_runner.EXIT_TORQUE_OFF
EXIT_NOT_STARTED = sweep_runner.EXIT_NOT_STARTED


HINT_MIN_MA = 15


def current_hint(ends: Dict) -> Optional[str]:
    """'+' or '-' when the holding current says which way the line pulls,
    else None. A servo holding a taut line pushes toward the tightening side,
    so the current's sign is the pull direction (tendon load =
    current x pull). Given only when both ends have the same sign and the
    larger load is on that side, at least HINT_MIN_MA: weak evidence, since
    stiction alone is 20-50 mA."""
    plus, minus = ends["plus"]["current_ma"], ends["minus"]["current_ma"]
    if plus * minus <= 0:
        return None
    sign = "+" if plus > 0 else "-"
    larger_end = "plus" if abs(plus) > abs(minus) else "minus"
    if max(abs(plus), abs(minus)) < HINT_MIN_MA:
        return None
    return sign if (larger_end == "plus") == (sign == "+") else None


class DirectionTest:
    def __init__(self, runner: sweep_runner.Runner, run: Path, abort_ma: int,
                 load_ma: int, stop_ma: int = DEFAULT_STOP_MA, say=print) -> None:
        self.runner = runner
        self.link = runner.link
        self.run = Path(run)
        self.abort_ma = abort_ma
        self.load_ma = load_ma
        self.stop_ma = stop_ma
        self.say = say
        self.result: Dict = {"format": "head-direction-test-1",
                             "started_utc": datetime.now(timezone.utc).isoformat(),
                             "limit_ma": runner.limit_ma, "abort_ma": abort_ma,
                             "load_ma": load_ma, "stop_ma": stop_ma, "servos": {}, "stopped": None}

    # Board -------------------------------------------------------------------
    def check(self) -> Optional[str]:
        """One poll; a reason to turn torque off, or None. Step answers are
        left in link.step_results for the caller (move_to gives them to
        HomeMover, which must see each one)."""
        reason = self.runner.pump()
        if reason is not None:
            return reason
        state = self.link.state
        if state is None:
            return None
        if not self.link.has_control or state.state != tuning_core.TUNING_STATE:
            return "the board left tuning"
        for index in self.runner.branches:
            current = abs(state.servos[index].current_ma)
            if current >= self.abort_ma:
                return f"servo {index} at {current} mA (abort load {self.abort_ma} mA)"
        return None

    def move_to(self, index: int, target: int,
                stop_at_load: bool = True) -> tuple[Optional[str], Optional[str]]:
        """(torque-off reason, notice). Drives one servo to target with
        HomeMover's lag-limited steps; with stop_at_load the move ends where
        the servo first reaches stop_ma."""
        mover = tuning_core.HomeMover({index: target}, MOVE_MAX_LEAD_TICKS)
        deadline = time.monotonic() + MOVE_TIMEOUT_S
        while True:
            reason = self.check()
            if reason is not None:
                return reason, None
            while self.link.step_results:
                servo_index, step, result = self.link.step_results.pop(0)
                state = self.link.state
                if self.runner.record is not None and state is not None:
                    self.runner.record.add_step(state.servos[servo_index], step, result)
                message = mover.on_result(servo_index, result, self.link.clock())
                if message:
                    return None, message
            state = self.link.state
            if stop_at_load and state is not None and \
                    abs(state.servos[index].current_ma) >= self.stop_ma:
                return None, (f"stopped at load {state.servos[index].current_ma:+d} mA, "
                              f"{state.servos[index].present_tick}")
            # given_up before is_done: is_done ignores servos given up.
            if mover.given_up:
                return None, " ".join(mover.notices) or f"servo {index} given up"
            if mover.is_done(state):
                return None, None
            if time.monotonic() > deadline:
                return None, f"servo {index} did not reach {target} in {MOVE_TIMEOUT_S:g} s"
            if not self.link.step_in_flight and self.link.state_is_fresh():
                step = mover.next_step(state)
                if step is not None:
                    self.link.request_step(*step)

    def settle(self, index: int) -> tuple[Optional[str], Dict]:
        """Wait SETTLE_S and return the servo's median tick and current."""
        ticks: List[int] = []
        currents: List[int] = []
        last = None
        end = time.monotonic() + SETTLE_S
        while time.monotonic() < end:
            reason = self.check()
            self.runner.take_results(None)
            if reason is not None:
                return reason, {}
            state = self.link.state
            if state is not None and state.received_monotonic_s != last:
                last = state.received_monotonic_s
                ticks.append(state.servos[index].present_tick)
                currents.append(state.servos[index].current_ma)
        if not ticks:
            return None, {}
        state = self.link.state
        return None, {"present_tick": int(statistics.median(ticks)),
                      "goal_tick": state.servos[index].goal_tick,
                      "current_ma": int(statistics.median(currents)),
                      "peak_current_ma": max(currents, key=abs)}

    def picture(self, name: str) -> Optional[str]:
        camera = self.runner.camera
        if camera is None:
            return None
        path = self.run / "stills" / f"{name}.jpg"
        try:
            camera.save(path)
        except Exception as error:  # a camera fault never stops the test
            self.say(f"  no picture: {error}")
            return None
        return str(path.relative_to(self.run))

    # Test --------------------------------------------------------------------
    def test_servo(self, index: int, amplitudes: List[int]) -> Optional[str]:
        state = self.link.state
        start = state.servos[index].present_tick
        entry = {"start_tick": start, "trials": [], "tightens_toward": None}
        self.result["servos"][str(index)] = entry
        self.say(f"Servo {index}: starts at {start}.")
        for amplitude in amplitudes:
            trial = {"amplitude_ticks": amplitude, "rest": None, "ends": {}}
            entry["trials"].append(trial)
            reason, rest = self.settle(index)
            if reason:
                return reason
            rest["picture"] = self.picture(f"servo{index:02d}_a{amplitude}_rest")
            trial["rest"] = rest
            for sign, name in ((1, "plus"), (-1, "minus")):
                target = start + sign * amplitude
                reason, notice = self.move_to(index, target)
                if reason:
                    self.save()
                    return reason
                reason, end = self.settle(index)
                if reason:
                    self.save()
                    return reason
                end["target_tick"] = target
                end["start_tick"] = start
                end["notice"] = notice
                end["picture"] = self.picture(f"servo{index:02d}_a{amplitude}_{name}")
                trial["ends"][name] = end
                self.say(f"  {name} {amplitude}: at {end['present_tick']} (target {target}), "
                         f"{end['current_ma']:+d} mA, peak {end['peak_current_ma']:+d} mA"
                         + (f" | {notice}" if notice else ""))
                reason, notice = self.move_to(index, start, stop_at_load=False)
                if reason:
                    self.save()
                    return reason
                if notice:
                    self.say(f"  back to start: {notice}")
                self.save()
            tight = {name: self.is_tight(end) for name, end in trial["ends"].items()}
            trial["tight"] = tight
            trial["current_hint"] = current_hint(trial["ends"])
            if trial["current_hint"]:
                self.say(f"  current hint: tightens toward {trial['current_hint']} (weak evidence)")
            if tight["plus"] != tight["minus"]:
                entry["tightens_toward"] = "+" if tight["plus"] else "-"
                self.say(f"  tight on {entry['tightens_toward']} only. Done with servo {index}.")
                break
            if tight["plus"] and tight["minus"]:
                self.say(f"  tight both ways: servo {index} is not a simple line. Stopped here.")
                break
        if entry["tightens_toward"] is None:
            self.say(f"  servo {index}: no clear load difference at {amplitudes[-1]} ticks.")
        self.save()
        return None

    def is_tight(self, end: Dict) -> bool:
        """An end is tight when the servo was blocked before its target
        (stop load, or it could not follow within the lead limit) or holds
        load_ma there."""
        # Only short of the target counts: a servo past its target (a loaded
        # line pulling the horn on, 2026-11-09 servo 10) was not blocked.
        travel = end["target_tick"] - end["start_tick"]
        short_ticks = (end["target_tick"] - end["present_tick"]) * (1 if travel > 0 else -1)
        blocked = short_ticks > tuning_core.HOME_ARRIVE_TICKS
        return blocked or abs(end["current_ma"]) >= self.load_ma

    def save(self) -> None:
        tuning_core.write_json_atomically(self.run / "direction.json", self.result)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=headctl.AUTO_PORT)
    parser.add_argument("--records", type=Path, default=sweep_runner.DEFAULT_RECORDS)
    parser.add_argument("--preview", action="store_true",
                        help="simulated board, records in <records>/preview")
    parser.add_argument("--limit", type=int, required=True, help="tuning load limit (mA)")
    parser.add_argument("--servo", type=int, action="append", required=True)
    parser.add_argument("--amplitude", type=int, action="append", required=True,
                        help=f"ticks each way (repeat, smallest first; max {MAX_AMPLITUDE_TICKS})")
    parser.add_argument("--abort-ma", type=int, default=DEFAULT_ABORT_MA)
    parser.add_argument("--load-ma", type=int, default=DEFAULT_LOAD_MA,
                        help="load that counts as 'tight' at an end")
    parser.add_argument("--stop-ma", type=int, default=DEFAULT_STOP_MA,
                        help="a move ends early at this load")
    parser.add_argument("--camera", choices=("none", "realsense"), default="none")
    parser.add_argument("--stop-file", type=Path)
    arguments = parser.parse_args(argv)

    amplitudes = sorted(set(arguments.amplitude))
    if any(a <= 0 or a > MAX_AMPLITUDE_TICKS for a in amplitudes):
        parser.error(f"each --amplitude must be 1..{MAX_AMPLITUDE_TICKS}")
    if JAW_INDEX in arguments.servo:
        parser.error("the jaw (7) is not tested here")
    if arguments.abort_ma >= arguments.limit * tuning_core.BACKOFF_LOAD_FRACTION:
        parser.error("--abort-ma must be below 95 % of --limit")
    if not arguments.load_ma <= arguments.stop_ma < arguments.abort_ma:
        parser.error("need --load-ma <= --stop-ma < --abort-ma")

    records = arguments.records / "preview" if arguments.preview else arguments.records
    stop_file = arguments.stop_file or records / "STOP"
    if stop_file.exists():
        print(f"Remove the stop file first: {stop_file}")
        return EXIT_NOT_STARTED

    camera = None
    if arguments.camera == "realsense":
        from face_camera import RealSenseCamera  # optional, see face_camera.py
        camera = RealSenseCamera()

    device = tuning_core.preview_controller() if arguments.preview \
        else headctl.Device(arguments.port)
    link = tuning_core.TuningLink(device)
    link.connect()
    info = headctl.decode_info(device.get(headctl.GET_INFO, headctl.INFO))
    branches = headctl.read_servo_branches(device, info["active_servo_mask"])
    missing = [index for index in arguments.servo if index not in branches]
    if missing:
        print(f"Not active on the board: {missing}")
        return EXIT_NOT_STARTED
    guard_args = argparse.Namespace(jaw_hold_ma=tuning_core.JAW_HOLD_MA,
                                    jaw_hold_s=tuning_core.JAW_HOLD_S,
                                    jaw_peak_ma=tuning_core.JAW_PEAK_MA,
                                    jaw_peak_s=tuning_core.JAW_PEAK_S)
    caps, guard = sweep_runner.session_limits(device, info, arguments.limit, JAW_INDEX,
                                              guard_args)
    run = records_layout.run_dir(records, "direction")
    print(f"Run folder: {run}", flush=True)
    runner = sweep_runner.Runner(link, branches, run, arguments.limit, {}, stop_file,
                                 camera=camera, say=lambda text: print(text, flush=True),
                                 caps=caps, jaw_guard=guard)
    test = DirectionTest(runner, run, arguments.abort_ma, arguments.load_ma,
                         arguments.stop_ma, say=lambda text: print(text, flush=True))
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda signum, _frame:
                      runner.request_stop(signal.Signals(signum).name))
    try:
        if not runner.start():
            return EXIT_NOT_STARTED
        for index in arguments.servo:
            reason = test.test_servo(index, amplitudes)
            if reason is not None:
                test.result["stopped"] = reason
                test.save()
                print(f"Torque off: {reason}.", flush=True)
                runner.disable()
                return EXIT_TORQUE_OFF
        runner.disable()
        test.save()
        print(f"Torque off: end of run. Result: {run / 'direction.json'}", flush=True)
        return EXIT_FINISHED
    except BaseException:
        runner.disable()
        raise
    finally:
        runner.close()
        try:
            device.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
