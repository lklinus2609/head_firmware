"""Find each tendon's range (neutral and end tick) at several jaw openings.

For each jaw opening (ticks from where the jaw is at the start; the jaw opens
toward LOWER ticks): a jaw sweep of one stage brings the jaw there and pulls
every tendon to the preload from below; those ticks are the tendons' neutral
there. Then each tendon in turn is pulled until it holds END load for
END_SAMPLES samples (or MAX_PULL_TICKS, whichever first); that tick is its
end. It goes back to its neutral before the next tendon. The others and the
jaw hold their goals meanwhile. One ranges.json in its run folder collects everything,
with a picture of each end when --camera realsense.

The panel must be closed. Same stops as head-sweep: Ctrl-C, SIGTERM or the
stop file turn torque off, as do back-off, a board fault, or ABORT on any
servo. At the end torque goes off (the face relaxes).

    scripts/head-ranges --limit 400 --opening 0 --opening -40 --opening -80 --camera realsense

With --neutral-only no ends are searched: it only finds every line's neutral
at each jaw tick and writes neutrals.json (run folder) for head-face. Give
the closed jaw tick first, then the open one (the open one becomes the
session's jaw open limit; 3880 is the most open allowed):

    scripts/head-ranges --limit 650 --neutral-only --slack-ticks 300 --jaw-tick 4000 --jaw-tick 3900
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
    from .sweep_runner import DEFAULT_RECORDS, EXIT_FINISHED, EXIT_HELD, EXIT_NOT_STARTED, \
        EXIT_TORQUE_OFF, Runner, add_jaw_arguments, session_limits
    from .jaw_probe import Probe
except ImportError:  # run as a plain script from this directory
    import headctl  # type: ignore[no-redef]
    import records_layout  # type: ignore[no-redef]
    import tuning_core  # type: ignore[no-redef]
    from sweep_runner import DEFAULT_RECORDS, EXIT_FINISHED, EXIT_HELD, \
        EXIT_NOT_STARTED, EXIT_TORQUE_OFF, Runner, add_jaw_arguments, \
        session_limits  # type: ignore[no-redef]
    from jaw_probe import Probe  # type: ignore[no-redef]

RANGES_FORMAT = "head-tendon-ranges-1"
SESSION_NEUTRAL_FORMAT = "head-session-neutral-1"
SESSION_NEUTRAL_SUFFIX = "_session_neutral.json"
# A tendon's end: it holds this load (mA, pull direction) for END_SAMPLES
# samples in a row. 150 mA (about 18 N of line at the datasheet's 0.95 N m/A
# on the 8.1 mm horn, an upper bound) was chosen by the operator on
# 2026-10-18.
END_MA = 150
END_SAMPLES = 3
# An end search stops here even below END_MA: the line is slack or slipping.
MAX_PULL_TICKS = 800
SETTLE_S = 0.5


class RangeFinder:
    def __init__(self, runner: Runner, end_ma: int, abort_ma: int, max_pull: int,
                 jaw: int, jaw_abort_ma: int) -> None:
        self.runner = runner
        self.link = runner.link
        self.end_ma = end_ma
        self.abort_ma = abort_ma
        self.jaw, self.jaw_abort_ma = jaw, jaw_abort_ma
        # Servo -> its own pull limit, where the default would go too far (the
        # diag modiolus magnet came off at about 780 ticks, 2026-10-20).
        self.max_pull_by_servo: Dict[int, int] = {}
        self.max_pull = max_pull
        self._last_answer: Dict[int, float] = {}
        self._seen_s = float("-inf")

    # One poll, with this tool's own result handling ------------------------------
    def pump(self, watched: List[int]) -> Optional[str]:
        reason = self.runner.pump()
        state = self.link.state
        while self.link.step_results:
            index, step, result = self.link.step_results.pop(0)
            self._last_answer[index] = self.link.clock()
            if self.runner.record is not None and state is not None:
                self.runner.record.add_step(state.servos[index], step, result)
            if result != 0:
                self.runner.say(f"    servo {index} step {step:+d} refused: "
                                f"{tuning_core.step_result_text(result)}")
        if reason is not None:
            return reason
        if state is None or not self.link.has_control or state.state != tuning_core.TUNING_STATE:
            return "the board left tuning"
        for index in watched:
            abort = self.jaw_abort_ma if index == self.jaw else self.abort_ma
            if abs(state.servos[index].current_ma) >= abort:
                return (f"servo {index} at {state.servos[index].current_ma} mA "
                        f"(abort {abort} mA)")
        return None

    def new_sample(self) -> bool:
        state = self.link.state
        if state is None or state.received_monotonic_s <= self._seen_s:
            return False
        self._seen_s = state.received_monotonic_s
        return True

    def may_step(self, index: int) -> bool:
        state = self.link.state
        return (state is not None and not self.link.step_in_flight and self.link.state_is_fresh()
                and state.received_monotonic_s > self._last_answer.get(index, float("-inf")))

    def wait(self, seconds: float, watched: List[int]) -> Optional[str]:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            reason = self.pump(watched)
            if reason is not None:
                return reason
        return None

    # Steps -------------------------------------------------------------------------
    def slice_neutral(self, settings: tuning_core.SweepSettings) -> tuple[Optional[dict], Optional[str], bool]:
        """Run a one-stage sweep until its open stage is recorded: the jaw at
        the target and every tendon at the preload. Returns (row, stop reason,
        whether the stop needs torque off)."""
        runner, link = self.runner, self.link
        sweep = tuning_core.JawSweep(tuning_core.new_sweep_path(runner.records), settings,
                                     link.state, link.clock())
        while True:
            reason = runner.pump()
            if reason is not None:
                sweep.save()
                return None, reason, True
            runner.take_results(sweep)
            if not link.has_control or link.state.state != tuning_core.TUNING_STATE:
                sweep.save()
                return None, "the board left tuning", True
            row = sweep.observe(link.state)
            if row is not None and row["leg"] == "open":
                sweep.save()
                return row, None, False
            if sweep.stopped is None and not link.step_in_flight and link.state_is_fresh():
                step = sweep.next_step(link.state)
                if step is not None:
                    link.request_step(*step)
            if sweep.stopped is not None:
                sweep.save()
                return None, sweep.stopped, sweep.torque_off

    def find_end(self, index: int, pull: int, watched: List[int]) -> dict:
        state = self.link.state
        neutral_goal = state.servos[index].goal_tick
        neutral_tick = state.servos[index].present_tick
        held = 0
        result: dict = {}
        while True:
            reason = self.pump(watched)
            if reason is not None:
                return {"stopped": reason}
            servo = self.link.state.servos[index]
            load = servo.current_ma * pull
            if self.new_sample():
                held = held + 1 if load >= self.end_ma else 0
            if held >= END_SAMPLES:
                result = {"end_tick": servo.present_tick, "end_goal_tick": servo.goal_tick,
                          "load_ma": servo.current_ma, "reason": "end load"}
                break
            pulled = (servo.goal_tick - neutral_goal) * pull
            max_pull = self.max_pull_by_servo.get(index, self.max_pull)
            if pulled >= max_pull:
                result = {"end_tick": servo.present_tick, "end_goal_tick": servo.goal_tick,
                          "load_ma": servo.current_ma,
                          "reason": f"pulled {max_pull} ticks without reaching {self.end_ma} mA"}
                break
            if load < self.end_ma and self.may_step(index):
                size = max(1, min(tuning_core.TENSION_MAX_STEP_TICKS,
                                  round(tuning_core.TENSION_GAIN * (self.end_ma - load)
                                        / tuning_core.LAG_MA_PER_TICK)))
                self.link.request_step(index, size * pull)
        result["neutral_tick"] = neutral_tick
        result["range_ticks"] = (result["end_tick"] - neutral_tick) * pull
        return result

    def go_to_goal(self, index: int, goal: int, watched: List[int]) -> Optional[str]:
        deadline = time.monotonic() + 30.0
        while True:
            reason = self.pump(watched)
            if reason is not None:
                return reason
            if time.monotonic() > deadline:
                return f"servo {index} did not get back to its neutral goal in 30 s"
            error = goal - self.link.state.servos[index].goal_tick
            if error == 0:
                return self.wait(SETTLE_S, watched)
            if self.may_step(index):
                self.link.request_step(index, max(-25, min(25, error)))


def session_neutral(log: dict, jaw: int, session_id: Optional[int]) -> dict:
    """The head-face input from a --neutral-only run: neutral goal ticks with
    the jaw closed (first slice) and open (last slice), and the controller boot
    session they belong to (head-face refuses them after a restart)."""
    closed, opened = log["slices"][0], log["slices"][-1]
    return {"format": SESSION_NEUTRAL_FORMAT, "made_utc": datetime.now(timezone.utc).isoformat(),
            "source": log.get("source"), "controller_session_id": session_id, "jaw": jaw,
            "preload_ma": int(log["boxes"]["preload_ma"]),
            "jaw_closed_tick": closed["jaw_tick"], "jaw_open_tick": opened["jaw_tick"],
            "jaw_open_target": opened.get("jaw_target"),
            "closed": {i: v["goal_tick"] for i, v in closed["neutral"].items()},
            "open": {i: v["goal_tick"] for i, v in opened["neutral"].items()},
            "slices": log["slices"]}


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=headctl.AUTO_PORT)
    parser.add_argument("--records", type=Path, default=DEFAULT_RECORDS)
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--limit", type=int, required=True)
    parser.add_argument("--opening", type=int, action="append",
                        help="jaw opening in ticks from the start (repeat; 0 = as it is)")
    parser.add_argument("--jaw-tick", type=int, action="append",
                        help="absolute jaw tick per slice (repeat), instead of --opening")
    parser.add_argument("--slack-ticks", type=int, default=0,
                        help="before each slice, pay every line out this much, move the jaw "
                             "there, then pull every line back to the preload from below")
    parser.add_argument("--max-pull-servo", action="append", default=[],
                        metavar="SERVO=TICKS", help="a servo's own pull limit (repeat)")
    parser.add_argument("--end-ma", type=int, default=END_MA)
    parser.add_argument("--max-pull", type=int, default=MAX_PULL_TICKS)
    parser.add_argument("--camera", choices=("none", "realsense"), default="none")
    parser.add_argument("--neutral-only", action="store_true",
                        help="find the neutrals only (no ends); write a session neutral file")
    add_jaw_arguments(parser)
    for key in tuning_core.SWEEP_BOX_DEFAULTS:
        parser.add_argument("--" + key.replace("_", "-"), dest="box_" + key)
    raw = list(sys.argv[1:] if argv is None else argv)
    glued: List[str] = []
    for item in raw:   # argparse takes "-40" for an option name; glue it to its flag
        if glued and glued[-1] == "--opening" and item.startswith("-"):
            glued[-1] = "--opening=" + item
        else:
            glued.append(item)
    a = parser.parse_args(glued)
    if bool(a.opening) == bool(a.jaw_tick):
        print("Give either --opening or --jaw-tick."); return EXIT_NOT_STARTED
    one_turn = tuning_core.headctl_one_turn_ticks()
    max_pull_by_servo = {}
    for item in a.max_pull_servo:
        servo, _, ticks = item.partition("=")
        max_pull_by_servo[int(servo)] = int(ticks)
    if not 0 <= a.slack_ticks <= one_turn or not all(0 < v <= one_turn for v in max_pull_by_servo.values()):
        print("Slack and pull limits must be 0 to one turn."); return EXIT_NOT_STARTED

    records = a.records / "preview" if a.preview else a.records
    boxes = (dict(tuning_core.PREVIEW_SWEEP_BOXES) if a.preview
             else tuning_core.load_sweep_boxes(records))
    for key in tuning_core.SWEEP_BOX_DEFAULTS:
        value = getattr(a, "box_" + key)
        if value is not None:
            boxes[key] = value
    if not a.end_ma < int(boxes["abort_ma"]):
        print("The end load must be below the abort load."); return EXIT_NOT_STARTED
    if not 0 < a.max_pull <= tuning_core.headctl_one_turn_ticks():
        print("--max-pull must be 1 to one turn."); return EXIT_NOT_STARTED
    stop_file = records / "STOP"
    if stop_file.exists():
        print(f"Remove the stop file first: {stop_file}"); return EXIT_NOT_STARTED

    camera = None
    if a.camera == "realsense":
        from face_camera import RealSenseCamera  # optional
        camera = RealSenseCamera()
    device = tuning_core.preview_controller() if a.preview else headctl.Device(a.port)
    link = tuning_core.TuningLink(device)
    link.connect()
    info = headctl.decode_info(device.get(headctl.GET_INFO, headctl.INFO))
    branches = headctl.read_servo_branches(device, info["active_servo_mask"])
    caps, guard = session_limits(device, info, a.limit, int(boxes["jaw"]), a)
    run = records_layout.run_dir(records, "neutrals" if a.neutral_only else "ranges")
    print(f"Run folder: {run}", flush=True)
    runner = Runner(link, branches, run, a.limit, boxes, stop_file,
                    say=lambda text: print(text, flush=True), caps=caps, jaw_guard=guard,
                    jaw_abort_ma=a.jaw_abort_ma)
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda signum, _f: runner.request_stop(signal.Signals(signum).name))
    finder = RangeFinder(runner, a.end_ma, int(boxes["abort_ma"]), a.max_pull, int(boxes["jaw"]),
                         a.jaw_abort_ma or int(boxes["abort_ma"]))
    finder.max_pull_by_servo = max_pull_by_servo
    out_path = run / "ranges.json"
    frames = run / "pictures"
    log = {"format": RANGES_FORMAT, "started_utc": datetime.now(timezone.utc).isoformat(),
           "boxes": boxes, "end_ma": a.end_ma, "max_pull_ticks": a.max_pull,
           "limit_ma": a.limit, "slack_ticks": a.slack_ticks,
           "max_pull_by_servo": {str(k): v for k, v in max_pull_by_servo.items()},
           "slices": [], "stopped": None, "source": out_path.name}
    print("Boxes: " + ", ".join(f"{k}={v}" for k, v in boxes.items()), flush=True)

    def save():
        tuning_core.write_json_atomically(out_path, log)

    def picture(name):
        if camera is not None:
            try:
                camera.save(frames / f"{name}.jpg")
            except Exception as error:
                print(f"    no picture: {error}", flush=True)

    code = EXIT_FINISHED
    try:
        if not runner.start():
            return EXIT_NOT_STARTED
        jaw = int(boxes["jaw"])
        jaw_start = link.state.servos[jaw].present_tick
        targets = a.jaw_tick or [jaw_start + opening for opening in a.opening]
        pull_all = tuning_core.parse_pull_directions(boxes["pull"])
        mover = Probe(runner, jaw, pull_all, int(boxes["abort_ma"]), camera, frames,
                      a.jaw_abort_ma or int(boxes["abort_ma"]))
        for target in targets:
            opening = target - jaw_start
            if a.slack_ticks:
                state = link.state
                print(f"Jaw to {target}: slack every line by {a.slack_ticks}, move the jaw...", flush=True)
                reason = mover.lines_to({i: state.servos[i].goal_tick - sign * a.slack_ticks
                                         for i, sign in pull_all.items()}, True)
                reason = reason or mover.jaw_to(target, int(boxes["jaw_push_ma"]), f"to{target}")
                if reason is not None:
                    log["stopped"] = reason; code = EXIT_TORQUE_OFF
                    print(f"  stopped: {reason}", flush=True); break
            open_ticks = 0 if a.slack_ticks else target - link.state.servos[jaw].present_tick
            settings = runner.settings_for(open_ticks, 1)
            settings.defer_on_open = []            # every tendon to the preload at the slice
            problems = tuning_core.check_sweep_settings(settings, sorted(branches), a.limit, caps)
            if problems:
                log["stopped"] = "not started: " + " ".join(problems); code = EXIT_NOT_STARTED
                print(log["stopped"], flush=True); break
            print(f"Jaw {opening:+d} (tick {settings.jaw_open_tick}): pretension and neutral...",
                  flush=True)
            row, reason, torque_off = finder.slice_neutral(settings)
            if row is None:
                log["stopped"] = reason
                code = EXIT_TORQUE_OFF if torque_off else EXIT_HELD
                print(f"  stopped: {reason}", flush=True); break
            watched = [jaw, *settings.pull]
            state = link.state
            slice_log = {"jaw_opening": opening, "jaw_target": target,
                         "jaw_tick": state.servos[jaw].present_tick,
                         "jaw_ma": state.servos[jaw].current_ma, "tendons": {}}
            log["slices"].append(slice_log)
            print(f"  jaw at {slice_log['jaw_tick']} ({slice_log['jaw_ma']:+d} mA); neutral: "
                  + " ".join(f"{i}:{state.servos[i].present_tick}" for i in settings.pull), flush=True)
            picture(f"jaw{opening:+d}_neutral")
            if a.neutral_only:
                # The goal is what a driver is commanded to; the present tick
                # trails it by the preload's lag.
                slice_log["neutral"] = {
                    str(i): {"goal_tick": state.servos[i].goal_tick,
                             "present_tick": state.servos[i].present_tick,
                             "current_ma": state.servos[i].current_ma}
                    for i in settings.pull}
                save()
                continue
            for index, sign in settings.pull.items():
                neutral_goal = link.state.servos[index].goal_tick
                result = finder.find_end(index, sign, watched)
                slice_log["tendons"][str(index)] = result
                save()
                if "stopped" in result:
                    log["stopped"] = result["stopped"]; code = EXIT_TORQUE_OFF
                    print(f"  servo {index}: stopped: {result['stopped']}", flush=True); break
                print(f"  servo {index}: neutral {result['neutral_tick']} -> end {result['end_tick']} "
                      f"({result['range_ticks']:+d} ticks, {result['load_ma']:+d} mA, {result['reason']}); "
                      f"jaw {link.state.servos[jaw].present_tick}", flush=True)
                picture(f"jaw{opening:+d}_servo{index}_end")
                reason = finder.go_to_goal(index, neutral_goal, watched)
                if reason is not None:
                    log["stopped"] = reason; code = EXIT_TORQUE_OFF
                    print(f"  stopped: {reason}", flush=True); break
            save()
            if log["stopped"]:
                break
        return code
    finally:
        runner.disable()
        print(f"Torque off. Ranges: {out_path}", flush=True)
        save()
        if a.neutral_only and code == EXIT_FINISHED and log["slices"]:
            session_path = run / records_layout.NEUTRALS
            tuning_core.write_json_atomically(session_path, session_neutral(
                log, int(boxes["jaw"]), getattr(device, "session_id", None)))
            print(f"Session neutrals: {session_path}", flush=True)
        runner.close()
        if camera is not None:
            camera.close()
        try:
            device.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
