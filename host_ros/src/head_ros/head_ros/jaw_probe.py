"""How far can the jaw open once the modiolus lines are paid out?

Pays every listed line out by --payout ticks (in 25-tick steps, so friction is
beaten, each only once the horn has caught up; a line whose horn stays
PAYOUT_BLOCKED_LAG_TICKS behind its goal for PAYOUT_BLOCKED_S is left there),
then opens the jaw toward --open ticks (the jaw opens
toward LOWER ticks, so --open is negative) in 5-tick steps while it pushes
less than --jaw-push mA, until it gets there or stops moving for STALL_S. It
records jaw position against jaw load, closes the jaw again, reels the lines
back to where they started, and turns torque off.

Same session and stops as head-sweep (panel closed; Ctrl-C, SIGTERM or the
stop file turn torque off, as do back-off and board faults), plus torque off
when any line or the jaw reaches --abort mA. Writes jawprobe.json in its run folder and,
with --camera realsense, a picture every 25 jaw ticks.

    bench/.venv/bin/python jaw_probe.py --limit 300 --abort 280 --jaw-push 270 \
        --payout 500 --open -250 --camera realsense

Pulse mode (--pulse-ma, 2026-11-11; operator: "650 mA for 1 s should pull it
more wide open"): instead of the slow 5-tick opening, each listed push is one
short pulse. The jaw goal is kept PUSH / LAG_MA_PER_TICK ticks ahead of the
jaw (the servos are P-only, so the lead sets the push) for --pulse-s, then the
goal comes back to the jaw and the jaw closes to where it started; --rest-s
between pulses. Each pulse logs the jaw's widest tick, the time to reach 90 %
of it and the current. --payout 0 keeps the lines where they are (the
baseline to compare with). The pulse must end before the jaw peak guard
(--jaw-peak-s), and each push must stay under the jaw abort.
With --camera realsense the pulse mode records a video and a still at the end
of each pulse instead of the step pictures.

    scripts/head-jawprobe --limit 650 --abort 280 --jaw-abort-ma 615 --jaw-push 400 \
        --payout 300 --open -300 --pulse-ma 450 --pulse-ma 520 --pulse-ma 580 \
        --pulse-s 0.8 --jaw-peak-s 1.0 --camera realsense
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
except ImportError:  # run as a plain script from this directory
    import headctl  # type: ignore[no-redef]
    import records_layout  # type: ignore[no-redef]
    import tuning_core  # type: ignore[no-redef]
    from sweep_runner import DEFAULT_RECORDS, Runner, add_jaw_arguments, \
        session_limits  # type: ignore[no-redef]

LINE_STEP = 25
# Operator rule 2026-11-10: stop any run beyond 52 C (face_stream.TEMP_STOP_C).
TEMP_STOP_C = 52
# Largest tuning step the firmware takes (HEAD_TUNING_MAX_STEP_TICKS).
MAX_STEP_TICKS = 50
# Pulse mode: the jaw opens toward lower ticks (memory 2026-10-18; knee_test).
JAW_OPEN_DIRECTION = -1
PULSE_S_RANGE = (0.1, 1.0)
REST_S_RANGE = (1.0, 30.0)
# Closing after a pulse: the skin pulls the jaw shut; the goal leads by at
# most this push (it only has to beat friction).
CLOSE_PUSH_MA = 300
CLOSE_TIMEOUT_S = 3.0
# Jaw push per tick of goal lead during a pulse: run 122451 (2026-11-11) held
# 521 mA at 149 ticks and 600 mA at 173 (3.5 mA/tick; tuning_core's 3 is low).
PULSE_MA_PER_TICK = 3.5
# The jaw goal never goes below this on the head (face_stream.JAW_GOAL_MIN_TICK).
JAW_GOAL_MIN_TICK = 3600
# While paying out, a horn that lags its goal by this much (in the pay-out
# direction) is blocked: the line does not come off, or it is winding on the
# other way. Load in the pull direction is normal then (the skin pulls the
# line out); on 2026-10-20 a load stop wrongly halted 14 and 19 after 79 and
# 286 ticks.
# Blocked = at least one step behind for PAYOUT_BLOCKED_S. (Until 2026-10-20
# it took 100 ticks, so a horn stuck 25-99 ticks behind waited forever: servo 9
# held a range-finder run for minutes at 27 ticks.)
PAYOUT_BLOCKED_LAG_TICKS = LINE_STEP
PAYOUT_BLOCKED_S = 1.0
# Per-servo exceptions (servo -> ticks). Empty: line 9 had 40 for a few hours
# on 2026-10-22, until its "blocked" pay-outs turned out most likely to be its
# line winding back onto the horn the other way (negative holding load after
# a 600-tick pay-out) -- the case this rule exists for.
PAYOUT_BLOCKED_LAG_BY_SERVO: Dict[int, int] = {}
# Paying out only gives slack, so it uses the firmware's largest tuning step
# (HEAD_TUNING_MAX_STEP_TICKS, 50); taking in keeps LINE_STEP (2026-10-21).
PAYOUT_STEP = 50
# After a line move: settle this long before the caller goes on. All the
# stop checks keep running while it waits. Was 1 s until 2026-10-21.
LINES_SETTLE_S = 0.3
# No line move may take longer than this; it then stops and the run ends.
LINES_TIMEOUT_S = 30.0
JAW_STEP = 5
STALL_S = 3.0
PICTURE_EVERY_TICKS = 25


class Probe:
    def __init__(self, runner: Runner, jaw: int, pull: Dict[int, int], abort_ma: int,
                 camera, frames: Path, jaw_abort_ma: Optional[int] = None) -> None:
        self.runner, self.link = runner, runner.link
        self.jaw, self.pull, self.abort_ma = jaw, pull, abort_ma
        self.jaw_abort_ma = jaw_abort_ma or abort_ma
        self.camera, self.frames = camera, frames
        self.watched = [jaw, *pull]
        self._last_answer: Dict[int, float] = {}
        self.curve: List[dict] = []

    def pump(self) -> Optional[str]:
        reason = self.runner.pump()
        state = self.link.state
        while self.link.step_results:
            index, step, result = self.link.step_results.pop(0)
            self._last_answer[index] = self.link.clock()
            if self.runner.record is not None and state is not None:
                self.runner.record.add_step(state.servos[index], step, result)
        if reason is not None:
            return reason
        if state is None or not self.link.has_control or state.state != tuning_core.TUNING_STATE:
            return "the board left tuning"
        for servo in state.servos:
            if servo.temperature_c > TEMP_STOP_C:
                return f"servo {servo.index} at {servo.temperature_c} C (stop over {TEMP_STOP_C} C)"
        for index in self.watched:
            abort = self.jaw_abort_ma if index == self.jaw else self.abort_ma
            if abs(state.servos[index].current_ma) >= abort:
                return f"servo {index} at {state.servos[index].current_ma} mA (abort {abort})"
        return None

    def may_step(self, index: int) -> bool:
        state = self.link.state
        return (not self.link.step_in_flight and self.link.state_is_fresh()
                and state.received_monotonic_s > self._last_answer.get(index, float("-inf")))

    def picture(self, name: str) -> None:
        if self.camera is not None:
            try:
                self.camera.save(self.frames / f"{name}.jpg")
            except Exception as error:
                print(f"  no picture: {error}", flush=True)

    def wait(self, seconds: float) -> Optional[str]:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            reason = self.pump()
            if reason is not None:
                return reason
        return None

    def lines_to(self, goals: Dict[int, int], paying_out: bool,
                 settle_s: float = LINES_SETTLE_S) -> Optional[str]:
        left = dict(goals)
        behind_since: Dict[int, float] = {}
        deadline = time.monotonic() + LINES_TIMEOUT_S
        while left:
            reason = self.pump()
            if reason is not None:
                return reason
            if time.monotonic() > deadline:
                return (f"lines {sorted(left)} did not reach their goals in "
                        f"{LINES_TIMEOUT_S:.0f} s")
            state = self.link.state
            for index in list(left):
                servo = state.servos[index]
                # All of the trail counts here, the part the load explains
                # too: while paying out, a servo pushing in the pay-out
                # direction is pushing against something (a stuck horn, or a
                # line winding on the wrong way, where paying out pulls).
                lag_out = (servo.present_tick - servo.goal_tick) * self.pull[index]
                blocked_lag = PAYOUT_BLOCKED_LAG_BY_SERVO.get(index, PAYOUT_BLOCKED_LAG_TICKS)
                now = state.received_monotonic_s
                if paying_out and lag_out >= blocked_lag:
                    if now - behind_since.setdefault(index, now) >= PAYOUT_BLOCKED_S:
                        print(f"  servo {index} blocked while paying out (horn {lag_out} ticks "
                              f"behind for {PAYOUT_BLOCKED_S:g} s, {servo.current_ma:+d} mA) at "
                              f"{servo.present_tick}; left there", flush=True)
                        del left[index]
                        continue
                else:
                    behind_since.pop(index, None)
                error = left[index] - servo.goal_tick
                if error == 0:
                    del left[index]
                    continue
                # Step only once the horn has caught up, so the goal never
                # runs far ahead of a horn that cannot follow. A servo with a
                # larger allowance (PAYOUT_BLOCKED_LAG_BY_SERVO) creeps instead:
                # small steps that keep its goal at most that far ahead.
                step = PAYOUT_STEP if paying_out else LINE_STEP
                if paying_out and lag_out >= LINE_STEP:
                    if blocked_lag <= LINE_STEP or lag_out >= blocked_lag:
                        continue
                    step = blocked_lag - lag_out
                if self.may_step(index):
                    self.link.request_step(index, max(-step, min(step, error)))
                    break
        return self.wait(settle_s)

    def jaw_to(self, target: int, push_ma: int, label: str) -> Optional[str]:
        best, best_time, next_picture = None, time.monotonic(), None
        while True:
            reason = self.pump()
            if reason is not None:
                return reason
            jaw = self.link.state.servos[self.jaw]
            error = target - jaw.present_tick
            direction = 1 if error > 0 else -1
            if abs(error) <= tuning_core.HOME_ARRIVE_TICKS:
                print(f"  {label}: jaw reached {jaw.present_tick} at {jaw.current_ma:+d} mA", flush=True)
                return None
            if best is None or abs(error) < best:
                best, best_time = abs(error), time.monotonic()
                self.curve.append({"leg": label, "jaw_tick": jaw.present_tick,
                                   "jaw_ma": jaw.current_ma, "jaw_goal": jaw.goal_tick})
                if next_picture is None or (jaw.present_tick - next_picture) * direction >= 0:
                    print(f"  {label}: jaw {jaw.present_tick} {jaw.current_ma:+d} mA", flush=True)
                    self.picture(f"{label}_jaw{jaw.present_tick}")
                    next_picture = jaw.present_tick + direction * PICTURE_EVERY_TICKS
            if time.monotonic() - best_time > STALL_S:
                print(f"  {label}: jaw stopped moving at {jaw.present_tick}, {jaw.current_ma:+d} mA",
                      flush=True)
                return None
            push = jaw.current_ma * direction
            lead = (jaw.goal_tick - jaw.present_tick) * direction
            room = push_ma // tuning_core.LAG_MA_PER_TICK - lead
            if push < push_ma and room > 0 and self.may_step(self.jaw):
                self.link.request_step(self.jaw, direction * min(JAW_STEP, abs(error), room))

    def jaw_pulse(self, push_ma: int, pulse_s: float, open_limit: int, label: str,
                  still=None) -> tuple:
        """One pulse: keep the jaw goal push_ma / LAG_MA_PER_TICK ticks ahead
        of the jaw in the opening direction for pulse_s (never past
        open_limit), then bring the goal back to the jaw. Returns (stop
        reason or None, result dict)."""
        direction = JAW_OPEN_DIRECTION
        # The lead starts at push / PULSE_MA_PER_TICK and is trimmed down while
        # the jaw pushes harder than push_ma (it stops at a wall, so a fixed
        # lead overshoots: run 122451 pulse 2).
        want_lead = push_ma / PULSE_MA_PER_TICK
        state = self.link.state
        start = state.servos[self.jaw].present_tick
        samples: List[list] = []
        seen = float("-inf")
        t0 = time.monotonic()
        while time.monotonic() - t0 < pulse_s:
            reason = self.pump()
            if reason is not None:
                return reason, {"label": label, "push_ma": push_ma, "start": start, "samples": samples}
            state = self.link.state
            jaw = state.servos[self.jaw]
            if state.received_monotonic_s > seen:
                seen = state.received_monotonic_s
                samples.append([round(time.monotonic() - t0, 3), jaw.goal_tick, jaw.present_tick,
                                jaw.current_ma])
                push = jaw.current_ma * direction
                if push > push_ma:
                    want_lead = max(0.0, want_lead - max(2.0, (push - push_ma) / PULSE_MA_PER_TICK))
            goal = jaw.present_tick + direction * round(want_lead)
            if (goal - open_limit) * direction > 0:
                goal = open_limit
            error = goal - jaw.goal_tick
            if abs(error) > 2 and self.may_step(self.jaw):
                self.link.request_step(self.jaw, (1 if error > 0 else -1) * min(MAX_STEP_TICKS, abs(error)))
        if still is not None:
            still(label)
        result = {"label": label, "push_ma": push_ma, "pulse_s": pulse_s, "start": start,
                  "samples": samples}
        if samples:
            presents = [row[2] for row in samples]
            widest = min(presents) if direction < 0 else max(presents)
            opened = (widest - start) * direction
            t90 = next((row[0] for row in samples
                        if (row[2] - start) * direction >= 0.9 * opened), None) if opened > 0 else None
            late = [abs(row[3]) for row in samples if row[0] >= pulse_s / 2]
            result.update(widest=widest, opened_ticks=opened, t90_s=t90,
                          peak_ma=max(abs(row[3]) for row in samples),
                          late_mean_ma=round(sum(late) / len(late), 1) if late else None,
                          end_ma=samples[-1][3])
        # Close fast: the goal goes back to the start in the largest steps,
        # leading the jaw toward closed by at most CLOSE_PUSH_MA worth of
        # ticks (the skin closes the jaw; holding it wide for seconds would
        # trip the jaw hold guard).
        close_lead = CLOSE_PUSH_MA // tuning_core.LAG_MA_PER_TICK
        t_close = time.monotonic()
        while time.monotonic() - t_close < CLOSE_TIMEOUT_S:
            reason = self.pump()
            if reason is not None:
                return reason, result
            jaw = self.link.state.servos[self.jaw]
            if jaw.goal_tick == start and abs(jaw.present_tick - start) <= tuning_core.HOME_ARRIVE_TICKS:
                break
            # Closing direction is -direction; the goal may lead the jaw by close_lead.
            furthest = jaw.present_tick - direction * close_lead
            target = start if (start - furthest) * direction >= 0 else furthest
            step = target - jaw.goal_tick
            if step * -direction > 0 and self.may_step(self.jaw):
                self.link.request_step(self.jaw, (1 if step > 0 else -1) * min(MAX_STEP_TICKS, abs(step)))
        jaw = self.link.state.servos[self.jaw]
        result["closed"] = {"present": jaw.present_tick, "goal": jaw.goal_tick, "ma": jaw.current_ma,
                            "s": round(time.monotonic() - t_close, 3)}
        return None, result


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=headctl.AUTO_PORT)
    parser.add_argument("--records", type=Path, default=DEFAULT_RECORDS)
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--limit", type=int, required=True)
    parser.add_argument("--abort", type=int, required=True)
    parser.add_argument("--jaw-push", type=int, required=True)
    parser.add_argument("--payout", type=int, required=True)
    parser.add_argument("--open", type=int, required=True, help="jaw ticks to open (negative)")
    parser.add_argument("--lines", help="pull directions, default from the sweep boxes")
    parser.add_argument("--camera", choices=("none", "realsense"), default="none")
    parser.add_argument("--pulse-ma", type=int, action="append", default=[],
                        help="pulse mode: one pulse at this jaw push (repeat for a ladder)")
    parser.add_argument("--pulse-s", type=float, default=0.8,
                        help=f"pulse length, {PULSE_S_RANGE[0]}-{PULSE_S_RANGE[1]} s, under --jaw-peak-s")
    parser.add_argument("--rest-s", type=float, default=5.0, help="rest between pulses (s)")
    add_jaw_arguments(parser)
    raw = list(sys.argv[1:] if argv is None else argv)
    glued: List[str] = []
    for item in raw:   # argparse takes "-250" for an option name; glue it to its flag
        if glued and glued[-1] == "--open" and item.startswith("-"):
            glued[-1] = "--open=" + item
        else:
            glued.append(item)
    a = parser.parse_args(glued)
    jaw_abort = a.jaw_abort_ma or a.abort
    if not a.jaw_push < jaw_abort:
        print("Need jaw push < jaw abort."); return 4
    one_turn = tuning_core.headctl_one_turn_ticks()
    pulse_mode = bool(a.pulse_ma)
    if not ((0 if pulse_mode else 1) <= a.payout <= one_turn and 0 < abs(a.open) <= one_turn):
        print("Payout and opening must be 1 to one turn (payout 0 is allowed with --pulse-ma)."); return 4
    if pulse_mode:
        if a.open >= 0:
            print("--open must be negative (the jaw opens toward lower ticks)."); return 4
        if not PULSE_S_RANGE[0] <= a.pulse_s <= PULSE_S_RANGE[1]:
            print(f"--pulse-s must be {PULSE_S_RANGE[0]}-{PULSE_S_RANGE[1]} s."); return 4
        if not a.pulse_s < a.jaw_peak_s:
            print(f"--pulse-s must be shorter than the jaw peak guard (--jaw-peak-s {a.jaw_peak_s:g})."); return 4
        if not REST_S_RANGE[0] <= a.rest_s <= REST_S_RANGE[1]:
            print(f"--rest-s must be {REST_S_RANGE[0]:g}-{REST_S_RANGE[1]:g} s."); return 4
        if any(not 0 < p < jaw_abort for p in a.pulse_ma):
            print(f"Each --pulse-ma must be 1 to below the jaw abort ({jaw_abort} mA)."); return 4

    records = a.records / "preview" if a.preview else a.records
    boxes = (tuning_core.PREVIEW_SWEEP_BOXES if a.preview else tuning_core.load_sweep_boxes(records))
    pull = tuning_core.parse_pull_directions(a.lines or boxes["pull"])
    jaw = int(boxes["jaw"])
    stop_file = records / "STOP"
    if stop_file.exists():
        print(f"Remove the stop file first: {stop_file}"); return 4

    camera = None
    if a.camera == "realsense":
        try:
            from .face_camera import RealSenseCamera  # optional
        except ImportError:
            from face_camera import RealSenseCamera  # type: ignore[no-redef]
        camera = RealSenseCamera()
    device = tuning_core.preview_controller() if a.preview else headctl.Device(a.port)
    link = tuning_core.TuningLink(device)
    link.connect()
    info = headctl.decode_info(device.get(headctl.GET_INFO, headctl.INFO))
    branches = headctl.read_servo_branches(device, info["active_servo_mask"])
    caps, guard = session_limits(device, info, a.limit, jaw, a)
    line_cap = min(caps.get(index, a.limit) for index in pull)
    if pulse_mode and max(a.pulse_ma) >= tuning_core.BACKOFF_LOAD_FRACTION * caps.get(jaw, a.limit):
        print(f"Each --pulse-ma must be under 95 % of the jaw's cap ({caps.get(jaw, a.limit)} mA).")
        device.close()
        return 4
    if not (a.abort < tuning_core.BACKOFF_LOAD_FRACTION * line_cap and
            jaw_abort < tuning_core.BACKOFF_LOAD_FRACTION * caps.get(jaw, a.limit)):
        print(f"Need abort < 95 % of the lines' cap ({line_cap} mA) and jaw abort < 95 % of "
              f"the jaw's cap ({caps.get(jaw, a.limit)} mA).")
        device.close()
        return 4
    run = records_layout.run_dir(records, "jawprobe")
    print(f"Run folder: {run}", flush=True)
    runner = Runner(link, branches, run, a.limit, boxes, stop_file,
                    say=lambda text: print(text, flush=True), caps=caps, jaw_guard=guard,
                    jaw_abort_ma=a.jaw_abort_ma)
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda signum, _f: runner.request_stop(signal.Signals(signum).name))
    out_path = run / "jawprobe.json"
    recorder = None
    if pulse_mode and camera is not None:
        try:
            from .face_camera import Recorder
        except ImportError:
            from face_camera import Recorder  # type: ignore[no-redef]
        recorder = Recorder(camera, run / records_layout.VIDEO)   # the recorder owns the camera now
    probe = Probe(runner, jaw, pull, a.abort, None if recorder else camera, run / "pictures",
                  jaw_abort)

    def still(label: str) -> None:
        if recorder is not None:
            recorder.still(run / records_layout.STILLS / f"{label}.jpg")
    log = {"format": "head-jaw-probe-1", "started_utc": datetime.now(timezone.utc).isoformat(),
           "limit_ma": a.limit, "abort_ma": a.abort, "jaw_push_ma": a.jaw_push,
           "payout_ticks": a.payout, "open_ticks": a.open, "jaw_abort_ma": jaw_abort,
           "caps": {str(k): v for k, v in caps.items()},
           "pull": {str(k): v for k, v in pull.items()}, "stopped": None,
           "pulse": {"push_ma": a.pulse_ma, "pulse_s": a.pulse_s, "rest_s": a.rest_s,
                     "jaw_peak_s": a.jaw_peak_s, "pulses": []} if pulse_mode else None,
           "temp_stop_c": TEMP_STOP_C}
    reason = None
    try:
        if not runner.start():
            return 4
        reason = probe.wait(1.0)
        state = link.state
        start = {i: state.servos[i].goal_tick for i in [jaw, *pull]}
        log["start"] = {str(i): {"goal": state.servos[i].goal_tick, "present": state.servos[i].present_tick,
                                 "ma": state.servos[i].current_ma} for i in start}
        print("start: " + " ".join(f"{i}:{state.servos[i].present_tick}/{state.servos[i].current_ma:+d}"
                                   for i in start), flush=True)
        probe.picture("0_start")
        if a.payout > 0:
            reason = reason or probe.lines_to({i: start[i] - pull[i] * a.payout for i in pull}, True)
        state = link.state
        log["paid_out"] = {str(i): {"present": state.servos[i].present_tick, "ma": state.servos[i].current_ma}
                           for i in pull}
        print("lines paid out: " + " ".join(f"{i}:{state.servos[i].present_tick}/{state.servos[i].current_ma:+d}"
                                            for i in pull), flush=True)
        probe.picture("1_lines_out")
        still("1_lines_out")
        if pulse_mode:
            open_limit = start[jaw] + a.open
            if not a.preview and open_limit < JAW_GOAL_MIN_TICK:
                reason = (f"the jaw goal would go to {open_limit}, below {JAW_GOAL_MIN_TICK}; "
                          "give a smaller --open")
            for number, push in enumerate(a.pulse_ma):
                if reason is not None:
                    break
                label = f"pulse{number + 1}_{push}ma"
                print(f"{label}: {a.pulse_s:g} s, jaw goal up to {push / PULSE_MA_PER_TICK:.0f} "
                      f"ticks ahead, trimmed to keep the push at {push} mA (not past {open_limit})",
                      flush=True)
                reason, result = probe.jaw_pulse(push, a.pulse_s, open_limit, label, still)
                log["pulse"]["pulses"].append(result)
                if "widest" in result:
                    print(f"  widest {result['widest']} = {result['opened_ticks']} ticks open, 90 % in "
                          f"{result['t90_s']} s, peak {result['peak_ma']} mA, late mean "
                          f"{result['late_mean_ma']} mA", flush=True)
                if "closed" in result:
                    print(f"  closed to {result['closed']['present']} in {result['closed']['s']} s "
                          f"({result['closed']['ma']:+d} mA)", flush=True)
                if reason is None and abs(link.state.servos[jaw].present_tick - start[jaw]) > \
                        tuning_core.HOME_ARRIVE_TICKS:
                    reason = probe.jaw_to(start[jaw], CLOSE_PUSH_MA, f"close{number + 1}")
                reason = reason or probe.wait(a.rest_s)
            reason = reason or probe.lines_to({i: start[i] for i in pull}, False)
            still("3_end")
            return 3 if reason else 0
        reason = reason or probe.jaw_to(link.state.servos[jaw].present_tick + a.open, a.jaw_push, "open")
        if reason is None:
            jaw_now = link.state.servos[jaw]
            log["widest"] = {"jaw_tick": jaw_now.present_tick, "jaw_ma": jaw_now.current_ma,
                             "opened_ticks": jaw_now.present_tick - start[jaw]}
            probe.picture("2_widest")
            reason = probe.wait(1.0)
        reason = reason or probe.jaw_to(start[jaw], a.jaw_push, "close")
        reason = reason or probe.lines_to({i: start[i] for i in pull}, False)
        probe.picture("3_end")
        return 3 if reason else 0
    finally:
        log["stopped"] = reason
        log["curve"] = probe.curve
        runner.disable()
        tuning_core.write_json_atomically(out_path, log)
        print(f"Torque off: {reason or 'end of probe'}. {out_path}", flush=True)
        runner.close()
        if recorder is not None:
            recorder.close()
            log["video"] = str(recorder.video_path)
            tuning_core.write_json_atomically(out_path, log)
        elif camera is not None:
            camera.close()
        try:
            device.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
