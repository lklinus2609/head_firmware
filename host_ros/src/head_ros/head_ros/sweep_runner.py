"""Jaw sweeps from the command line, without the tuning panel.

Same logic, checks and files as box 3 of the tuning panel
(tuning_core.JawSweep): a tuning record, its telemetry CSV, and one
<date>_<time>_jawsweep.json per sweep in its run folder (runs/<date>/<time>_sweep/). The panel must be closed: only
one program can hold the port.

Each --sweep is OPEN:STAGES. OPEN is in jaw ticks from where the jaw is when
that sweep starts (the jaw opens toward LOWER ticks on the head, so opening is
negative; 0 is pretension only). The sweeps run one after the other in one
tuning session; the next starts only if the one before finished.

Stops: Ctrl-C, SIGTERM, or creating the stop file (default <records>/STOP)
request the verified torque-off path. A lost USB link or a killed process lets
the 1 s lease run out, which turns torque off too. Keep the 12 V switch near.

    scripts/head-sweep --limit 400 --sweep 0:1 --sweep -80:4
    scripts/head-sweep --limit 400 --sweep -200:8 --camera realsense
    scripts/head-sweep --preview --limit 600 --sweep 0:1 --sweep 40:2
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import sys
import time
from typing import Callable, Dict, List, Optional

try:
    from . import headctl, records_layout, tuning_core
except ImportError:  # run as a plain script from this directory
    import headctl  # type: ignore[no-redef]
    import records_layout  # type: ignore[no-redef]
    import tuning_core  # type: ignore[no-redef]

DEFAULT_RECORDS = Path(__file__).resolve().parents[4] / "tuning_runs"
POLL_S = 0.005
START_TIMEOUT_S = 3.0
EXIT_FINISHED, EXIT_HELD, EXIT_TORQUE_OFF, EXIT_NOT_STARTED = 0, 2, 3, 4


def add_jaw_arguments(parser: argparse.ArgumentParser) -> None:
    """The jaw's own abort load and its load-over-time guard."""
    parser.add_argument("--jaw-abort-ma", type=int,
                        help="the jaw's abort load (default: the lines' abort load)")
    parser.add_argument("--jaw-hold-ma", type=int, default=tuning_core.JAW_HOLD_MA)
    parser.add_argument("--jaw-hold-s", type=float, default=tuning_core.JAW_HOLD_S)
    parser.add_argument("--jaw-peak-ma", type=int, default=tuning_core.JAW_PEAK_MA)
    parser.add_argument("--jaw-peak-s", type=float, default=tuning_core.JAW_PEAK_S)


def session_limits(device, info: dict, limit_ma: int, jaw: int, args):
    """(caps, jaw guard): each servo's tuning cap read from the board's
    profile, printed, and the jaw guard from the arguments."""
    operating = tuning_core.read_operating_currents(device, info["active_servo_mask"])
    caps = tuning_core.tuning_caps(limit_ma, operating)
    print("Caps (min of limit and operating_current_ma): "
          + " ".join(f"{index}:{cap}" for index, cap in sorted(caps.items())), flush=True)
    guard = tuning_core.JawGuard(jaw, args.jaw_hold_ma, args.jaw_hold_s,
                                 args.jaw_peak_ma, args.jaw_peak_s)
    return caps, guard


def parse_sweep_spec(text: str) -> tuple[int, int]:
    """'-80:4' -> (-80, 4)."""
    try:
        open_ticks, stages = (int(part) for part in text.split(":"))
    except ValueError:
        raise argparse.ArgumentTypeError(f"'{text}' is not OPEN:STAGES, like -80:4")
    if stages < 1:
        raise argparse.ArgumentTypeError("STAGES must be at least 1")
    if abs(open_ticks) > tuning_core.headctl_one_turn_ticks():
        raise argparse.ArgumentTypeError("OPEN may not be more than one turn")
    return open_ticks, stages


class Runner:
    """One tuning session that runs jaw sweeps one after the other."""

    def __init__(self, link: tuning_core.TuningLink, branches: Dict[int, int],
                 records: Path, limit_ma: int, boxes: Dict[str, str],
                 stop_file: Path, camera=None, say: Callable[[str], None] = print,
                 markers: Optional[dict] = None, caps: Optional[Dict[int, int]] = None,
                 jaw_guard: Optional[tuning_core.JawGuard] = None,
                 jaw_abort_ma: Optional[int] = None) -> None:
        self.link = link
        # Servo -> its tuning cap (tuning_core.tuning_caps); back-off uses each
        # servo's own cap. Without it every servo has limit_ma.
        self.caps = caps or {}
        self.jaw_guard = jaw_guard
        self.jaw_abort_ma = jaw_abort_ma
        self.branches = branches
        self.records = Path(records)
        self.limit_ma = limit_ma
        self.boxes = boxes
        self.stop_file = Path(stop_file)
        self.camera = camera
        # Marker starting points (face_markers.load_markers); each sweep gets
        # its own tracker, whose zero is that sweep's "before" picture.
        self.markers = markers
        self.tracker = None
        self.marker_log: Optional[dict] = None
        self.say = say
        self.stop_reason: Optional[str] = None
        self.record: Optional[tuning_core.TuningRecord] = None
        self.telemetry: Optional[tuning_core.TelemetryLog] = None
        self.backoff = tuning_core.BackOff()

    # Session -----------------------------------------------------------------
    def request_stop(self, reason: str) -> None:
        if self.stop_reason is None:
            self.stop_reason = reason

    def start(self) -> bool:
        self.link.poll(0.2)
        state = self.link.state
        if state is None or state.state != tuning_core.HOMING_REQUIRED_STATE:
            self.say("Not started: the board is "
                     + (state.state_name if state else "not sending state")
                     + ", not HOMING_REQUIRED. Close the panel, or clear the fault first.")
            return False
        self.link.start(self.limit_ma)
        # self.records is the run's folder (records_layout.run_dir).
        self.record = tuning_core.TuningRecord(tuning_core.new_record_path(self.records, "tuning.json"),
                                               self.limit_ma, dict(self.branches))
        self.record.save()
        try:
            self.telemetry = tuning_core.TelemetryLog(self.record.telemetry_path,
                                                      self.record.servo_branches)
            self.link.sample_listener = self.telemetry.observe
        except OSError as error:
            self.say(f"No telemetry log: {error}")
        deadline = time.monotonic() + START_TIMEOUT_S
        while time.monotonic() < deadline:
            self.link.poll(0.05)
            if self.link.state is not None and self.link.state.state == tuning_core.TUNING_STATE:
                self.say(f"Tuning at {self.limit_ma} mA. Record: {self.record.path.name}")
                return True
        self.say("Not started: the board did not reach TUNING.")
        return False

    def disable(self) -> None:
        try:
            self.link.disable()
        except Exception as error:  # the link may already be gone
            self.say(f"Torque off could not be sent ({error}); the lease runs out in 1 s.")

    def close(self) -> None:
        if self.record is not None:
            self.record.save()
        self.link.sample_listener = None
        if self.telemetry is not None:
            self.telemetry.close()
        if self.camera is not None:
            self.camera.close()

    def snapshot(self, sweep: tuning_core.JawSweep, name: str) -> None:
        if self.camera is None:
            return
        path = sweep.path.with_name(sweep.path.stem + "_frames") / f"{name}.jpg"
        try:
            image, _ = self.camera.capture()
            self.camera.save(path, image)
            self.say(f"  picture: {path}")
        except Exception as error:  # a camera fault never stops a sweep
            self.say(f"  no picture: {error}")
            return
        if self.markers is not None:
            self.track_markers(sweep, name, image)

    def track_markers(self, sweep: tuning_core.JawSweep, name: str, image) -> None:
        import face_markers  # optional, needs numpy

        log_path = sweep.path.with_name(sweep.path.stem + "_markers.json")
        if self.marker_log is None or self.marker_log.get("sweep") != sweep.path.name:
            self.tracker = face_markers.MarkerTracker(self.markers)
            self.marker_log = {"format": "head-face-marker-track-1", "sweep": sweep.path.name,
                               "markers": self.markers, "pictures": []}
        found = self.tracker.track(image)
        moved = self.tracker.displacement(found)
        self.marker_log["pictures"].append({
            "name": name, "utc": datetime.now(timezone.utc).isoformat(),
            "found": found, "displacement": moved})
        try:
            tuning_core.write_json_atomically(log_path, self.marker_log)
        except OSError as error:
            self.say(f"  markers not saved: {error}")
        parts = []
        for marker, entry in self.markers.items():
            if entry["reference"]:
                continue
            shift = moved[marker]["px"]
            parts.append(f"{marker} LOST" if shift is None
                         else f"{marker} {shift[0]:+.1f},{shift[1]:+.1f}")
        lost_references = [m for m, e in self.markers.items()
                           if e["reference"] and found[m].get("lost")]
        self.say("  markers (px from before, + = right/down): " + ", ".join(parts)
                 + (f" | reference lost: {lost_references}" if lost_references else ""))

    # One poll ------------------------------------------------------------------
    def pump(self) -> Optional[str]:
        """Read the board once. Returns a reason to turn torque off, if any."""
        self.link.poll(POLL_S)
        # The serial read returns at once when nothing is waiting; do not spin.
        time.sleep(0.001)
        state = self.link.state
        if self.stop_file.exists():
            self.request_stop(f"stop file {self.stop_file} found")
        if self.stop_reason is not None:
            return self.stop_reason
        if self.link.control_lost_reason:
            if self.record is not None and state is not None and \
                    state.state == tuning_core.FAULT_STATE:
                self.record.add_fault(state)
            return self.link.control_lost_reason
        if state is not None and self.record is not None:
            self.record.observe(state)
            limits = {index: self.caps.get(index, self.limit_ma) for index in self.branches}
            started = self.backoff.observe(state, limits, sorted(self.branches))
            if started:
                return (f"servo {', '.join(map(str, started))} pushed at the load limit "
                        "and did not move (back-off)")
            if self.jaw_guard is not None and state.state == tuning_core.TUNING_STATE:
                verdict = self.jaw_guard.observe(state)
                guard = self.jaw_guard
                if verdict == "torque off":
                    return f"jaw over {guard.peak_ma} mA for {guard.peak_s:g} s"
                if verdict == "back off":
                    self.back_off_jaw()
                    return f"jaw held over {guard.hold_ma} mA for {guard.hold_s:g} s; backed off"
        return None

    def back_off_jaw(self) -> None:
        """Step the jaw's goal back toward where the jaw is, so it pushes less,
        for up to 3 s (the skin then closes it a little)."""
        jaw = self.jaw_guard.jaw
        end = time.monotonic() + 3.0
        last_sample = float("-inf")
        while time.monotonic() < end:
            self.link.poll(POLL_S)
            time.sleep(0.001)
            self.take_results(None)
            state = self.link.state
            if state is None or not self.link.has_control:
                return
            servo = state.servos[jaw]
            lead = servo.goal_tick - servo.present_tick
            if abs(lead) <= tuning_core.BACKOFF_LEAD_TICKS and \
                    abs(servo.current_ma) < self.jaw_guard.hold_ma:
                break
            if not self.link.step_in_flight and self.link.state_is_fresh() and \
                    state.received_monotonic_s > last_sample:
                step = -max(-10, min(10, lead))
                if step:
                    self.link.request_step(jaw, step)
                    last_sample = state.received_monotonic_s
        state = self.link.state
        if state is not None:
            self.say(f"  jaw backed off: now {state.servos[jaw].present_tick}, "
                     f"{state.servos[jaw].current_ma:+d} mA")

    def take_results(self, sweep: Optional[tuning_core.JawSweep]) -> None:
        while self.link.step_results:
            index, step, result = self.link.step_results.pop(0)
            servo = self.link.state.servos[index] if self.link.state else None
            if self.record is not None and servo is not None:
                self.record.add_step(servo, step, result)
            if sweep is not None:
                sweep.on_result(index, result, self.link.clock())

    # Sweeps ----------------------------------------------------------------------
    def settings_for(self, open_ticks: int, stages: int) -> tuning_core.SweepSettings:
        boxes = self.boxes
        jaw = int(boxes["jaw"])
        jaw_now = self.link.state.servos[jaw].present_tick
        return tuning_core.SweepSettings(
            jaw=jaw, jaw_open_tick=jaw_now + open_ticks, stages=stages,
            pull=tuning_core.parse_pull_directions(boxes["pull"]),
            release=tuning_core.parse_servo_list(boxes["release"]),
            preload_ma=int(boxes["preload_ma"]), band_ma=int(boxes["band_ma"]),
            jaw_push_ma=int(boxes["jaw_push_ma"]), abort_ma=int(boxes["abort_ma"]),
            max_travel_ticks=int(boxes["max_travel_ticks"]),
            defer_on_open=tuning_core.parse_servo_list(boxes["defer_on_open"]),
            jaw_abort_ma=self.jaw_abort_ma)

    def run_sweep(self, open_ticks: int, stages: int) -> int:
        settings = self.settings_for(open_ticks, stages)
        problems = tuning_core.check_sweep_settings(settings, sorted(self.branches),
                                                    self.limit_ma, self.caps)
        if problems:
            self.say("Sweep not started: " + " ".join(problems))
            return EXIT_NOT_STARTED
        sweep = tuning_core.JawSweep(tuning_core.new_sweep_path(self.records), settings,
                                     self.link.state, self.link.clock())
        sweep.save()
        self.say(f"Sweep {sweep.path.name}: jaw {sweep.targets[0]} -> {settings.jaw_open_tick} "
                 f"({open_ticks:+d} ticks) in {stages} stage(s) and back.")
        self.snapshot(sweep, "before")
        while sweep.active:
            reason = self.pump()
            if reason is not None:
                sweep.save()
                self.say(f"Torque off: {reason}. {len(sweep.rows)} stage(s) saved.")
                self.disable()
                return EXIT_TORQUE_OFF
            self.take_results(sweep)
            if not self.link.has_control or self.link.state.state != tuning_core.TUNING_STATE:
                sweep.save()
                self.say("Stopped: the board left tuning.")
                self.disable()
                return EXIT_TORQUE_OFF
            row = sweep.observe(self.link.state)
            if row is not None:
                sweep.save()
                self.say(self.row_text(sweep, row))
                self.snapshot(sweep, f"stage_{row['stage']:02d}_{row['leg']}")
            if sweep.stopped is None and not self.link.step_in_flight and \
                    self.link.state_is_fresh():
                step = sweep.next_step(self.link.state)
                if step is not None:
                    self.link.request_step(*step)
            # A stop can come from observe() or from next_step() (travel limit).
            if sweep.stopped is not None:
                sweep.save()
                if sweep.torque_off:
                    self.say(f"Torque off: {sweep.stopped}")
                    self.disable()
                    return EXIT_TORQUE_OFF
                self.say(f"Sweep stopped, holding: {sweep.stopped} "
                         f"{len(sweep.rows)} stage(s) saved.")
                return EXIT_HELD
        sweep.save()
        self.say(f"Sweep finished: {len(sweep.rows)} stages in {sweep.path.name}.")
        return EXIT_FINISHED

    @staticmethod
    def row_text(sweep: tuning_core.JawSweep, row: dict) -> str:
        servos = row["servos"]
        jaw = str(sweep.settings.jaw)
        tendons = " ".join(f"{index}:{row['taken_in_ticks'][index]:+d}t/"
                           f"{servos[index]['current_ma']:+d}mA"
                           for index in row["taken_in_ticks"])
        text = (f"  stage {row['stage']} {row['leg']}: jaw {servos[jaw]['present_tick']} "
                f"{servos[jaw]['current_ma']:+d}mA | taken in / load: {tendons}")
        if row["slack"]:
            text += f" | left slack: {row['slack']}"
        if row["unverified"]:
            text += f" | UNVERIFIED (hardly moved): {row['unverified']}"
        return text

    def hold(self) -> int:
        self.say("Holding. Ctrl-C, SIGTERM or the stop file turns torque off.")
        while True:
            reason = self.pump()
            self.take_results(None)
            if reason is not None:
                self.say(f"Torque off: {reason}.")
                self.disable()
                return EXIT_TORQUE_OFF


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=headctl.AUTO_PORT)
    parser.add_argument("--records", type=Path, default=DEFAULT_RECORDS)
    parser.add_argument("--preview", action="store_true",
                        help="simulated board, records in <records>/preview")
    parser.add_argument("--limit", type=int, required=True, help="tuning load limit (mA)")
    parser.add_argument("--sweep", type=parse_sweep_spec, action="append", required=True,
                        metavar="OPEN:STAGES")
    parser.add_argument("--after", choices=("off", "hold"), default="off",
                        help="when the sweeps end: torque off (default) or hold until stopped")
    parser.add_argument("--camera", choices=("none", "realsense"), default="none")
    parser.add_argument("--stop-file", type=Path)
    add_jaw_arguments(parser)
    for key in tuning_core.SWEEP_BOX_DEFAULTS:
        parser.add_argument("--" + key.replace("_", "-"), dest="box_" + key,
                            help=f"override the {key} box")
    # argparse takes "-80:4" for an option name; glue it to its flag first.
    raw = list(sys.argv[1:] if argv is None else argv)
    glued: List[str] = []
    for item in raw:
        if glued and glued[-1] == "--sweep" and item.startswith("-"):
            glued[-1] = "--sweep=" + item
        else:
            glued.append(item)
    arguments = parser.parse_args(glued)

    records = arguments.records / "preview" if arguments.preview else arguments.records
    boxes = (dict(tuning_core.PREVIEW_SWEEP_BOXES) if arguments.preview
             else tuning_core.load_sweep_boxes(records))
    for key in tuning_core.SWEEP_BOX_DEFAULTS:
        value = getattr(arguments, "box_" + key)
        if value is not None:
            boxes[key] = value
    stop_file = arguments.stop_file or records / "STOP"
    if stop_file.exists():
        print(f"Remove the stop file first: {stop_file}")
        return EXIT_NOT_STARTED

    camera = None
    markers = None
    if arguments.camera == "realsense":
        from face_camera import RealSenseCamera  # optional, see face_camera.py
        camera = RealSenseCamera()
        try:
            import face_markers
            markers = face_markers.load_markers(records_layout.config_dir(records))
            print("Markers: " + ", ".join(markers), flush=True)
        except FileNotFoundError:
            print("No face_markers.json in the records' config/: pictures only.", flush=True)

    if arguments.preview:
        device = tuning_core.preview_controller()
    else:
        device = headctl.Device(arguments.port)
    link = tuning_core.TuningLink(device)
    link.connect()
    info = headctl.decode_info(device.get(headctl.GET_INFO, headctl.INFO))
    branches = headctl.read_servo_branches(device, info["active_servo_mask"])
    caps, guard = session_limits(device, info, arguments.limit, int(boxes["jaw"]), arguments)
    run = records_layout.run_dir(records, "sweep")
    print(f"Run folder: {run}", flush=True)
    runner = Runner(link, branches, run, arguments.limit, boxes, stop_file,
                    camera=camera, say=lambda text: print(text, flush=True), markers=markers,
                    caps=caps, jaw_guard=guard, jaw_abort_ma=arguments.jaw_abort_ma)

    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda signum, _frame:
                      runner.request_stop(signal.Signals(signum).name))
    print("Boxes: " + ", ".join(f"{key}={value}" for key, value in boxes.items()), flush=True)
    exit_code = EXIT_NOT_STARTED
    try:
        if not runner.start():
            return EXIT_NOT_STARTED
        for open_ticks, stages in arguments.sweep:
            exit_code = runner.run_sweep(open_ticks, stages)
            if exit_code != EXIT_FINISHED:
                break
        if exit_code == EXIT_TORQUE_OFF:
            return exit_code
        if arguments.after == "hold":
            return runner.hold()
        runner.disable()
        print("Torque off: end of run.", flush=True)
        return exit_code
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
