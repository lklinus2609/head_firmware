"""Range-of-motion tour for the eyes and eyelids, recorded.

Moves only the eye servos (0 right eye, 1 eye pitch, 2 left eye) and the
eyelids (5 right lid, 6 left lid; servo_notes.json), and only between the
operator's tuning-panel marks on this head (tuning_core.HEAD_MARKS_SINCE):
eyes need neutral, end_1 and end_2; lids need neutral and end_1 (closed). It
refuses to start when a mark is missing, or when a servo sits more than
STALE_MARK_TICKS outside its marks (a turn-count jump, or stale marks).

The firmware enforces the same limits: a session envelope is uploaded first
(each eye/lid servo's min..max = its marks, plus where it sits now, plus
ENVELOPE_MARGIN_TICKS), so every target is clamped by the board as well.
All other servos hold their pose from Enable.

Tour: each servo alone (neutral -> end_1 -> end_2 -> neutral; lids neutral ->
closed -> neutral), both eyes to end_1 and end_2 together, pitch with both,
then both lids blink twice. Moves are speed-limited (--speed ticks/s).

Stops (torque off): any servo at --abort-ma, a board fault, leaving ENABLED,
stale STATE, Ctrl-C/SIGTERM, or the stop file (<records>/STOP). The panel must
be closed. With --camera realsense: video.webm and a still at every pose.

    scripts/head-eyes --speed 1000 --camera realsense
    scripts/head-eyes --preview --speed 2500
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import math
from pathlib import Path
import signal
import sys
import time
from typing import Dict, List, Optional, Tuple

try:
    from . import face_stream as fs, headctl, records_layout, tuning_core
except ImportError:  # run as a plain script from this directory
    import face_stream as fs  # type: ignore[no-redef]
    import headctl  # type: ignore[no-redef]
    import records_layout  # type: ignore[no-redef]
    import tuning_core  # type: ignore[no-redef]

EYE_SERVOS = {0: "right eye", 1: "eye pitch", 2: "left eye"}
LID_SERVOS = {5: "right lid", 6: "left lid"}
TOUR_SERVOS = {**EYE_SERVOS, **LID_SERVOS}
ENVELOPE_MARGIN_TICKS = 20
STALE_MARK_TICKS = 300
DEFAULT_ABORT_MA = 280
HOLD_S = 0.4
MOVE_TIMEOUT_S = 6.0
ARRIVE_TICKS = 30
SPEED_MAX_TPS = fs.SPEED_MAX_TPS
EXIT_FINISHED, EXIT_STOPPED, EXIT_NOT_STARTED = 0, 3, 4


class SetupError(Exception):
    pass


def tour_limits(marks: Dict[int, Dict[str, dict]],
                servos: Optional[List[int]] = None) -> Dict[int, dict]:
    """servo -> {"neutral", "ends": [end_1, end_2?], "low", "high"} from the
    panel marks, for `servos` (default all of TOUR_SERVOS). SetupError when a
    mark is missing or an end is over a turn from neutral."""
    limits: Dict[int, dict] = {}
    missing: List[str] = []
    for index, name in TOUR_SERVOS.items():
        if servos is not None and index not in servos:
            continue
        servo_marks = marks.get(index) or {}
        neutral = (servo_marks.get("neutral") or {}).get("present_tick")
        names = ("end_1", "end_2") if index in EYE_SERVOS else ("end_1",)
        ends = [(servo_marks.get(n) or {}).get("present_tick") for n in names]
        if neutral is None or any(end is None for end in ends):
            missing.append(f"{index} ({name}): needs neutral + {' + '.join(names)}")
            continue
        neutral, ends = int(neutral), [int(end) for end in ends]
        far = [end for end in ends if abs(end - neutral) > headctl_one_turn()]
        if far:
            raise SetupError(f"servo {index}: end mark {far} is over one turn from neutral "
                             f"{neutral}; re-mark it")
        limits[index] = {"neutral": neutral, "ends": ends,
                         "low": min(neutral, *ends), "high": max(neutral, *ends)}
    if missing:
        raise SetupError("Missing panel marks (since " + tuning_core.HEAD_MARKS_SINCE + "): "
                         + "; ".join(missing) + ". Mark them in the tuning panel first.")
    return limits


def headctl_one_turn() -> int:
    return tuning_core.headctl_one_turn_ticks()


def check_present(limits: Dict[int, dict], present: Dict[int, int]) -> None:
    """SetupError when a servo sits far outside its marks."""
    for index, limit in limits.items():
        tick = present[index]
        outside = max(limit["low"] - tick, tick - limit["high"], 0)
        if outside > STALE_MARK_TICKS:
            raise SetupError(f"servo {index} sits at {tick}, {outside} ticks outside its marks "
                             f"{limit['low']}..{limit['high']}: the marks are stale (or the turn "
                             "count jumped); re-mark it in the panel")


def tour_envelope(profile: dict, limits: Dict[int, dict], present: Dict[int, int],
                  speed_tps: float) -> Tuple[dict, List[str]]:
    """A copy of the board profile: each tour servo's min..max is its marks
    (and where it sits now) plus ENVELOPE_MARGIN_TICKS, home its neutral, and
    its per-cycle step large enough for speed_tps. Others keep their range
    when they sit inside it, else get fs.OTHER_WINDOW_TICKS around where they
    are (they only hold)."""
    import json
    out = json.loads(json.dumps(profile))
    notes: List[str] = []
    active = int(out["active_servo_mask"])
    for slot in out["joints"]:
        index = int(slot["index"])
        if not active >> index & 1:
            continue
        tick = present[index]
        old = (slot["min_tick"], slot["max_tick"])
        if index in limits:
            limit = limits[index]
            low = min(limit["low"], tick) - ENVELOPE_MARGIN_TICKS
            high = max(limit["high"], tick) + ENVELOPE_MARGIN_TICKS
            home = limit["neutral"]
            ticks = max(1, math.ceil(speed_tps * 0.002))
            fraction = (ticks + 0.5) / (high - low)
            if fraction > slot["max_position_fraction_per_control_cycle"]:
                slot["max_position_fraction_per_control_cycle"] = min(1.0, fraction)
        elif old[0] <= tick <= old[1]:
            continue
        else:
            low, high = tick - fs.OTHER_WINDOW_TICKS, tick + fs.OTHER_WINDOW_TICKS
            home = tick
        slot["min_tick"], slot["max_tick"], slot["home_tick"] = int(low), int(high), int(home)
        if (low, high) != old:
            notes.append(f"servo {index}: {old[0]}..{old[1]} -> {low}..{high} (at {tick})")
    return out, notes


def tour_poses(limits: Dict[int, dict]) -> List[Tuple[str, Dict[int, int]]]:
    """(label, servo -> target tick) in order, for the servos in limits only;
    servos not named go to (or stay at) neutral."""
    n = {i: limit["neutral"] for i, limit in limits.items()}
    end = {i: limit["ends"] for i, limit in limits.items()}
    poses: List[Tuple[str, Dict[int, int]]] = [("neutral", dict(n))]
    for index in EYE_SERVOS:
        if index in limits:
            poses += [(f"{index}_end1", {**n, index: end[index][0]}),
                      (f"{index}_end2", {**n, index: end[index][1]}),
                      ("neutral", dict(n))]
    for index in LID_SERVOS:
        if index in limits:
            poses += [(f"{index}_closed", {**n, index: end[index][0]}), ("neutral", dict(n))]
    if 0 in limits and 2 in limits:
        poses += [("eyes_end1", {**n, 0: end[0][0], 2: end[2][0]}),
                  ("eyes_end2", {**n, 0: end[0][1], 2: end[2][1]})]
        if 1 in limits:
            poses += [("eyes_end1_pitch_end1", {**n, 0: end[0][0], 2: end[2][0], 1: end[1][0]}),
                      ("eyes_end2_pitch_end2", {**n, 0: end[0][1], 2: end[2][1], 1: end[1][1]})]
        poses += [("neutral", dict(n))]
    if 5 in limits and 6 in limits:
        for blink in (1, 2):
            poses += [(f"blink{blink}_closed", {**n, 5: end[5][0], 6: end[6][0]}),
                      ("neutral", dict(n))]
    return poses


class EyeTour:
    def __init__(self, link: fs.StreamLink, limits: Dict[int, dict], run: Path,
                 speed_tps: float, abort_ma: int, stop_file: Path, recorder=None,
                 say=print) -> None:
        self.link, self.limits, self.run = link, limits, Path(run)
        self.speed_tps, self.abort_ma, self.stop_file = speed_tps, abort_ma, stop_file
        self.recorder, self.say = recorder, say
        self.requested_stop: Optional[str] = None
        self.hold: Dict[int, int] = {}
        self.command: Dict[int, float] = {}
        self.stills = 0
        self.log: Dict = {"format": "head-eye-tour-1",
                          "started_utc": datetime.now(timezone.utc).isoformat(),
                          "speed_tps": speed_tps, "abort_ma": abort_ma,
                          "limits": {str(i): v for i, v in limits.items()},
                          "poses": [], "stopped": None}

    def make_envelope(self) -> None:
        state = self.link.wait_torque_off()
        present = {i: state.servos[i].present_tick for i in self.link.active}
        check_present(self.limits, present)
        board = headctl.read_profile(self.link.device)
        profile, notes = tour_envelope(board, self.limits, present, self.speed_tps)
        tuning_core.write_json_atomically(self.run / "profile_before.json", board)
        tuning_core.write_json_atomically(self.run / "session_profile.json", profile)
        for note in notes:
            self.say("  envelope " + note)
        generation = self.link.apply_envelope(profile, self.say)
        self.say("Envelope already on the board." if generation is None else
                 f"Envelope uploaded (generation {generation}).")
        self.log["envelope"] = {"notes": notes, "generation": generation}

    def start(self) -> None:
        self.link.connect()
        missing = [i for i in self.limits if i not in self.link.active]
        if missing:
            raise SetupError(f"servos {missing} are not active on the board")
        self.make_envelope()
        self.link.enable()
        state = self.link.state
        self.hold = {i: state.servos[i].goal_tick for i in self.link.active}
        self.command = {i: float(self.hold[i]) for i in self.limits}

    def check(self) -> Optional[str]:
        link = self.link
        if self.requested_stop is not None:
            return self.requested_stop
        if self.stop_file.exists():
            return f"stop file {self.stop_file}"
        if link.control_lost_reason is not None:
            return link.control_lost_reason
        state = link.state
        if state is None or state.state != fs.ENABLED_STATE:
            return f"the board left ENABLED ({state.state_name if state else 'no state'})"
        if not link.state_is_fresh():
            return "no fresh STATE from the board"
        for index in link.active:
            current = abs(state.servos[index].current_ma)
            if current >= self.abort_ma:
                return f"servo {index} at {current} mA (abort {self.abort_ma} mA)"
        hot = [f"{s.index} at {s.temperature_c} C" for s in state.servos
               if s.online and s.temperature_c >= fs.TEMP_STOP_C]
        if hot:                                  # operator rule 2026-11-10
            return f"temperature stop {fs.TEMP_STOP_C} C: servo " + ", ".join(hot)
        return None

    def send(self) -> None:
        targets = dict(self.hold)
        for index, tick in self.command.items():
            low, high = self.link.ranges[index]
            targets[index] = min(max(int(round(tick)), low), high)
        self.link.send_targets(targets)

    def move(self, label: str, pose: Dict[int, int]) -> Optional[str]:
        """Drive to the pose at the speed limit, hold HOLD_S, take a still.
        Returns a stop reason, or None."""
        period = 1.0 / fs.STREAM_HZ
        step = self.speed_tps * period
        start = time.monotonic()
        arrived_at = None
        while True:
            self.link.poll(0.002)
            reason = self.check()
            if reason is not None:
                return reason
            for index, target in pose.items():
                now = self.command[index]
                delta = max(-step, min(step, target - now))
                self.command[index] = now + delta
            self.send()
            state = self.link.state
            done = all(self.command[i] == pose[i] and
                       abs(state.servos[i].present_tick - pose[i]) <= ARRIVE_TICKS for i in pose)
            elapsed = time.monotonic() - start
            if done and arrived_at is None:
                arrived_at = elapsed
            if arrived_at is not None and elapsed - arrived_at >= HOLD_S:
                break
            if elapsed > MOVE_TIMEOUT_S:
                break
            time.sleep(period)
        state = self.link.state
        entry = {"label": label, "target": {str(i): t for i, t in pose.items()},
                 "present": {str(i): state.servos[i].present_tick for i in pose},
                 "current_ma": {str(i): state.servos[i].current_ma for i in pose},
                 "arrived_s": None if arrived_at is None else round(arrived_at, 2)}
        if arrived_at is None:
            short = {i: pose[i] - state.servos[i].present_tick for i in pose
                     if abs(pose[i] - state.servos[i].present_tick) > ARRIVE_TICKS}
            entry["not_arrived_ticks"] = {str(i): v for i, v in short.items()}
            self.say(f"  {label}: not reached in {MOVE_TIMEOUT_S:g} s, short by {short}")
        if self.recorder is not None:
            self.stills += 1
            path = self.run / records_layout.STILLS / f"{self.stills:02d}_{label}.jpg"
            self.recorder.still(path)
            entry["still"] = path.name
        self.log["poses"].append(entry)
        loads = " ".join(f"{i}:{state.servos[i].current_ma:+d}mA" for i in sorted(pose))
        self.say(f"  {label}: {loads}")
        return None

    def run_tour(self) -> str:
        for label, pose in tour_poses(self.limits):
            reason = self.move(label, pose)
            if reason is not None:
                return reason
        return "finished"

    def finish(self, reason: str) -> None:
        if self.log["stopped"] is None:
            self.log["stopped"] = reason
            self.link.disable()
            self.say(f"Stopped: {reason}. Torque off.")


def preview_marks(device) -> Dict[int, Dict[str, dict]]:
    """Marks around where the simulated servos sit, for --preview."""
    marks: Dict[int, Dict[str, dict]] = {}
    for index in TOUR_SERVOS:
        tick = device.present(index)
        entry = {"neutral": {"present_tick": tick}, "end_1": {"present_tick": tick + 400}}
        if index in EYE_SERVOS:
            entry["end_2"] = {"present_tick": tick - 400}
        marks[index] = entry
    return marks


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=headctl.AUTO_PORT)
    parser.add_argument("--records", type=Path,
                        default=Path(__file__).resolve().parents[4] / "tuning_runs")
    parser.add_argument("--preview", action="store_true", help="simulated board")
    parser.add_argument("--speed", type=float, required=True, help="ticks/s")
    parser.add_argument("--abort-ma", type=int, default=DEFAULT_ABORT_MA)
    parser.add_argument("--camera", choices=("none", "realsense"), default="none")
    parser.add_argument("--servo", type=int, action="append", choices=sorted(TOUR_SERVOS),
                        help="only these servos (repeat; default all); the others hold")
    a = parser.parse_args(argv)
    say = lambda text: print(text, flush=True)
    if not 0 < a.speed <= SPEED_MAX_TPS:
        print(f"--speed must be 1 to {SPEED_MAX_TPS} ticks/s"); return EXIT_NOT_STARTED
    if not 0 < a.abort_ma <= 300:
        print("--abort-ma must be 1 to 300 (max 300 mA rule)"); return EXIT_NOT_STARTED
    records = a.records / "preview" if a.preview else a.records
    stop_file = records / "STOP"
    if stop_file.exists():
        print(f"Remove the stop file first: {stop_file}"); return EXIT_NOT_STARTED
    device = fs.face_preview_controller() if a.preview else headctl.Device(a.port)
    try:
        marks = (preview_marks(device) if a.preview else
                 tuning_core.load_earlier_marks(records, since=tuning_core.HEAD_MARKS_SINCE)[0])
        limits = tour_limits(marks, a.servo)
    except SetupError as error:
        print(error)
        device.close()
        return EXIT_NOT_STARTED
    for index, limit in limits.items():
        say(f"  {index} ({TOUR_SERVOS[index]}): neutral {limit['neutral']}, ends {limit['ends']}")
    run = records_layout.run_dir(records, "eyes")
    say(f"Run folder: {run}")
    link = fs.StreamLink(device)
    telemetry = tuning_core.TelemetryLog(run / records_layout.TELEMETRY, sorted(limits))
    link.sample_listener = lambda state, pwm: telemetry.observe(state, pwm)
    recorder = None
    if a.camera == "realsense":
        from face_camera import RealSenseCamera, Recorder  # optional
        recorder = Recorder(RealSenseCamera(), run / records_layout.VIDEO)
        time.sleep(1.0)
    tour = EyeTour(link, limits, run, a.speed, a.abort_ma, stop_file, recorder, say)
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda signum, _f: setattr(tour, "requested_stop",
                                                         signal.Signals(signum).name))
    code = EXIT_NOT_STARTED
    try:
        try:
            tour.start()
        except (SetupError, RuntimeError) as error:
            say(f"Not started: {error}")
            tour.log["stopped"] = f"not started: {error}"
            return EXIT_NOT_STARTED
        say(f"Enabled; eye tour at {a.speed:g} ticks/s.")
        reason = tour.run_tour()
        tour.finish(reason)
        code = EXIT_FINISHED if reason == "finished" else EXIT_STOPPED
        return code
    except BaseException:
        tour.finish("error")
        raise
    finally:
        try:
            link.disable()
        except Exception:
            pass
        if recorder is not None:
            time.sleep(1.0)
            recorder.still(run / records_layout.STILLS / "99_torque_off_1s.jpg")
            time.sleep(1.0)
            recorder.close()
            tour.log["video"] = str(recorder.video_path)
            say(f"Video: {recorder.video_path} ({recorder.frames} frames)")
        telemetry.close()
        tuning_core.write_json_atomically(run / records_layout.RECORD, tour.log)
        say(f"Record: {run / records_layout.RECORD}")
        try:
            device.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
