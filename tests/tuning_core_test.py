"""ROS-free regression tests for the tuning panel logic and profile proposal."""

from __future__ import annotations

import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace

sys.modules.setdefault("serial", SimpleNamespace(Serial=object))
FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))
sys.path.insert(0, str(FIRMWARE / "scripts"))

import headctl  # noqa: E402
import tuning_core  # noqa: E402
from generate_profile import build_profile  # noqa: E402


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


def pump(link: tuning_core.TuningLink, count: int = 20) -> None:
    for _ in range(count):
        link.poll(0.0)


def started_link(load_ma_per_tick: float = 0.5):
    clock = FakeClock()
    board = tuning_core.SimulatedController({10: 2, 15: 1}, load_ma_per_tick, clock=clock)
    link = tuning_core.TuningLink(board, clock=clock)
    link.connect()
    link.start(3)
    clock.now += 0.1
    pump(link)
    return clock, board, link


def test_connect_requires_tuning_capability() -> None:
    board = tuning_core.SimulatedController()
    board.hello = lambda: {"capabilities": headctl.CAP_TRANSACTION_IDS, "session_id": 1}
    link = tuning_core.TuningLink(board)
    try:
        link.connect()
    except RuntimeError as error:
        assert "no tuning mode" in str(error)
    else:
        raise AssertionError("connect accepted firmware without tuning")


def test_start_step_and_load_limit() -> None:
    clock, board, link = started_link()
    assert link.state is not None and link.state.state == tuning_core.TUNING_STATE
    assert board.limit_ma == 3

    record = tuning_core.TuningRecord(Path("unused.json"), 3, {10: 2, 15: 1})
    record.observe(link.state)
    assert record.neutral_ticks == {10: board.neutral[10], 15: board.neutral[15]}

    # One request at a time: a second click while one is in flight is dropped.
    link.request_step(10, 1)
    assert link.step_in_flight
    link.request_step(10, 1)
    pump(link)
    assert link.step_results == [(10, 1, 0)]
    link.step_results.clear()

    # Load grows 0.5 mA per tick; at 6 ticks it reaches the 3 mA limit.
    for _ in range(5):
        link.request_step(10, 1)
        pump(link)
    assert all(result == 0 for _, _, result in link.step_results)
    link.step_results.clear()
    link.request_step(10, 1)
    pump(link)
    assert link.step_results == [(10, 1, -16)]
    assert "load limit" in tuning_core.step_result_text(-16)
    link.step_results.clear()
    link.request_step(10, -1)          # back toward neutral is always allowed
    pump(link)
    assert link.step_results == [(10, -1, 0)]
    assert board.goal[10] == board.neutral[10] + 5
    assert link.has_control

    # Larger steps move the whole amount in one click; 0 and >50 are refused.
    link.step_results.clear()
    link.request_step(15, 10)
    pump(link)
    assert link.step_results == [(15, 10, 0)] and board.goal[15] == board.neutral[15] + 10
    for bad in (0, 51, -51):
        try:
            link.request_step(15, bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"step {bad} was accepted")


def test_lease_is_renewed_and_loss_is_reported() -> None:
    clock, board, link = started_link()
    renewals = []
    original = board._receive

    def counting(kind, payload, transaction_id):
        if kind == headctl.RENEW_LEASE:
            renewals.append(transaction_id)
        original(kind, payload, transaction_id)

    board._receive = counting
    for _ in range(5):
        clock.now += tuning_core.LEASE_RENEW_PERIOD_S
        pump(link)
    assert len(renewals) == 5 and link.has_control

    # The board stops answering: control is reported lost after the timeout.
    board._receive = lambda kind, payload, transaction_id: None
    clock.now += tuning_core.LEASE_RENEW_PERIOD_S
    pump(link)
    clock.now += tuning_core.ANSWER_TIMEOUT_S + 0.01
    pump(link)
    assert not link.has_control
    assert "Control lost" in link.control_lost_reason


def test_unanswered_step_loses_control() -> None:
    clock, board, link = started_link()
    board._receive = lambda kind, payload, transaction_id: None
    link.request_step(15, 1)
    clock.now += tuning_core.ANSWER_TIMEOUT_S + 0.01
    pump(link)
    assert not link.step_in_flight and not link.has_control


def test_disable_returns_to_torque_off() -> None:
    clock, board, link = started_link()
    link.disable()
    assert board.state == tuning_core.HOMING_REQUIRED_STATE
    assert not link.has_control
    link.request_step(10, 1)            # ignored without control
    assert not link.step_in_flight


def make_record(path: Path, neutral: int, end_1: int, end_2: int, peak: int) -> None:
    record = tuning_core.TuningRecord(path, 40, {10: 2})
    record.neutral_ticks[10] = neutral
    record.peak_current_ma[10] = peak
    for name, tick in (("end_1", end_1), ("end_2", end_2)):
        record.add_mark(tuning_core.ServoSample(10, tick, tick, peak, True, 4), name)
    record.save()


def run_propose(*arguments: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(FIRMWARE / "scripts" / "propose_profile.py"),
                           *arguments], capture_output=True, text=True)


def test_propose_profile_from_record() -> None:
    with tempfile.TemporaryDirectory() as directory:
        folder = Path(directory)
        base = folder / "base.json"
        base.write_text(json.dumps(build_profile(1 << 10)))
        record = folder / "run.json"
        make_record(record, neutral=2000, end_1=2600, end_2=1500, peak=150)
        loaded = json.loads(record.read_text())
        assert loaded["format"] == "head-tuning-record-1"
        assert loaded["servos"]["10"]["marks"]["end_1"]["present_tick"] == 2600

        output = folder / "proposed.json"
        result = run_propose("--base", str(base), "--record", str(record),
                             "--end-margin-ticks", "50", "--current-margin-ma", "100",
                             "--output", str(output))
        assert result.returncode == 0, result.stderr
        slot = json.loads(output.read_text())["joints"][10]
        assert (slot["home_tick"], slot["min_tick"], slot["max_tick"]) == (2000, 1550, 2550)
        assert slot["operating_current_ma"] == 250
        # Homing currents never exceed the new operating current.
        assert slot["homing_current_limit_ma"] <= 250 and slot["homing_current_ma"] <= 250

        # Margins are a decision, so they have no default.
        missing = run_propose("--base", str(base), "--record", str(record),
                              "--output", str(output))
        assert missing.returncode != 0

        # A margin that swallows the neutral is refused, not silently clamped.
        too_wide = run_propose("--base", str(base), "--record", str(record),
                               "--end-margin-ticks", "600", "--current-margin-ma", "100",
                               "--output", str(folder / "bad.json"))
        assert too_wide.returncode == 1 and "not inside the ends" in too_wide.stderr
        assert not (folder / "bad.json").exists()


def test_propose_profile_with_one_end() -> None:
    with tempfile.TemporaryDirectory() as directory:
        folder = Path(directory)
        base = folder / "base.json"
        base.write_text(json.dumps(build_profile(1 << 10)))
        def proposal(end: int, margin: int = 50, click_start: bool = True):
            record_path = folder / f"one_{end}_{margin}_{click_start}.json"
            record = tuning_core.TuningRecord(record_path, 40, {10: 2})
            record.neutral_ticks[10] = 1900
            record.peak_current_ma[10] = 150
            if click_start:
                record.add_mark(tuning_core.ServoSample(10, 2000, 2000, 0, True, 4), "neutral")
            record.add_mark(tuning_core.ServoSample(10, end, end, 150, True, 4), "end_1")
            record.save()
            output = folder / "proposed.json"
            result = run_propose("--base", str(base), "--record", str(record_path),
                                 "--end-margin-ticks", str(margin), "--current-margin-ma", "100",
                                 "--output", str(output))
            slot = json.loads(output.read_text())["joints"][10] if result.returncode == 0 else None
            output.unlink(missing_ok=True)
            return result, slot
        # The range stops exactly at home; only the far end gets the margin.
        result, slot = proposal(2600)
        assert result.returncode == 0, result.stderr
        assert (slot["home_tick"], slot["min_tick"], slot["max_tick"]) == (2000, 2000, 2550)
        result, slot = proposal(1500)
        assert (slot["home_tick"], slot["min_tick"], slot["max_tick"]) == (2000, 1550, 2000)
        # A margin that leaves nothing is refused.
        result, slot = proposal(2040)
        assert result.returncode == 1 and "leaves no range" in result.stderr
        # Without a clicked start, the session start pose is used and reported.
        result, slot = proposal(2600, click_start=False)
        assert slot["home_tick"] == 1900 and "start never clicked" in result.stderr


def test_notes_file_round_trip() -> None:
    with tempfile.TemporaryDirectory() as folder:
        directory = Path(folder)
        assert tuning_core.load_notes(directory) == {}
        tuning_core.save_note(directory, 11, "frays near pulley")
        tuning_core.save_note(directory, 3, "end is soft")
        assert tuning_core.load_notes(directory) == {3: "end is soft", 11: "frays near pulley"}
        tuning_core.save_note(directory, 3, "")
        assert tuning_core.load_notes(directory) == {11: "frays near pulley"}
        record = tuning_core.TuningRecord(directory / "r.json", 40, {11: 1})
        record.notes[11] = "frays near pulley"
        assert record.to_json()["servos"]["11"]["note"] == "frays near pulley"
        assert tuning_core.is_ready_for_profile({"neutral": {}, "end_1": {}})
        assert tuning_core.is_ready_for_profile({"neutral": {}, "end_2": {}})
        assert not tuning_core.is_ready_for_profile({"end_1": {}, "end_2": {}})


def test_earlier_marks_and_home_target() -> None:
    with tempfile.TemporaryDirectory() as folder:
        directory = Path(folder)                        # the records folder
        panel = directory / "panel"                     # panel records live here
        panel.mkdir()
        def write(name: str, servos: dict) -> None:
            (panel / name).write_text(json.dumps({
                "format": "head-tuning-record-1", "servos": servos}))
        write("20261009T000000Z_tuning.json", {
            "3": {"neutral_tick": 900, "marks": {"neutral": {"present_tick": 1000},
                                                "end_1": {"present_tick": 1400}}},
            "4": {"neutral_tick": 50, "marks": {"end_1": {"present_tick": 70}}}})
        write("20261012T000000Z_tuning.json", {
            "3": {"neutral_tick": 1300, "marks": {"neutral": {"present_tick": 1100}}}})
        (panel / "20261013T000000Z_tuning.json").write_text("{broken")
        (directory / "20261014T000000Z_tuning.json").write_text("{broken")   # not in panel/: unread
        marks, skipped = tuning_core.load_earlier_marks(directory)
        # Per servo and mark, the latest file wins; other marks are kept.
        assert marks[3]["neutral"]["present_tick"] == 1100
        assert marks[3]["end_1"]["present_tick"] == 1400
        assert skipped == ["20261013T000000Z_tuning.json"]

        record = tuning_core.TuningRecord(directory / "now.json", 100, {3: 0, 4: 0, 5: 0})
        record.neutral_ticks = {3: 1250, 4: 60, 5: 7}
        # Earlier start mark beats this session's start pose...
        assert tuning_core.home_tick(3, record, marks) == 1100
        # ...a start mark clicked in this session beats both...
        record.marks[3] = {"neutral": {"present_tick": 1180}}
        assert tuning_core.home_tick(3, record, marks) == 1180
        # ...and with no start mark anywhere, this session's start pose is used.
        assert tuning_core.home_tick(4, record, marks) == 60
        assert tuning_core.home_tick(6, record, marks) is None


def test_home_mover_walks_back_with_bounded_steps() -> None:
    clock, board, link = started_link(load_ma_per_tick=0.0)
    for _ in range(3):
        link.request_step(10, 50)
        clock.now += 0.1
        pump(link)
    link.request_step(15, -40)
    clock.now += 0.1
    pump(link)
    link.step_results.clear()
    targets = {10: board.neutral[10], 15: board.neutral[15]}
    mover = tuning_core.HomeMover(targets, 200)
    sent = []
    for _ in range(200):
        clock.now += 0.02
        link.poll(0.0)
        while link.step_results:
            index, _, result = link.step_results.pop(0)
            assert mover.on_result(index, result, clock.now) is None
        if mover.is_done(link.state):
            break
        if not link.step_in_flight:
            step = mover.next_step(link.state)
            if step is not None:
                sent.append(step)
                link.request_step(*step)
    assert mover.is_done(link.state), sent
    assert board.goal[10] == board.neutral[10] and board.goal[15] == board.neutral[15]
    assert all(abs(step) <= headctl.TUNING_MAX_STEP_TICKS for _, step in sent)
    # One step each in turn, not one servo all the way first.
    assert [index for index, _ in sent[:2]] == [10, 15], sent


def test_home_mover_waits_and_gives_up() -> None:
    def state(goal: int, present: int, received_s: float = 1.0):
        servos = [tuning_core.ServoSample(i, 0, 0, 0, True, 4) for i in range(20)]
        servos[3] = tuning_core.ServoSample(3, goal, present, 0, True, 4)
        return tuning_core.ControllerState(tuning_core.TUNING_STATE, 0, 2, servos, received_s)
    mover = tuning_core.HomeMover({3: 1000}, 200)
    # Steps follow where the servo is: at most 50, never past home.
    assert mover.next_step(state(1100, 1090)) == (3, -50)
    # After an answer, a sample from before it is not used.
    assert mover.on_result(3, 0, 2.0) is None
    assert mover.next_step(state(1030, 1030, received_s=1.5)) is None
    assert mover.next_step(state(1030, 1030, received_s=2.1)) == (3, -30)
    # Within HOME_ARRIVE_TICKS of home the servo has arrived, whatever its goal.
    arrived = tuning_core.HomeMover({3: 1000}, 200)
    assert arrived.next_step(state(940, 996, received_s=3.0)) is None
    assert arrived.is_done(state(940, 996, received_s=3.0))
    # A servo blocked with its goal far beyond it, home behind it: the goal
    # comes back toward home at once (servo 7 case).
    blocked = tuning_core.HomeMover({3: 4071}, 200)
    assert blocked.next_step(state(3751, 3915, received_s=5.0)) == (3, 50)
    # Home beyond the goal: it pushes harder, but only up to the lead cap
    # (goal 200 ticks past the servo).
    assert tuning_core.HomeMover({3: 3600}, 200).next_step(
        state(3751, 3915, received_s=5.0)) == (3, -36)
    # Refusals that may clear (busy, stale) are retried a bounded number of times.
    for _ in range(tuning_core.HOME_MAX_REFUSALS - 1):
        assert mover.on_result(3, tuning_core.EBUSY_RESULT, 3.0) is None
    assert "stopped before home" in mover.on_result(3, tuning_core.EBUSY_RESULT, 3.0)
    assert mover.remaining == [] and mover.is_done(state(1030, 1030))
    # A refusal that cannot clear gives up at once.
    other = tuning_core.HomeMover({3: 1000})
    assert "One full turn" in other.on_result(3, -34, 1.0)


def test_backoff_releases_a_blocked_servo() -> None:
    def state(goal: int, present: int, current: int, received_s: float):
        servos = [tuning_core.ServoSample(i, 0, 0, 0, True, 4) for i in range(20)]
        servos[7] = tuning_core.ServoSample(7, goal, present, current, True, 4)
        return tuning_core.ControllerState(tuning_core.TUNING_STATE, 0, 2, servos, received_s)
    backoff = tuning_core.BackOff()
    # The short peak after an ordinary step does not trigger it...
    assert backoff.observe(state(3932, 3990, -590, 1.0), 600, [7]) == []
    assert backoff.observe(state(3932, 3935, -20, 1.1), 600, [7]) == []
    # ...but staying at the limit with the goal far ahead does.
    assert backoff.observe(state(3682, 3887, -585, 2.0), 600, [7]) == []
    assert backoff.observe(state(3682, 3887, -590, 2.2), 600, [7]) == []
    assert backoff.observe(state(3682, 3887, -598, 2.31), 600, [7]) == [7]
    # Steps go toward the servo, at most 50 ticks, stopping inside the window.
    assert backoff.next_step(state(3682, 3887, -580, 2.31)) == (7, 50)
    backoff.on_result(7, 2.35)
    assert backoff.next_step(state(3732, 3887, -540, 2.34)) is None   # stale sample
    assert backoff.next_step(state(3732, 3887, -540, 2.4)) == (7, 50)
    assert backoff.next_step(state(3832, 3887, -190, 2.5)) == (7, 36)
    assert backoff.next_step(state(3868, 3887, -60, 2.6)) is None
    assert backoff.active == set()
    # A servo holding below the limit is left holding, however long: with
    # Position I = 0, holding 454 mA means trailing by ~170 ticks (2026-10-12).
    holding = tuning_core.BackOff()
    for t in (10.0, 10.5, 11.0, 15.0, 60.0):
        assert holding.observe(state(3702, 3872, -454, t), 600, [7]) == []
    assert holding.observe(state(3702, 3872, -560, 61.0), 600, [7]) == []   # 93 %: hold
    assert holding.active == set()
    # A refused step that leaves the goal far ahead also starts it.
    other = tuning_core.BackOff()
    assert other.on_refused(7, state(3682, 3887, -400, 1.0)) and other.active == {7}
    assert not tuning_core.BackOff().on_refused(7, state(3880, 3887, -40, 1.0))


def test_backoff_against_simulated_wall() -> None:
    clock = FakeClock()
    board = tuning_core.SimulatedController({7: 2}, 0.0, clock=clock, walls={7: 2259 - 150})
    link = tuning_core.TuningLink(board, clock=clock)
    link.connect(); link.start(600)
    clock.now += 0.1; pump(link)
    backoff = tuning_core.BackOff()
    triggered = []
    for _ in range(300):
        clock.now += 0.05
        link.poll(0.0)
        while link.step_results:
            index, _, result = link.step_results.pop(0)
            if index in backoff.active:
                backoff.on_result(index, clock.now)
            elif result == tuning_core.EBUSY_RESULT and backoff.on_refused(index, link.state):
                backoff.on_result(index, clock.now)
                triggered.append(("refused", clock.now))
        triggered += [("held", clock.now) for _ in backoff.observe(link.state, 600, [7])]
        if link.step_in_flight:
            continue
        step = backoff.next_step(link.state)
        if step is not None:
            link.request_step(*step)
        elif not backoff.active and not triggered:
            link.request_step(7, -50)        # the operator keeps clicking -50
    assert triggered, "back-off never started"
    servo = link.state.servos[7]
    assert abs(servo.goal_tick - servo.present_tick) < tuning_core.BACKOFF_LEAD_TICKS
    assert abs(servo.current_ma) < 0.9 * 600, servo


def test_fault_names_the_servo_error() -> None:
    clock, board, link = started_link()
    board.trip(10, 0x20)
    clock.now += 0.2
    pump(link)
    reason = link.control_lost_reason or ""
    assert reason.startswith("Board fault: SERVO"), reason
    assert "servo 10: 0x20 (Overload)" in reason, reason
    assert "servo 15" not in reason, reason
    record = tuning_core.TuningRecord(Path("/tmp/unused.json"), 3, {10: 2, 15: 1})
    record.add_fault(link.state)
    fault = record.to_json()["faults"][0]
    assert fault["fault"] == "SERVO"
    assert {entry["servo"]: entry["hardware_error"] for entry in fault["servos"]}[10] == 0x20
    assert tuning_core.describe_hardware_error(0x34) == \
        "0x34 (Overload, Electrical Shock / insufficient power, Overheating)"


def drive_home(board, link, clock, mover, steps: int = 400):
    for _ in range(steps):
        clock.now += 0.05
        link.poll(0.0)
        while link.step_results:
            index, _, result = link.step_results.pop(0)
            mover.on_result(index, result, clock.now)
        if mover.is_done(link.state):
            return True
        if not link.step_in_flight:
            step = mover.next_step(link.state)
            if step is not None:
                link.request_step(*step)
    return False


def test_home_drives_a_loaded_servo_onto_home() -> None:
    clock = FakeClock()
    board = tuning_core.SimulatedController({7: 2}, 0.0, clock=clock)
    link = tuning_core.TuningLink(board, clock=clock)
    link.connect(); link.start(600)
    clock.now += 0.1; pump(link)
    home = board.goal[7] - 200
    # The jaw: a spring pulls it back toward 100 ticks above home.
    board.springs[7] = (home + 100, 3.5)
    mover = tuning_core.HomeMover({7: home}, tuning_core.tuning_max_lead_ticks(600))
    assert drive_home(board, link, clock, mover), (board.goal[7], board.present(7))
    servo = link.state.servos[7]
    assert abs(servo.present_tick - home) <= tuning_core.HOME_ARRIVE_TICKS, servo
    # The goal is past home by the lag the hold needs, and it is holding.
    assert servo.goal_tick < home - 50 and abs(servo.current_ma) > 250, servo


def test_home_reports_when_the_limit_is_not_enough() -> None:
    clock = FakeClock()
    board = tuning_core.SimulatedController({7: 2}, 0.0, clock=clock)
    link = tuning_core.TuningLink(board, clock=clock)
    link.connect(); link.start(150)
    clock.now += 0.1; pump(link)
    home = board.goal[7] - 200
    board.springs[7] = (home + 300, 3.5)       # needs ~1000 mA to hold at home
    mover = tuning_core.HomeMover({7: home}, tuning_core.tuning_max_lead_ticks(150))
    assert drive_home(board, link, clock, mover)
    assert 7 in mover.given_up and "cannot reach home within the load limit" in mover.notices[0]
    # Saturated at the limit, the mechanism pushes it back past the cap; that
    # case is BackOff's job in the panel, not Home's.


def sweep_settings(open_tick: int, pull=None, release=None, travel: int = 300,
                   push: int = 100) -> "tuning_core.SweepSettings":
    return tuning_core.SweepSettings(
        jaw=7, jaw_open_tick=open_tick, stages=4,
        pull={8: 1, 11: -1, 14: 1} if pull is None else pull,
        release=[11, 14] if release is None else release,
        preload_ma=60, band_ma=10, jaw_push_ma=push, abort_ma=400, max_travel_ticks=travel)


def test_home_refuses_moves_over_one_turn() -> None:
    # 2026-10-15: after a servo power cycle servo 8 read 1595 against a start
    # mark of 5864, and Home offered +4268 ticks.
    state = sweep_state(0.0, {})
    state.servos[8] = tuning_core.ServoSample(8, 1595, 1595, 0, True, 4)
    state.servos[10] = tuning_core.ServoSample(10, 2926, 2926, 0, True, 4)
    state.servos[11] = tuning_core.ServoSample(11, 1802, 1802, 0, True, 4)
    targets = {8: 5864, 10: -1616, 11: 1323, 12: 2000 + 4096, 13: 2000 - 4096}
    assert tuning_core.home_moves_over_one_turn(targets, state) == {8: 4269, 10: -4542}


def test_pull_directions_and_sweep_settings_check() -> None:
    assert tuning_core.parse_pull_directions("8+ 11-, 14+") == {8: 1, 11: -1, 14: 1}
    for bad in ("", "8", "8*", "x+", "8+ 8-"):
        try:
            tuning_core.parse_pull_directions(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"{bad!r} was accepted")
    assert tuning_core.parse_servo_list("11 14, 16") == [11, 14, 16]
    assert tuning_core.parse_servo_list("") == []
    for bad in ("11 x", "11 11"):
        try:
            tuning_core.parse_servo_list(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"{bad!r} was accepted")
    assert tuning_core.check_sweep_settings(sweep_settings(2400), [7, 8, 11, 14], 600) == []
    bad = tuning_core.SweepSettings(7, 2400, 0, {7: 1, 9: 1}, [12], 10, 10, 2, 15, 0)
    problems = " ".join(tuning_core.check_sweep_settings(bad, [7, 8], 600))
    for text in ("Servo 9 is not active", "is the jaw", "Release servo 12 is not in the tendon",
                 "at least 1 stage", "preload must be larger than the band",
                 "jaw push limit must be at least", "abort load must be above",
                 "travel limit"):
        assert text in problems, (text, problems)
    assert "no more than one turn" in " ".join(tuning_core.check_sweep_settings(
        sweep_settings(2400, travel=4097), [7, 8, 11, 14], 600))
    eyelid = tuning_core.SweepSettings(7, 2400, 0, {8: 1, 6: 1}, [6], 60, 10, 1, 60, 280)
    assert "Servo 6 is skipped" not in " ".join(tuning_core.check_sweep_settings(
        eyelid, [6, 7, 8], 600))
    saved_skipped = tuning_core.SKIPPED_SERVOS
    tuning_core.SKIPPED_SERVOS = (6,)
    try:
        assert "Servo 6 is skipped" in " ".join(tuning_core.check_sweep_settings(
            eyelid, [6, 7, 8], 600))
    finally:
        tuning_core.SKIPPED_SERVOS = saved_skipped
    # The abort must be above the jaw push limit too.
    assert "above the jaw push limit" in " ".join(tuning_core.check_sweep_settings(
        sweep_settings(2400, push=450), [7, 8, 11, 14], 600))
    # The abort must fire before BackOff, which acts at 95 % of the limit.
    assert "so the sweep stops before back-off" in " ".join(
        tuning_core.check_sweep_settings(sweep_settings(2400), [7, 8, 11, 14], 400))


def test_sweep_runner_spec_parsing() -> None:
    import argparse
    import sweep_runner
    assert sweep_runner.parse_sweep_spec("-80:4") == (-80, 4)
    assert sweep_runner.parse_sweep_spec("0:1") == (0, 1)
    for bad in ("80", "-80:0", "x:2", "-5000:2"):
        try:
            sweep_runner.parse_sweep_spec(bad)
        except argparse.ArgumentTypeError:
            pass
        else:
            raise AssertionError(f"{bad!r} was accepted")


def test_sweep_runner_held_stop_ends_the_sequence() -> None:
    """2026-10-18: servo 16 hit the travel limit inside a step, the runner
    said "finished" and started the next sweep. A held stop must end it."""
    import io, contextlib
    import sweep_runner
    with tempfile.TemporaryDirectory() as directory:
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            code = sweep_runner.main(["--preview", "--records", directory, "--limit", "600",
                                      "--sweep", "0:1", "--sweep", "-40:2",
                                      "--max-travel-ticks", "10"])
        text = output.getvalue()
        assert code == sweep_runner.EXIT_HELD, (code, text)
        assert "would move more than 10 ticks" in text and "Sweep finished" not in text, text
        assert len(list((Path(directory) / "preview" / "runs").glob("*/*_sweep/*_jawsweep.json"))) == 1, text


def test_per_servo_caps_and_jaw_guard() -> None:
    """2026-10-20: the jaw may peak at 650 mA, the lines stay at 300 mA, and the
    jaw may not stay high: over 400 mA for 2 s backs off, over 600 for 0.5 s
    turns torque off."""
    assert tuning_core.tuning_caps(650, {7: 650, 8: 300}) == {7: 650, 8: 300}
    assert tuning_core.tuning_caps(300, {7: 650, 8: 300}) == {7: 300, 8: 300}
    board = tuning_core.preview_controller(FakeClock())
    assert tuning_core.read_operating_currents(board, 1 << 7 | 1 << 8) == {7: 650, 8: 300}

    def state(t, jaw_ma, line_ma=0, lead=40):
        servos = [tuning_core.ServoSample(i, 2000 + lead, 2000, 0, True, 4) for i in range(20)]
        servos[7] = tuning_core.ServoSample(7, 2000 + lead, 2000, jaw_ma, True, 4)
        servos[8] = tuning_core.ServoSample(8, 2000 + lead, 2000, line_ma, True, 4)
        return tuning_core.ControllerState(tuning_core.TUNING_STATE, 0, 2, servos, t, 0)
    # Back-off uses each servo's own cap: 500 mA is fine for the jaw (650),
    # 290 mA is at the line's (300 x 95 %).
    backoff = tuning_core.BackOff()
    caps = {7: 650, 8: 300}
    for t in (0.0, 0.2, 0.4):
        started = backoff.observe(state(t, 500, 290), caps, [7, 8])
    assert backoff.active == {8}, backoff.active

    guard = tuning_core.JawGuard(7)
    assert guard.observe(state(0.0, -450)) is None
    assert guard.observe(state(1.9, -450)) is None
    assert guard.observe(state(2.0, -450)) == "back off"
    guard.reset()
    assert guard.observe(state(3.0, 380)) is None        # under 400: may stay
    assert guard.observe(state(10.0, 380)) is None
    assert guard.observe(state(10.1, 620)) is None        # a short peak is allowed
    assert guard.observe(state(10.3, 300)) is None
    assert guard.observe(state(11.0, 610)) is None
    assert guard.observe(state(11.5, 615)) == "torque off"

    # The sweep checks the lines' abort against their cap and the jaw's
    # against its own.
    settings = sweep_settings(2400, push=450)
    settings.abort_ma = 280
    settings.jaw_abort_ma = 600
    assert tuning_core.check_sweep_settings(settings, [7, 8, 11, 14], 650,
                                            {7: 650, 8: 300, 11: 300, 14: 300}) == []
    settings.abort_ma = 290
    assert "load limit (300 mA)" in " ".join(tuning_core.check_sweep_settings(
        settings, [7, 8, 11, 14], 650, {7: 650, 8: 300, 11: 300, 14: 300}))
    settings.abort_ma, settings.jaw_abort_ma = 280, 640
    assert "jaw abort load" in " ".join(tuning_core.check_sweep_settings(
        settings, [7, 8, 11, 14], 650, {7: 650, 8: 300, 11: 300, 14: 300}))


def test_payout_does_not_wait_forever_on_a_stuck_horn() -> None:
    """2026-10-20: servo 9's horn stuck 27 ticks behind while paying out, and
    the range finder waited for minutes. A horn a step behind for 1 s is
    left there."""
    import time as time_module
    import jaw_probe
    import sweep_runner
    board = tuning_core.preview_controller()
    board.walls[10] = board.goal[10] - 1     # servo 10 cannot turn downward
    link = tuning_core.TuningLink(board)
    link.connect()
    with tempfile.TemporaryDirectory() as directory:
        runner = sweep_runner.Runner(link, {7: 2, 8: 2, 10: 1, 11: 1, 12: 1, 14: 1}, Path(directory),
                                     600, dict(tuning_core.PREVIEW_SWEEP_BOXES),
                                     Path(directory) / "STOP", say=lambda _text: None)
        assert runner.start()
        probe = jaw_probe.Probe(runner, 7, {10: 1}, 280, None, Path(directory))
        start = time_module.monotonic()
        goal = link.state.servos[10].goal_tick - 300
        reason = probe.lines_to({10: goal}, True)
        took = time_module.monotonic() - start
        runner.disable(); runner.close()
    assert reason is None, reason
    assert took < 5.0, took                    # gave up on the stuck horn quickly
    assert board.goal[10] > goal               # and did not push the goal all the way


def test_a_servo_allowance_creeps_out_against_friction() -> None:
    """A servo given a pay-out allowance (PAYOUT_BLOCKED_LAG_BY_SERVO, here
    40 ticks for a horn that needs 28 ticks of trail) creeps out instead of
    being left blocked, and its goal never runs further ahead than that."""
    import jaw_probe
    import sweep_runner
    saved = dict(jaw_probe.PAYOUT_BLOCKED_LAG_BY_SERVO)
    jaw_probe.PAYOUT_BLOCKED_LAG_BY_SERVO[9] = 40
    board = tuning_core.SimulatedController({7: 2, 9: 2, 10: 1})
    friction = 28

    real_present = board.present
    def present(index):                                 # the horn stays put until the
        tick = real_present(index)                      # goal is friction ticks ahead
        if index != 9:
            return tick
        return board._horn9 if abs(board.goal[9] - board._horn9) <= friction else \
            board.goal[9] + friction * (1 if board._horn9 > board.goal[9] else -1)
    board._horn9 = board.goal[9]
    board.present = present
    link = tuning_core.TuningLink(board)
    link.connect()
    with tempfile.TemporaryDirectory() as directory:
        runner = sweep_runner.Runner(link, {7: 2, 9: 2, 10: 1}, Path(directory), 600,
                                     dict(tuning_core.PREVIEW_SWEEP_BOXES),
                                     Path(directory) / "STOP", say=lambda _text: None)
        assert runner.start()
        probe = jaw_probe.Probe(runner, 7, {9: 1}, 280, None, Path(directory))
        start = board.goal[9]
        most_ahead = [0]                                 # ticks the goal led the horn
        real_step = board._step

        def step(token_ok, index, direction):
            result = real_step(token_ok, index, direction)
            if index == 9:
                board._horn9 = present(9)
                most_ahead[0] = max(most_ahead[0], board._horn9 - board.goal[9])
            return result
        board._step = step
        reason = probe.lines_to({9: start - 300}, True)
        runner.disable(); runner.close()
    assert reason is None, reason
    assert board.goal[9] == start - 300                 # paid all the way out
    assert most_ahead[0] <= jaw_probe.PAYOUT_BLOCKED_LAG_BY_SERVO[9], most_ahead[0]
    jaw_probe.PAYOUT_BLOCKED_LAG_BY_SERVO.clear()
    jaw_probe.PAYOUT_BLOCKED_LAG_BY_SERVO.update(saved)


def test_sweep_boxes_remembered() -> None:
    with tempfile.TemporaryDirectory() as directory:
        assert tuning_core.load_sweep_boxes(Path(directory)) == tuning_core.SWEEP_BOX_DEFAULTS
        assert tuning_core.SWEEP_BOX_DEFAULTS["pull"] == "8- 11+ 14+ 9- 15- 19-"
        assert tuning_core.SWEEP_BOX_DEFAULTS["release"] == "8 9 11 14 15 19"
        boxes = dict(tuning_core.SWEEP_BOX_DEFAULTS, preload_ma="75", stages="3")
        tuning_core.save_sweep_boxes(Path(directory), boxes)
        assert tuning_core.load_sweep_boxes(Path(directory)) == boxes
        # A damaged file falls back to the defaults instead of failing.
        (Path(directory) / "config" / tuning_core.SWEEP_BOXES_FILE).write_text("{", encoding="utf-8")
        assert tuning_core.load_sweep_boxes(Path(directory)) == tuning_core.SWEEP_BOX_DEFAULTS
        # Not a tuning record: earlier-mark loading skips it.
        assert tuning_core.load_earlier_marks(Path(directory)) == ({}, [])


def test_sweep_jaw_targets_go_out_and_back() -> None:
    assert tuning_core.sweep_jaw_targets(100, 200, 4) == [100, 125, 150, 175, 200,
                                                          175, 150, 125, 100]
    assert tuning_core.sweep_jaw_targets(300, 200, 1) == [300, 200, 300]


def sweep_link():
    """The preview board: jaw 7, tendon 8 that feels the jaw, and 11/14 hidden
    behind friction, holding the jaw closed until they pay out."""
    clock = FakeClock()
    board = tuning_core.preview_controller(clock)
    link = tuning_core.TuningLink(board, clock=clock)
    link.connect(); link.start(600)
    clock.now += 0.1; pump(link)
    return clock, board, link


def run_sweep(link, clock, sweep, ticks: int = 40000) -> int:
    """Run until the sweep ends; returns the highest jaw load seen (mA)."""
    peak_jaw_ma = 0
    for _ in range(ticks):
        clock.now += 0.01
        pump(link, 5)
        while link.step_results:
            index, _, result = link.step_results.pop(0)
            sweep.on_result(index, result, clock.now)
        sweep.observe(link.state)
        peak_jaw_ma = max(peak_jaw_ma, abs(link.state.servos[7].current_ma))
        if not sweep.active:
            break
        if not link.step_in_flight:
            step = sweep.next_step(link.state)
            if step is not None:
                link.request_step(*step)
    return peak_jaw_ma


def test_jaw_sweep_releases_the_lifting_tendons_to_open_the_jaw() -> None:
    clock, board, link = sweep_link()
    settings = sweep_settings(board.goal[7] + 120)
    with tempfile.TemporaryDirectory() as directory:
        sweep = tuning_core.JawSweep(tuning_core.new_sweep_path(Path(directory)), settings,
                                     link.state, clock.now)
        peak_jaw_ma = run_sweep(link, clock, sweep)
        assert sweep.done and sweep.stopped is None, sweep.stopped
        assert [row["leg"] for row in sweep.rows] == ["start"] + ["open"] * 4 + ["close"] * 4
        for row in sweep.rows:
            assert abs(row["servos"]["7"]["present_tick"] - row["jaw_target_tick"]) <= \
                tuning_core.HOME_ARRIVE_TICKS, row
            for index, sign in settings.pull.items():
                assert abs(row["servos"][str(index)]["current_ma"] * sign - 60) <= 10, row
        start, opened, end = sweep.rows[0], sweep.rows[4], sweep.rows[-1]
        # 11 (pull -) and 14 (pull +) paid out to let the jaw open 120 ticks.
        assert opened["servos"]["11"]["present_tick"] - start["servos"]["11"]["present_tick"] > 40
        assert start["servos"]["14"]["present_tick"] - opened["servos"]["14"]["present_tick"] > 30
        # Closed again, every tendon is back where it began: pulling in from
        # below found the lines that had gone slack behind the friction.
        for index in ("8", "11", "14"):
            assert abs(end["servos"][index]["present_tick"] -
                       start["servos"][index]["present_tick"]) <= 5, (start, end)
        # The jaw never pushed much more than it carried at a stage.
        carried = max(row["servos"]["7"]["current_ma"] for row in sweep.rows)
        assert peak_jaw_ma <= carried + settings.jaw_push_ma + 20, (peak_jaw_ma, carried)
        # The slack lines were pulled in at the pretension; none is unverified.
        assert all(ticks >= tuning_core.PRETENSION_MOTION_MIN_TICKS
                   for ticks in start["taken_in_ticks"].values()), start["taken_in_ticks"]
        assert all(row["unverified"] == [] for row in sweep.rows)
        sweep.save()
        saved = json.loads(sweep.path.read_text(encoding="utf-8"))
        assert saved["format"] == tuning_core.SWEEP_FORMAT and saved["finished"]
        assert len(saved["rows"]) == 9 and saved["settings"]["release"] == [11, 14]
        # The sweep file is not a tuning record: earlier-mark loading skips it.
        assert tuning_core.load_earlier_marks(Path(directory)) == ({}, [])


def test_jaw_sweep_leaves_deferred_tendons_slack_on_the_way_open() -> None:
    """Option B, 2026-10-18: lines that hold the jaw are not pulled back to
    the preload while it opens; they are re-tensioned on the way back."""
    clock, board, link = sweep_link()
    settings = sweep_settings(board.goal[7] + 120)
    settings.defer_on_open = [8]
    sweep = tuning_core.JawSweep(Path("unused.json"), settings, link.state, clock.now)
    run_sweep(link, clock, sweep)
    assert sweep.done, sweep.stopped
    legs = [(row["leg"], row["slack"]) for row in sweep.rows]
    assert legs == [("start", [])] + [("open", [8])] * 4 + [("close", [])] * 4, legs
    for row in sweep.rows:
        load_8 = row["servos"]["8"]["current_ma"]
        if row["leg"] == "open":
            # Never pulled in while opening, never left over the band.
            assert row["taken_in_ticks"]["8"] <= 0 and load_8 <= 70, row
        else:
            assert abs(load_8 - 60) <= 10, row
    assert sweep.to_json()["settings"]["defer_on_open"] == [8]
    bad = sweep_settings(2400); bad.defer_on_open = [12]
    assert "Slack-while-opening servo 12 is not in the tendon list" in " ".join(
        tuning_core.check_sweep_settings(bad, [7, 8, 11, 12, 14], 600))


def test_jaw_sweep_flags_a_tendon_that_did_not_move_at_pretension() -> None:
    """What servo 9 did on 2026-10-15: its horn hardly turned, yet it read the
    preload. The row must say its load is unverified."""
    clock, board, link = sweep_link()
    board.walls[10] = board.goal[10]           # servo 10 cannot turn at all
    settings = sweep_settings(board.goal[7] + 120, pull={8: 1, 10: 1, 11: -1, 14: 1})
    sweep = tuning_core.JawSweep(Path("unused.json"), settings, link.state, clock.now)
    run_sweep(link, clock, sweep)
    start = sweep.rows[0]
    assert start["unverified"] == [10], start
    assert start["taken_in_ticks"]["10"] == 0
    assert all(row["unverified"] == [] for row in sweep.rows[1:])

    # A second sweep starts with the lines already at the preload: nothing to
    # take in, so nothing is flagged, not even the jammed servo 10.
    second = tuning_core.JawSweep(Path("unused.json"), settings, link.state, clock.now)
    assert second._started_slack == set(), second._started_slack


def test_jaw_sweep_stops_when_releasing_does_not_free_the_jaw() -> None:
    """2026-10-18: the release tendons paid out ~200 ticks each and the jaw
    did not move. Servo 10 here is not coupled to the jaw at all."""
    clock, board, link = sweep_link()
    settings = sweep_settings(board.goal[7] + 120, pull={8: 1, 10: 1, 11: -1, 14: 1},
                              release=[10])
    sweep = tuning_core.JawSweep(Path("unused.json"), settings, link.state, clock.now)
    run_sweep(link, clock, sweep)
    assert sweep.stopped and "did not move while every release tendon (10) paid out" \
        in sweep.stopped, sweep.stopped
    assert not sweep.torque_off and len(sweep.rows) == 1
    # It stopped soon after the limit, not at the travel limit or the timeout.
    paid_out = sweep.rows[0]["servos"]["10"]["goal_tick"] - board.goal[10]
    assert tuning_core.NO_PROGRESS_RELEASE_TICKS <= paid_out < 3 * tuning_core.NO_PROGRESS_RELEASE_TICKS, \
        paid_out


def test_jaw_sweep_does_not_force_a_blocked_jaw() -> None:
    """What happened by hand on 2026-10-15: nothing gives the jaw room. The
    sweep must not push it to the load limit; it stops and says why."""
    clock, board, link = sweep_link()
    settings = sweep_settings(board.goal[7] + 120, release=[])
    sweep = tuning_core.JawSweep(Path("unused.json"), settings, link.state, clock.now)
    peak_jaw_ma = run_sweep(link, clock, sweep)
    assert sweep.stopped and "held at its push limit" in sweep.stopped, sweep.stopped
    assert "no release tendon" in sweep.stopped and not sweep.torque_off
    assert len(sweep.rows) == 1
    carried = sweep.rows[0]["servos"]["7"]["current_ma"]
    assert peak_jaw_ma <= carried + settings.jaw_push_ma + 20, (peak_jaw_ma, carried)


def test_jaw_sweep_stops_when_a_pull_direction_is_wrong() -> None:
    clock, board, link = sweep_link()
    # Servo 8 really tightens toward higher ticks; given as "-", it unwinds.
    settings = sweep_settings(board.goal[7] + 120, pull={8: -1, 11: -1, 14: 1}, travel=100)
    start_8 = board.goal[8]
    sweep = tuning_core.JawSweep(Path("unused.json"), settings, link.state, clock.now)
    run_sweep(link, clock, sweep)
    assert sweep.stopped and "would move more than 100 ticks" in sweep.stopped, sweep.stopped
    assert not sweep.torque_off and not sweep.rows
    assert abs(board.goal[8] - start_8) <= 100


def sweep_state(received_s: float, currents: dict) -> tuning_core.ControllerState:
    servos = [tuning_core.ServoSample(i, 2000, 2000, currents.get(i, 0), True, 4)
              for i in range(20)]
    return tuning_core.ControllerState(tuning_core.TUNING_STATE, 0, 2, servos, received_s, 0)


def test_jaw_sweep_abort_load_and_reading_against_pull() -> None:
    settings = sweep_settings(2100, pull={8: 1, 11: -1}, release=[11])
    new = lambda: tuning_core.JawSweep(Path("unused.json"), settings,  # noqa: E731
                                       sweep_state(0.0, {}), 0.0)
    sweep = new()
    sweep.observe(sweep_state(0.1, {8: 60, 11: -405}))
    assert sweep.stopped and "Servo 11 reached 405 mA" in sweep.stopped and sweep.torque_off
    assert sweep.next_step(sweep_state(0.2, {})) is None

    sweep = new()                       # the jaw has the same abort load
    sweep.observe(sweep_state(0.1, {7: 410, 8: 60, 11: -60}))
    assert sweep.stopped and "Servo 7 reached 410 mA" in sweep.stopped and sweep.torque_off

    sweep = new()                       # either direction: 8 pushed out hard
    sweep.observe(sweep_state(0.1, {8: -420, 11: -60}))
    assert sweep.stopped and "Servo 8 reached 420 mA" in sweep.stopped and sweep.torque_off

    sweep = new()
    # A reading against the pull direction is the servo pushing its horn out
    # against friction (servo 14 at -11 mA, 2026-10-15): no line load, and
    # never a reason to stop.
    for sample in range(1, 3 * tuning_core.SWEEP_SETTLE_SAMPLES):
        sweep.observe(sweep_state(0.1 * sample, {8: -40, 11: 60}))
    assert sweep.active, sweep.stopped
    assert sweep.tension_ma(sweep_state(0.0, {8: -40}).servos[8]) == 0
    assert sweep.tension_ma(sweep_state(0.0, {11: -45}).servos[11]) == 45

    sweep = new()
    sweep.observe(sweep_state(tuning_core.SWEEP_STAGE_TIMEOUT_S + 0.1, {8: 0, 11: 0}))
    assert sweep.stopped and "did not finish" in sweep.stopped

    sweep = new()
    sweep.on_result(8, -34, 0.1)
    assert sweep.stopped and "One full turn" in sweep.stopped


def state_at(mcu_time_ms: int, current_ma: int = 0, hardware_error: int = 0,
             board_state: int = tuning_core.TUNING_STATE, fault: int = 0):
    servos = [tuning_core.ServoSample(i, 0, 0, 0, True, 4) for i in range(20)]
    servos[7] = tuning_core.ServoSample(7, 3700, 3850, current_ma, True, 4, hardware_error, 30, 10900)
    return tuning_core.ControllerState(board_state, fault, 2, servos, 0.0, mcu_time_ms)


def read_telemetry(path: Path) -> list:
    import csv
    with open(path, encoding="utf-8", newline="") as source:
        return list(csv.DictReader(source))


def test_present_pwm_is_paired_with_its_state_and_logged() -> None:
    clock, board, link = started_link()
    assert link.capabilities & headctl.CAP_PRESENT_PWM
    with tempfile.TemporaryDirectory() as directory:
        record = tuning_core.TuningRecord(Path(directory) / "20261013T000000Z_tuning.json",
                                          3, {10: 2, 15: 1})
        assert record.telemetry_path.name == "20261013T000000Z_tuning.telemetry.csv"
        log = tuning_core.TelemetryLog(record.telemetry_path, record.servo_branches)
        seen = []
        link.sample_listener = lambda state, pwm: (seen.append((state, pwm)), log.observe(state, pwm))
        for _ in range(4):
            link.request_step(10, 1)
            clock.now += 0.1
            pump(link)
        assert seen and all(pwm is not None for _, pwm in seen)
        for state, pwm in seen:
            assert pwm[10] == state.servos[10].current_ma // 2, (state.servos[10], pwm[10])
        board.trip(10, 0x20)
        clock.now += 0.1
        pump(link)
        clock.now += 0.1
        pump(link)                        # the next STATE releases nothing unpaired
        record.add_fault(link.state)
        log.close()
        rows = read_telemetry(record.telemetry_path)
        assert tuple(rows[0]) == tuning_core.TELEMETRY_COLUMNS
        assert {row["servo"] for row in rows} == {"10", "15"}
        loaded = [row for row in rows if row["servo"] == "10" and row["board_state"] == "TUNING"]
        assert loaded[-1]["pwm_percent"] == \
            f"{int(loaded[-1]['current_ma']) // 2 * headctl.PRESENT_PWM_PERCENT_PER_UNIT:.2f}"
        tripped = [row for row in rows if row["hardware_error"] == "32"]
        assert tripped and tripped[0]["servo"] == "10" and tripped[0]["fault"] == "SERVO"
        data = record.to_json()
        assert data["telemetry_file"] == record.telemetry_path.name
        assert data["faults"][0]["mcu_time_ms"] == link.state.mcu_time_ms
        assert any(row["mcu_time_ms"] == str(data["faults"][0]["mcu_time_ms"]) for row in rows)


def test_state_without_pwm_is_still_passed_on() -> None:
    # Old firmware: no PRESENT_PWM capability, every STATE goes out with None.
    clock = FakeClock()
    board = tuning_core.SimulatedController({7: 2}, 0.0, clock=clock)
    board.hello = lambda: {"capabilities": headctl.CAP_TRANSACTION_IDS | headctl.CAP_TUNING,
                           "session_id": board.SESSION_ID}
    link = tuning_core.TuningLink(board, clock=clock)
    link.connect()
    seen = []
    link.sample_listener = lambda state, pwm: seen.append(pwm)
    for _ in range(3):
        clock.now += 0.1
        pump(link)
    assert len(seen) == 3 and seen == [None, None, None]

    # New firmware, but one PRESENT_PWM is lost (replaced in the USB queue):
    # that STATE is passed on without PWM when the next STATE arrives.
    link = tuning_core.TuningLink(board, clock=clock)
    link.capabilities = headctl.CAP_PRESENT_PWM
    seen = []
    link.sample_listener = lambda state, pwm: seen.append((state.mcu_time_ms, pwm))
    pwm_payload = struct.pack("<I20h", 1000, *([5] * 20))
    link._handle(headctl.STATE, board._state_payload(1000))
    link._handle(headctl.PRESENT_PWM, pwm_payload)
    link._handle(headctl.STATE, board._state_payload(1010))
    link._handle(headctl.PRESENT_PWM, struct.pack("<I20h", 999, *([6] * 20)))   # stale
    link._handle(headctl.STATE, board._state_payload(1020))
    assert seen == [(1000, tuple([5] * 20)), (1010, None)], seen


def test_telemetry_log_spacing_and_change_rows() -> None:
    with tempfile.TemporaryDirectory() as directory:
        log = tuning_core.TelemetryLog(Path(directory) / "t.telemetry.csv", [7])
        pwm = tuple([100] * 20)
        logged = [log.observe(state_at(1000 + 10 * n, 400), pwm) for n in range(25)]
        # 100 Hz in, one row per 100 ms of controller time out.
        assert [n for n, done in enumerate(logged) if done] == [0, 10, 20]
        # A trip is logged at once, between the periodic rows, and so is the fault.
        assert log.observe(state_at(1255, 450, 0x20), pwm)
        assert log.observe(state_at(1265, 0, 0x20, tuning_core.FAULT_STATE,
                                    headctl.FAULT_NAMES.index("SERVO")), pwm)
        assert not log.observe(state_at(1275, 0, 0x20, tuning_core.FAULT_STATE,
                                        headctl.FAULT_NAMES.index("SERVO")), pwm)
        # MCU time wraps after ~49.7 days; the spacing still works across it.
        log2 = tuning_core.TelemetryLog(Path(directory) / "w.telemetry.csv", [7])
        assert log2.observe(state_at(0xFFFFFFF0), pwm)
        assert not log2.observe(state_at(0x00000010), pwm)
        assert log2.observe(state_at(0x00000060), pwm)
        log.close(); log2.close()
        rows = read_telemetry(Path(directory) / "t.telemetry.csv")
        assert [row["mcu_time_ms"] for row in rows] == ["1000", "1100", "1200", "1255", "1265"]
        assert rows[0]["pwm_percent"] == f"{100 * 0.113:.2f}" and rows[0]["current_ma"] == "400"
        assert rows[3]["hardware_error"] == "32" and rows[4]["fault"] == "SERVO"
        assert not log.observe(state_at(2000), pwm)      # closed: nothing more


def main() -> None:
    test_present_pwm_is_paired_with_its_state_and_logged()
    test_state_without_pwm_is_still_passed_on()
    test_telemetry_log_spacing_and_change_rows()
    test_home_drives_a_loaded_servo_onto_home()
    test_home_reports_when_the_limit_is_not_enough()
    test_fault_names_the_servo_error()
    test_backoff_releases_a_blocked_servo()
    test_backoff_against_simulated_wall()
    test_propose_profile_with_one_end()
    test_notes_file_round_trip()
    test_earlier_marks_and_home_target()
    test_home_mover_walks_back_with_bounded_steps()
    test_home_mover_waits_and_gives_up()
    test_connect_requires_tuning_capability()
    test_start_step_and_load_limit()
    test_lease_is_renewed_and_loss_is_reported()
    test_unanswered_step_loses_control()
    test_disable_returns_to_torque_off()
    test_propose_profile_from_record()
    test_home_refuses_moves_over_one_turn()
    test_pull_directions_and_sweep_settings_check()
    test_sweep_runner_spec_parsing()
    test_sweep_runner_held_stop_ends_the_sequence()
    test_per_servo_caps_and_jaw_guard()
    test_payout_does_not_wait_forever_on_a_stuck_horn()
    test_a_servo_allowance_creeps_out_against_friction()
    test_sweep_boxes_remembered()
    test_sweep_jaw_targets_go_out_and_back()
    test_jaw_sweep_releases_the_lifting_tendons_to_open_the_jaw()
    test_jaw_sweep_leaves_deferred_tendons_slack_on_the_way_open()
    test_jaw_sweep_flags_a_tendon_that_did_not_move_at_pretension()
    test_jaw_sweep_stops_when_releasing_does_not_free_the_jaw()
    test_jaw_sweep_does_not_force_a_blocked_jaw()
    test_jaw_sweep_stops_when_a_pull_direction_is_wrong()
    test_jaw_sweep_abort_load_and_reading_against_pull()
    print("tuning core: all tests passed")


if __name__ == "__main__":
    main()
