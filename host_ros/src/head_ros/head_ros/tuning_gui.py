"""Tuning panel: find each mechanism's range one tick at a time.

Talks to the Teensy over USB with headctl's serial code; ROS is not needed.
All firmware protections stay active. Each Start writes a new record file in
--records (default: firmware/tuning_runs/). Use scripts/propose_profile.py to
turn records into a proposed profile.

    scripts/head-tuning                 real board, found through /dev/serial/by-id
    scripts/head-tuning --preview       simulated board, no hardware
"""

from __future__ import annotations

import argparse
from pathlib import Path
import signal
import sys
from typing import Dict, Optional

from PyQt5.QtCore import QEvent, QObject, QSize, Qt, QTimer
from PyQt5.QtGui import QFont, QIntValidator, QKeySequence
from PyQt5.QtWidgets import (QAbstractItemView, QApplication, QGridLayout,
                             QGroupBox, QHBoxLayout, QLabel, QLineEdit,
                             QMainWindow, QMessageBox, QPushButton, QShortcut,
                             QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

try:
    from . import headctl, records_layout, tuning_core
except ImportError:  # run as a plain script from this directory
    import records_layout  # type: ignore[no-redef]
    import headctl  # type: ignore[no-redef]
    import tuning_core  # type: ignore[no-redef]

DEFAULT_RECORDS = Path(__file__).resolve().parents[4] / "tuning_runs"
POLL_PERIOD_MS = 20
# Load limit offered when the profile cannot be read (the profile value on
# 2026-10-12). The board refuses a limit above any active servo's
# operating_current_ma, so the lowest of those is offered when it is known.
DEFAULT_LIMIT_MA = 600
COLUMNS = ("Servo", "Connector", "Position", "From home", "Load (mA)",
           "Peak (mA)", "PWM (%)", "Temp (°C)", "Marked", "Notes")
NOTES_COLUMN = COLUMNS.index("Notes")
MARK_LABELS = {"neutral": "start", "end_1": "end", "end_2": "2nd end"}
DEGREES_PER_TICK = 360.0 / 4096


class EscapeStopsFilter(QObject):
    """Calls `stop` on every Esc key press, then lets the event carry on.

    on_stop() is safe to call twice, so Esc reaching the shortcut as well is
    harmless."""

    def __init__(self, stop) -> None:
        super().__init__()
        self.stop = stop

    def eventFilter(self, watched, event) -> bool:  # noqa: N802 - Qt override
        if event.type() == QEvent.KeyPress and event.key() == Qt.Key_Escape:
            self.stop()
        return False


class TuningWindow(QMainWindow):
    def __init__(self, open_device, port: str, records_directory: Path) -> None:
        super().__init__()
        self.open_device = open_device
        self.records_directory = records_directory
        self.link: Optional[tuning_core.TuningLink] = None
        self.record: Optional[tuning_core.TuningRecord] = None
        # Telemetry of the tuned servos for the whole session, next to the
        # record, so what led up to a fault can be read afterwards.
        self.telemetry_log: Optional[tuning_core.TelemetryLog] = None
        self.servo_branches: Dict[int, int] = {}
        # Marks from earlier sessions (read at Connect and at each Start), and
        # the walk back to home while one is running.
        self.earlier_marks: Dict[int, Dict[str, dict]] = {}
        self.filling_notes = False
        self.homer: Optional[tuning_core.HomeMover] = None
        self.sweep: Optional[tuning_core.JawSweep] = None
        self.backoff = tuning_core.BackOff()
        self.setWindowTitle("Head tuning")

        big = QFont()
        big.setPointSize(13)

        self.port_edit = QLineEdit(port)
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.on_connect)
        self.board_label = QLabel("Board: not connected")
        self.board_label.setFont(big)

        self.limit_edit = QLineEdit(str(DEFAULT_LIMIT_MA))
        self.limit_edit.setPlaceholderText("type a number")
        self.limit_edit.setValidator(QIntValidator(1, 910, self))
        self.limit_edit.textChanged.connect(self.refresh_controls)
        self.start_button = QPushButton("Start tuning")
        self.start_button.clicked.connect(self.on_start)
        self.stop_button = QPushButton("STOP  (Esc)")
        self.stop_button.setStyleSheet(
            "QPushButton { background: #c62828; color: white; font-weight: bold; padding: 10px; }")
        self.stop_button.clicked.connect(self.on_stop)
        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self.on_stop)
        # A text box that has focus (a Notes cell being edited) takes Esc for
        # itself, and the shortcut above never fires. Watch every key press in
        # the application so Esc stops the servos wherever the focus is.
        self.escape_filter = EscapeStopsFilter(self.on_stop)
        QApplication.instance().installEventFilter(self.escape_filter)

        self.table = QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        # Only the Notes cells are editable (see on_connect).
        self.table.setEditTriggers(QAbstractItemView.DoubleClicked |
                                   QAbstractItemView.EditKeyPressed |
                                   QAbstractItemView.SelectedClicked)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.itemChanged.connect(self.on_item_changed)
        self.table.verticalHeader().setVisible(False)
        self.table.itemSelectionChanged.connect(self.refresh_controls)

        self.selected_label = QLabel("Pick a servo in the table.")
        self.selected_label.setFont(big)
        self.step_buttons = []
        for step in (-50, -10, -1, 1, 10, 50):
            button = QPushButton(f"{step:+d}")
            button.setAutoRepeat(False)   # one click is one step, never more
            button.setMinimumHeight(48)
            button.setFont(big)
            button.clicked.connect(lambda _=False, s=step: self.on_step(s))
            self.step_buttons.append(button)
        self.mark_buttons = {}
        for name, text in (("neutral", "Mark start (home)"), ("end_1", "Mark end"),
                           ("end_2", "Mark 2nd end (only if it has one)")):
            button = QPushButton(text)
            button.clicked.connect(lambda _=False, n=name: self.on_mark(n))
            self.mark_buttons[name] = button

        self.home_selected_button = QPushButton("Home selected")
        self.home_selected_button.setToolTip(
            "Step the selected servo back to its latest start mark "
            "(or where this session started). Esc stops.")
        self.home_selected_button.clicked.connect(lambda: self.on_home(all_servos=False))
        self.home_all_button = QPushButton("Home all")
        self.home_all_button.setToolTip(
            "Step every servo back to its start, one small step each in turn. Esc stops.")
        self.home_all_button.clicked.connect(lambda: self.on_home(all_servos=True))

        # Jaw sweep: the boxes open with the values last used (or the ones
        # confirmed on the head, tuning_core.SWEEP_BOX_DEFAULTS); the open tick
        # is filled from where the jaw is at each Start.
        ticks = QIntValidator(-1_000_000, 1_000_000, self)
        self.sweep_jaw_edit = QLineEdit()
        self.sweep_open_edit = QLineEdit()
        self.sweep_open_now_button = QPushButton("Jaw now")
        self.sweep_open_now_button.setToolTip(
            "Fill the open tick with where the jaw is now (pretension only, no jaw motion).")
        self.sweep_open_now_button.clicked.connect(self.fill_open_tick_from_jaw)
        self.sweep_stages_edit = QLineEdit()
        self.sweep_pull_edit = QLineEdit()
        self.sweep_pull_edit.setPlaceholderText("servo and the direction that tightens it, "
                                                "e.g. 8+ 11- 14+")
        self.sweep_release_edit = QLineEdit()
        self.sweep_release_edit.setPlaceholderText("tendons that pay out so the jaw can open, "
                                                   "e.g. 11 14 16 19")
        self.sweep_defer_edit = QLineEdit()
        self.sweep_defer_edit.setPlaceholderText("left slack on the way open, e.g. 8 9")
        self.sweep_defer_edit.setToolTip(
            "On the way open these tendons are not pulled back to the preload, so they do "
            "not pull the jaw shut; they are re-tensioned at the start and on the way back.")
        self.sweep_preload_edit = QLineEdit()
        self.sweep_band_edit = QLineEdit()
        self.sweep_push_edit = QLineEdit()
        self.sweep_push_edit.setToolTip(
            "While it moves, the jaw pushes at most this many mA more than it held at the "
            "last stage. When it is held there, the release tendons pay out.")
        self.sweep_abort_edit = QLineEdit()
        self.sweep_travel_edit = QLineEdit()
        for edit in (self.sweep_jaw_edit, self.sweep_stages_edit, self.sweep_preload_edit,
                     self.sweep_band_edit, self.sweep_push_edit, self.sweep_abort_edit,
                     self.sweep_travel_edit):
            edit.setValidator(QIntValidator(0, 100_000, self))
        self.sweep_open_edit.setValidator(ticks)
        self.sweep_button = QPushButton("Start jaw sweep")
        self.sweep_button.setToolTip(
            "Pull every tendon to the preload, then open the jaw in stages and close it "
            "again. The release tendons pay out when the jaw is held at its push limit, "
            "and every tendon is pulled back to the preload at each stage. Each settled "
            "stage is saved. Esc stops.")
        self.sweep_button.clicked.connect(self.on_sweep)
        boxes = tuning_core.load_sweep_boxes(self.records_directory)
        for key, edit in self.sweep_boxes().items():
            edit.setText(boxes[key])

        # The last fault stays here until the next Start, so a later message
        # cannot hide it.
        self.fault_label = QLabel("")
        self.fault_label.setFont(big)
        self.fault_label.setWordWrap(True)
        self.fault_label.setStyleSheet("color: #b00020; font-weight: bold;")

        self.message_label = QLabel("Connect to the board.")
        self.message_label.setFont(big)
        self.message_label.setWordWrap(True)
        self.record_label = QLabel("")

        connection = QHBoxLayout()
        connection.addWidget(QLabel("Port:"))
        connection.addWidget(self.port_edit)
        connection.addWidget(self.connect_button)
        connection.addStretch(1)
        connection.addWidget(self.board_label)

        start_box = QGroupBox("1. Start")
        start_layout = QHBoxLayout(start_box)
        start_layout.addWidget(QLabel("Load limit (mA):"))
        start_layout.addWidget(self.limit_edit)
        start_layout.addWidget(QLabel("Start low. Raise it slowly."))
        start_layout.addWidget(self.start_button)
        start_layout.addStretch(1)
        start_layout.addWidget(self.stop_button)

        move_box = QGroupBox("2. Move one servo")
        move_layout = QGridLayout(move_box)
        move_layout.addWidget(self.selected_label, 0, 0, 1, 6)
        move_layout.addWidget(QLabel("Ticks per click (1 tick = 0.088°):"), 1, 0, 1, 6)
        for column, button in enumerate(self.step_buttons):
            move_layout.addWidget(button, 2, column)
        marks = QHBoxLayout()
        for button in self.mark_buttons.values():
            marks.addWidget(button)
        move_layout.addLayout(marks, 3, 0, 1, 6)
        homes = QHBoxLayout()
        homes.addWidget(self.home_selected_button)
        homes.addWidget(self.home_all_button)
        move_layout.addLayout(homes, 4, 0, 1, 6)

        sweep_box = QGroupBox("3. Jaw sweep: the tendons follow the jaw")
        sweep_layout = QGridLayout(sweep_box)
        sweep_rows = (
            (("Jaw servo:", self.sweep_jaw_edit), ("Open at tick:", self.sweep_open_edit),
             ("Stages:", self.sweep_stages_edit)),
            (("Preload (mA):", self.sweep_preload_edit), ("Band ± (mA):", self.sweep_band_edit),
             ("Jaw push limit (mA):", self.sweep_push_edit)),
            (("Abort at (mA):", self.sweep_abort_edit),
             ("Max tendon travel (ticks):", self.sweep_travel_edit)))
        for row, fields in enumerate(sweep_rows):
            for column, (text, edit) in enumerate(fields):
                sweep_layout.addWidget(QLabel(text), row, 2 * column)
                sweep_layout.addWidget(edit, row, 2 * column + 1)
        sweep_layout.addWidget(QLabel("Tendons:"), 3, 0)
        sweep_layout.addWidget(self.sweep_pull_edit, 3, 1, 1, 5)
        sweep_layout.addWidget(self.sweep_open_now_button, 0, 6)
        sweep_layout.addWidget(QLabel("Release on open:"), 4, 0)
        sweep_layout.addWidget(self.sweep_release_edit, 4, 1, 1, 3)
        sweep_layout.addWidget(self.sweep_button, 4, 4, 1, 2)
        sweep_layout.addWidget(QLabel("Slack while opening:"), 5, 0)
        sweep_layout.addWidget(self.sweep_defer_edit, 5, 1, 1, 3)

        layout = QVBoxLayout()
        layout.addLayout(connection)
        layout.addWidget(start_box)
        layout.addWidget(self.table, 1)
        layout.addWidget(move_box)
        layout.addWidget(sweep_box)
        layout.addWidget(self.fault_label)
        layout.addWidget(self.message_label)
        layout.addWidget(self.record_label)
        central = QWidget()
        central.setLayout(layout)
        self.setCentralWidget(central)
        # Twice the original 820 x 680, made smaller only where the screen
        # cannot fit it.
        size = QSize(1640, 1360)
        screen = QApplication.primaryScreen()
        if screen is not None:
            size = size.boundedTo(screen.availableGeometry().size())
        self.resize(size)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_poll)
        self.refresh_controls()

    # Helpers -------------------------------------------------------------
    def say(self, text: str) -> None:
        self.message_label.setText(text)

    def close_telemetry_log(self) -> None:
        if self.link is not None:
            self.link.sample_listener = None
        if self.telemetry_log is not None:
            self.telemetry_log.close()
            self.telemetry_log = None

    def lose_link(self, text: str) -> None:
        """The board restarted or USB failed: save, drop the link, say so.

        Torque is not ours to remove any more: a restarted board boots with
        torque off, and a board we cannot reach turns torque off when its
        lease runs out. Still try Disable, in case the board can hear it."""
        self.timer.stop()
        if self.record is not None:
            try:
                self.record.save()
            except OSError:
                pass
        self.close_telemetry_log()
        link, self.link = self.link, None
        if link is not None:
            for action in (link.disable, link.device.close):
                try:
                    action()
                except Exception:  # the link is already broken
                    pass
        self.board_label.setText("Board: not connected")
        self.say(text)
        self.refresh_controls()

    def lose_link_for(self, error: Exception) -> None:
        if "rebooted" in str(error):
            self.lose_link("The board restarted. The servos are off. "
                           "Run 'headctl blackbox' and send me the output. "
                           "Then click Connect.")
        else:
            self.lose_link(f"USB connection lost ({error}). The servos turn off "
                           "within 1 s. Click Connect.")

    def selected_servo(self) -> Optional[int]:
        rows = self.table.selectionModel().selectedRows() if self.table.selectionModel() else []
        if not rows:
            return None
        item = self.table.item(rows[0].row(), 0)
        return int(item.text()) if item else None

    def sample(self, index: int) -> Optional[tuning_core.ServoSample]:
        if self.link is None or self.link.state is None:
            return None
        return self.link.state.servos[index]

    def in_tuning(self) -> bool:
        return (self.link is not None and self.link.state is not None and
                self.link.state.state == tuning_core.TUNING_STATE)

    def refresh_controls(self) -> None:
        connected = self.link is not None
        fresh = connected and self.link.state_is_fresh()
        ready_to_start = (fresh and self.link.state.state == tuning_core.HOMING_REQUIRED_STATE and
                          self.limit_edit.hasAcceptableInput() and not self.link.has_control)
        can_move = (fresh and self.in_tuning() and self.link.has_control and
                    not self.link.step_in_flight and self.selected_servo() is not None and
                    self.homer is None and self.sweep is None and not self.backoff.active)
        can_home = bool(fresh and self.in_tuning() and self.link.has_control and
                        self.record is not None and self.homer is None and self.sweep is None)
        self.connect_button.setEnabled(not connected)
        self.port_edit.setEnabled(not connected)
        self.start_button.setEnabled(bool(ready_to_start))
        self.limit_edit.setEnabled(not (connected and self.link.has_control))
        for button in self.step_buttons:
            button.setEnabled(bool(can_move))
        for button in self.mark_buttons.values():
            button.setEnabled(bool(fresh and self.in_tuning() and self.record is not None and
                                   self.selected_servo() is not None and self.homer is None))
        self.home_selected_button.setEnabled(can_home and self.selected_servo() is not None)
        self.home_all_button.setEnabled(can_home)
        self.sweep_button.setEnabled(can_home and not self.backoff.active)
        index = self.selected_servo()
        if index is None:
            self.selected_label.setText("Pick a servo in the table.")
        else:
            self.selected_label.setText(
                f"Servo {index}  (J{self.servo_branches.get(index, -1) + 1})")

    # Actions -------------------------------------------------------------
    def on_connect(self) -> None:
        try:
            device = self.open_device(self.port_edit.text().strip())
            link = tuning_core.TuningLink(device)
            link.connect()
            info = headctl.decode_info(device.get(headctl.GET_INFO, headctl.INFO))
            self.servo_branches = headctl.read_servo_branches(device, info["active_servo_mask"])
        except Exception as error:  # show any failure in plain words
            self.say(f"Cannot connect. {error}")
            return
        self.link = link
        self.offer_profile_limit(device, info["active_servo_mask"])
        self.load_earlier_marks()
        try:
            notes = tuning_core.load_notes(self.records_directory)
        except (OSError, ValueError, KeyError, TypeError) as error:
            notes = {}
            self.say(f"Could not read the notes file: {error}")
        self.filling_notes = True
        self.table.setRowCount(len(self.servo_branches))
        for row, (index, branch) in enumerate(sorted(self.servo_branches.items())):
            for column in range(len(COLUMNS)):
                text = (str(index) if column == 0 else f"J{branch + 1}" if column == 1 else
                        notes.get(index, "") if column == NOTES_COLUMN else "")
                item = QTableWidgetItem(text)
                if column != NOTES_COLUMN:
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                else:
                    item.setToolTip("Double-click to type a note. Saved at once, kept for "
                                    "every later session.")
                self.table.setItem(row, column, item)
        self.filling_notes = False
        self.timer.start(POLL_PERIOD_MS)
        self.say("Connected. Type a load limit, then click Start tuning.")

    def offer_profile_limit(self, device, active_mask: int) -> None:
        try:
            currents = [headctl.decode_slot(device.get(headctl.GET_SLOT, headctl.SLOT,
                                                       bytes((index,))))["operating_current_ma"]
                        for index in range(headctl.HEAD_SERVO_COUNT) if (active_mask >> index) & 1]
        except Exception:  # the default stays; the board still checks the limit
            return
        currents = [value for value in currents if value > 0]
        if currents:
            self.limit_edit.setText(str(min(currents)))

    def on_start(self) -> None:
        limit_ma = int(self.limit_edit.text())
        answer = QMessageBox.question(
            self, "Start tuning",
            f"All servos will hold where they are.\n"
            f"Load limit: {limit_ma} mA.\n\n"
            "Keep the 12 V power switch near you.\nStart now?")
        if answer != QMessageBox.Yes:
            return
        try:
            self.link.start(limit_ma)
        except Exception as error:
            self.say(f"The board did not start tuning. {error}")
            return
        self.load_earlier_marks()
        self.backoff = tuning_core.BackOff()
        self.fault_label.setText("")
        self.record = tuning_core.TuningRecord(
            tuning_core.new_record_path(records_layout.panel_dir(self.records_directory)), limit_ma,
            dict(self.servo_branches))
        self.record.save()
        self.close_telemetry_log()
        try:
            self.telemetry_log = tuning_core.TelemetryLog(self.record.telemetry_path,
                                                          self.record.servo_branches)
            self.link.sample_listener = self.telemetry_log.observe
            log_text = f" and {self.record.telemetry_path.name}"
        except OSError as error:
            log_text = f" (no telemetry log: {error})"
        self.record_label.setText(f"Saving to: {self.record.path}{log_text}")
        # Every servo holds where it is, so this is where the jaw stays.
        self.fill_open_tick_from_jaw()
        self.say("Starting. Wait until the board shows TUNING.")

    def on_stop(self) -> None:
        self.homer = None
        self.end_sweep(None)
        self.backoff = tuning_core.BackOff()
        if self.link is not None:
            try:
                self.link.disable()
            except Exception as error:
                self.lose_link_for(error)
                return
        if self.record is not None:
            self.record.save()
        self.close_telemetry_log()
        self.say("Stopped. The servos go limp.")
        self.refresh_controls()

    def on_step(self, step_ticks: int) -> None:
        index = self.selected_servo()
        if index is None or self.link is None:
            return
        if index in tuning_core.SKIPPED_SERVOS:
            self.say(f"Servo {index} is skipped (faulty eyelid): it is not moved.")
            return
        try:
            self.link.request_step(index, step_ticks)
        except Exception as error:
            self.lose_link_for(error)
            return
        self.refresh_controls()

    def on_item_changed(self, item: QTableWidgetItem) -> None:
        if self.filling_notes or item.column() != NOTES_COLUMN:
            return
        index = int(self.table.item(item.row(), 0).text())
        text = item.text().strip()
        try:
            tuning_core.save_note(self.records_directory, index, text)
            if self.record is not None:
                self.record.notes[index] = text
                self.record.save()
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.say(f"Servo {index}: note NOT saved ({error}).")
            return
        self.say(f"Servo {index}: note saved.")

    def load_earlier_marks(self) -> None:
        self.earlier_marks, skipped = tuning_core.load_earlier_marks(
            self.records_directory, since=tuning_core.HEAD_MARKS_SINCE)
        if skipped:
            self.say("Could not read these record files, so their marks are not used: "
                     + ", ".join(skipped))

    def home_of(self, index: int) -> Optional[int]:
        return tuning_core.home_tick(index, self.record, self.earlier_marks)

    def on_home(self, all_servos: bool) -> None:
        if self.link is None or self.link.state is None or self.record is None:
            return
        indexes = sorted(self.servo_branches) if all_servos else [self.selected_servo()]
        targets = {index: self.home_of(index) for index in indexes if index is not None}
        targets = {index: tick for index, tick in targets.items() if tick is not None}
        skipped = sorted(index for index in targets if index in tuning_core.HOME_SKIPPED_SERVOS)
        eyelids = sorted(index for index in targets if index in tuning_core.SKIPPED_SERVOS)
        targets = {index: tick for index, tick in targets.items()
                   if index not in skipped and index not in eyelids}
        skipped_text = (f" Servo {', '.join(map(str, skipped))} (the jaw) is never moved by "
                        "Home: it fights the tendons. Use the jaw sweep or single steps."
                        if skipped else "")
        if eyelids:
            skipped_text += (f" Servo {', '.join(map(str, eyelids))} (faulty eyelid) is "
                             "skipped.")
        if not targets:
            self.say("Not going home." + skipped_text)
            return
        too_far = tuning_core.home_moves_over_one_turn(targets, self.link.state)
        if too_far:
            self.say("Not going home. No Home may move more than one turn: " + "; ".join(
                f"servo {index} is {distance:+d} ticks ({distance / 4096:+.2f} turns) "
                f"from its start mark" for index, distance in too_far.items()) +
                ". Its turn count has probably changed (servo power cycle), so the mark "
                "is a turn off. Jog it to the right pose and click Mark start again.")
            return
        lines = []
        for index, tick in sorted(targets.items()):
            distance = tick - self.link.state.servos[index].present_tick
            if distance:
                lines.append(f"Servo {index}: {distance:+d} ticks ({distance * DEGREES_PER_TICK:+.1f}°)")
        if not lines:
            self.say("Already at home." + skipped_text)
            return
        answer = QMessageBox.question(
            self, "Go home",
            "These servos will step back to their start mark:\n\n" + "\n".join(lines) +
            ("\n\n" + skipped_text.strip() if skipped_text else "") +
            "\n\nCheck the distances look right. Esc stops at any time.\nGo now?")
        if answer != QMessageBox.Yes:
            return
        # The board may have faulted while the dialog was open.
        if self.link is None or not self.link.has_control or not self.in_tuning():
            self.say("Not going home: the board is no longer in tuning. See the red line.")
            return
        self.homer = tuning_core.HomeMover(
            targets, tuning_core.tuning_max_lead_ticks(self.record.current_limit_ma))
        self.say(f"Going home: {len(targets)} servo(s). Esc stops.")
        self.refresh_controls()

    def sweep_boxes(self) -> Dict[str, QLineEdit]:
        """The sweep boxes that are remembered, by SWEEP_BOX_DEFAULTS key."""
        return {"jaw": self.sweep_jaw_edit, "stages": self.sweep_stages_edit,
                "pull": self.sweep_pull_edit, "release": self.sweep_release_edit,
                "preload_ma": self.sweep_preload_edit, "band_ma": self.sweep_band_edit,
                "jaw_push_ma": self.sweep_push_edit, "abort_ma": self.sweep_abort_edit,
                "max_travel_ticks": self.sweep_travel_edit,
                "defer_on_open": self.sweep_defer_edit}

    def fill_open_tick_from_jaw(self) -> None:
        """Open tick = where the jaw is now; left alone if that is not known."""
        if self.link is None or self.link.state is None or \
                not self.sweep_jaw_edit.hasAcceptableInput():
            return
        jaw = int(self.sweep_jaw_edit.text())
        if jaw in self.servo_branches:
            self.sweep_open_edit.setText(str(self.link.state.servos[jaw].present_tick))

    def sweep_settings(self) -> tuning_core.SweepSettings:
        """The typed sweep settings; ValueError names the first empty box."""
        def number(edit: QLineEdit, name: str) -> int:
            if not edit.hasAcceptableInput():
                raise ValueError(f"Type the {name}.")
            return int(edit.text())
        return tuning_core.SweepSettings(
            jaw=number(self.sweep_jaw_edit, "jaw servo"),
            jaw_open_tick=number(self.sweep_open_edit, "jaw open tick"),
            stages=number(self.sweep_stages_edit, "number of stages"),
            pull=tuning_core.parse_pull_directions(self.sweep_pull_edit.text()),
            release=tuning_core.parse_servo_list(self.sweep_release_edit.text()),
            preload_ma=number(self.sweep_preload_edit, "preload"),
            band_ma=number(self.sweep_band_edit, "band"),
            jaw_push_ma=number(self.sweep_push_edit, "jaw push limit"),
            abort_ma=number(self.sweep_abort_edit, "abort load"),
            max_travel_ticks=number(self.sweep_travel_edit, "max tendon travel"),
            defer_on_open=tuning_core.parse_servo_list(self.sweep_defer_edit.text()))

    def on_sweep(self) -> None:
        if self.link is None or self.link.state is None or self.record is None:
            return
        try:
            settings = self.sweep_settings()
        except ValueError as error:
            self.say(f"Jaw sweep not started. {error}")
            return
        problems = tuning_core.check_sweep_settings(settings, sorted(self.servo_branches),
                                                    self.record.current_limit_ma)
        if problems:
            self.say("Jaw sweep not started. " + " ".join(problems))
            return
        jaw_now = self.link.state.servos[settings.jaw].present_tick
        travel = settings.jaw_open_tick - jaw_now
        if abs(travel) > tuning_core.headctl_one_turn_ticks():
            self.say(f"Jaw sweep not started: the open tick is {travel:+d} ticks from where "
                     "the jaw is, more than one turn. Check the Position column.")
            return
        pulls = "\n".join(f"Servo {index} tightens toward {'HIGHER' if sign > 0 else 'LOWER'} ticks"
                          + (" (pays out so the jaw can open)." if index in settings.release
                             else ".")
                          for index, sign in sorted(settings.pull.items()))
        answer = QMessageBox.question(
            self, "Jaw sweep",
            f"First every tendon is pulled to {settings.preload_ma} ± {settings.band_ma} mA "
            f"with the jaw where it is.\n"
            f"Then the jaw (servo {settings.jaw}) goes from {jaw_now} to "
            f"{settings.jaw_open_tick} ({travel:+d} ticks, {travel * DEGREES_PER_TICK:+.1f}°) "
            f"in {settings.stages} stages and back. While it moves, it pushes at most "
            f"{settings.jaw_push_ma} mA more than it held at the last stage; held there on "
            "the way open, the release tendons pay out. At each stage every tendon is pulled "
            "back to the preload.\n\n"
            f"{pulls}\n\nA wrong direction unwinds the line. Check each one.\n\n"
            f"It stops and goes limp if the jaw or a tendon reaches {settings.abort_ma} mA. "
            f"It stops and holds if a tendon would move more than "
            f"{settings.max_travel_ticks} ticks, or a stage does not finish in "
            f"{tuning_core.SWEEP_STAGE_TIMEOUT_S:.0f} s.\n"
            "Esc stops at any time.\nStart now?")
        if answer != QMessageBox.Yes:
            return
        # The board may have faulted while the dialog was open.
        if self.link is None or not self.link.has_control or not self.in_tuning():
            self.say("Jaw sweep not started: the board is no longer in tuning. See the red line.")
            return
        sweep = tuning_core.JawSweep(
            tuning_core.new_sweep_path(records_layout.panel_dir(self.records_directory)), settings,
            self.link.state, self.link.clock())
        try:
            sweep.save()
        except OSError as error:
            self.say(f"Jaw sweep not started: cannot write {sweep.path} ({error}).")
            return
        self.sweep = sweep
        try:
            tuning_core.save_sweep_boxes(self.records_directory,
                                         {key: edit.text().strip()
                                          for key, edit in self.sweep_boxes().items()})
            saved_text = ""
        except OSError as error:
            saved_text = f" (The box values were not saved: {error}.)"
        self.say(f"Jaw sweep: pulling the tendons to {settings.preload_ma} mA. Esc stops."
                 + saved_text)
        self.refresh_controls()

    def end_sweep(self, text: Optional[str]) -> None:
        """Save and forget the sweep; the servos hold where they are."""
        sweep, self.sweep = self.sweep, None
        if sweep is None:
            return
        try:
            sweep.save()
        except OSError as error:
            text = f"{text or ''} The sweep file was NOT saved ({error})."
        if text:
            self.say(text)

    @staticmethod
    def unverified_text(sweep: tuning_core.JawSweep) -> str:
        """A warning for tendons that hardly moved at the pretension."""
        if not sweep.rows or not sweep.rows[0]["unverified"]:
            return ""
        first = sweep.rows[0]
        servos = ", ".join(f"{index} ({first['taken_in_ticks'][str(index)]} ticks)"
                           for index in first["unverified"])
        return (f" Hardly moved at the pretension: servo {servos}. Their load may be "
                "friction, not line tension: check those lines by eye.")

    def service_sweep(self) -> bool:
        """Run the jaw sweep. True while one is running."""
        sweep = self.sweep
        if sweep is None or self.link is None or self.link.state is None:
            return False
        if not self.link.has_control or not self.in_tuning():
            self.end_sweep("Jaw sweep stopped: the board left tuning. See the red line.")
            return False
        row = sweep.observe(self.link.state)
        if row is not None:
            if sweep.done:
                self.end_sweep(f"Jaw sweep finished: {len(sweep.rows)} stages saved to "
                               f"{sweep.path.name}. The servos hold here."
                               + self.unverified_text(sweep))
                return False
            try:
                sweep.save()
            except OSError as error:
                self.end_sweep(f"Jaw sweep stopped: cannot write {sweep.path} ({error}). "
                               "The servos hold here.")
                return False
            self.say(f"Jaw sweep: stage {row['stage']} saved. Next: jaw to {sweep.jaw_target} "
                     f"({sweep.leg}). Esc stops." + self.unverified_text(sweep))
        if sweep.stopped is not None:
            text = (f"Jaw sweep stopped: {sweep.stopped} {len(sweep.rows)} stage(s) saved to "
                    f"{sweep.path.name}." + self.unverified_text(sweep))
            if sweep.torque_off:
                self.on_stop()
                self.say(f"{text} Torque off requested.")
            else:
                self.end_sweep(f"{text} The servos hold here.")
            return False
        if not self.link.step_in_flight and self.link.state_is_fresh():
            step = sweep.next_step(self.link.state)
            if step is not None:
                self.link.request_step(*step)
        return True

    def service_backoff(self) -> bool:
        """Release servos pushing at the load limit. True while one is active."""
        if self.link is None or self.link.state is None or self.record is None:
            return False
        if not self.link.has_control or not self.in_tuning():
            return False
        for index in self.backoff.observe(self.link.state, self.record.current_limit_ma,
                                          sorted(self.servo_branches)):
            self.start_backoff_message(index)
        if not self.backoff.active:
            return False
        before = set(self.backoff.active)
        if not self.link.step_in_flight and self.link.state_is_fresh():
            step = self.backoff.next_step(self.link.state)
            if step is not None:
                self.link.request_step(*step)
        for index in sorted(before - self.backoff.active):
            servo = self.link.state.servos[index]
            self.say(f"Servo {index} released at {servo.present_tick}, now {servo.current_ma} mA. "
                     "If this is the end, click Mark end.")
        return True

    def start_backoff_message(self, index: int) -> None:
        if self.homer is not None and index in self.homer.remaining:
            self.homer.given_up[index] = tuning_core.EBUSY_RESULT
        sweep_text = ""
        if self.sweep is not None:
            self.end_sweep(None)
            sweep_text = " The jaw sweep stopped; its saved stages are kept."
        self.say(f"Servo {index} is pushing at the load limit and not moving. "
                 f"Releasing it: its goal is stepped back to where it is.{sweep_text}")

    def service_home(self) -> None:
        if self.homer is None or self.link is None or self.link.state is None:
            return
        if not self.link.has_control or not self.in_tuning():
            self.homer = None
            self.say("Home stopped: the board left tuning. See the red line.")
            return
        if self.homer.notices:
            self.say(" ".join(self.homer.notices))
            self.homer.notices.clear()
        if self.homer.is_done(self.link.state):
            given_up = sorted(self.homer.given_up)
            held = ", ".join(f"servo {index} {abs(self.link.state.servos[index].current_ma)} mA"
                             for index in sorted(self.homer.arrived))
            self.homer = None
            self.say(("Home reached" if not given_up else
                      "Home reached, except servo(s) " + ", ".join(map(str, given_up))) +
                     (f". Holding: {held}." if held else "."))
            return
        if self.link.step_in_flight or not self.link.state_is_fresh():
            return
        step = self.homer.next_step(self.link.state)
        if step is not None:
            self.link.request_step(*step)

    def on_mark(self, name: str) -> None:
        index = self.selected_servo()
        sample = self.sample(index) if index is not None else None
        if sample is None or self.record is None:
            return
        self.record.add_mark(sample, name)
        self.record.save()
        label = MARK_LABELS[name]
        self.say(f"Servo {index}: {label} marked at {sample.present_tick}.")

    def on_poll(self) -> None:
        if self.link is None:
            return
        try:
            self.link.poll(0.005)
        except Exception as error:
            self.lose_link_for(error)
            return
        state = self.link.state
        if self.telemetry_log is not None and self.telemetry_log.error:
            self.say(self.telemetry_log.error)
            self.close_telemetry_log()
        if state is not None:
            self.board_label.setText(f"Board: {state.state_name}"
                                     + (f"  ({state.fault_name})" if state.fault else ""))
            if self.record is not None:
                self.record.observe(state)
        while self.link.step_results:
            index, direction, result = self.link.step_results.pop(0)
            sample = self.sample(index)
            if self.record is not None and sample is not None:
                self.record.add_step(sample, direction, result)
                self.record.save()
            if index in self.backoff.active:
                self.backoff.on_result(index, self.link.clock())
                if result != 0:
                    self.backoff.give_up(index)
                    self.say(f"Servo {index}: could not release it "
                             f"({tuning_core.step_result_text(result)}) Step it back by hand "
                             "or press Esc.")
                continue
            if result == tuning_core.EBUSY_RESULT and self.backoff.on_refused(index, state):
                self.backoff.on_result(index, self.link.clock())
                self.start_backoff_message(index)
                continue
            if self.sweep is not None:
                self.sweep.on_result(index, result, self.link.clock())
            elif self.homer is not None:
                notice = self.homer.on_result(index, result, self.link.clock())
                if notice:
                    self.say(notice)
            else:
                self.say(f"Servo {index}: {tuning_core.step_result_text(result)}")
        if self.link.control_lost_reason and self.link.token:
            reason = self.link.control_lost_reason
            self.fault_label.setText(reason)
            if self.record is not None and state is not None and \
                    state.state == tuning_core.FAULT_STATE:
                self.record.add_fault(state)
                self.record.save()
            try:
                self.link.disable()
            except Exception as error:
                self.lose_link_for(error)
                return
            self.homer = None
            self.end_sweep(None)
            log_text = (f" Telemetry before the fault: {self.telemetry_log.path.name}."
                        if self.telemetry_log is not None else "")
            self.say(f"{reason} Torque off requested.{log_text}")
        try:
            if not self.service_backoff() and not self.service_sweep():
                self.service_home()
        except Exception as error:
            self.lose_link_for(error)
            return
        self.update_table()
        self.refresh_controls()

    def update_table(self) -> None:
        if self.link is None or self.link.state is None:
            return
        for row in range(self.table.rowCount()):
            index = int(self.table.item(row, 0).text())
            sample = self.link.state.servos[index]
            home = self.home_of(index)
            peak = self.record.peak_current_ma.get(index, 0) if self.record else 0
            current_marks = self.record.marks.get(index, {}) if self.record else {}
            earlier = self.earlier_marks.get(index, {})
            marks = [MARK_LABELS[name] if name in current_marks else
                     f"{MARK_LABELS[name]} (earlier)"
                     for name in tuning_core.MARK_NAMES
                     if name in current_marks or name in earlier]
            ready = tuning_core.is_ready_for_profile({**earlier, **current_marks})
            pwm = self.link.present_pwm_raw
            values = (str(sample.present_tick),
                      "" if home is None else f"{sample.present_tick - home:+d}",
                      str(sample.current_ma), str(peak),
                      "" if pwm is None else
                      f"{pwm[index] * headctl.PRESENT_PWM_PERCENT_PER_UNIT:.0f}",
                      str(sample.temperature_c),
                      ("✓ " if ready else "") + ", ".join(marks))
            for column, text in enumerate(values, start=2):
                self.table.item(row, column).setText(text)

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override
        QApplication.instance().removeEventFilter(self.escape_filter)
        self.on_stop()
        super().closeEvent(event)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=headctl.AUTO_PORT)
    parser.add_argument("--records", type=Path, default=DEFAULT_RECORDS,
                        help="folder for tuning record files")
    parser.add_argument("--preview", action="store_true",
                        help="use a simulated board; nothing is sent to hardware")
    arguments = parser.parse_args()
    if arguments.preview:
        simulated = tuning_core.preview_controller()
        open_device = lambda _port: simulated  # noqa: E731
        records = arguments.records / "preview"
    else:
        open_device = headctl.Device
        records = arguments.records
    application = QApplication(sys.argv)
    window = TuningWindow(open_device, arguments.port, records)

    # PyQt aborts the whole process on an exception nobody caught in a slot.
    # Never let that happen here: report it, and treat the link as lost.
    def report_unexpected(kind, error, trace):
        sys.__excepthook__(kind, error, trace)
        if isinstance(error, KeyboardInterrupt):
            window.close()
            return
        window.lose_link(f"Unexpected error: {error}. The window is safe to close.")

    sys.excepthook = report_unexpected
    # Ctrl-C in the terminal closes the window, which runs the same stop path
    # as the STOP button. Python only sees the signal while it runs, so a
    # short idle timer hands it control between Qt events.
    signal.signal(signal.SIGINT, lambda _signal, _frame: window.close())
    idle = QTimer()
    idle.timeout.connect(lambda: None)
    idle.start(100)
    window.show()
    sys.exit(application.exec_())


if __name__ == "__main__":
    main()
