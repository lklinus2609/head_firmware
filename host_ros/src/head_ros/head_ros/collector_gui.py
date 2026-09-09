"""Qt operator panel for collection and the firmware routing hold."""

from __future__ import annotations

import sys
from types import SimpleNamespace
from typing import Any, Optional
import uuid

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    from head_msgs.msg import (HeadState, ProprioceptionEvent,
                               ProprioceptionSessionState)
    from head_msgs.srv import (AbortProprioceptionSession, AcquireControl,
                               Disable, PrepareProprioceptionSession,
                               ReleaseControl, RenewControl,
                               SetProprioceptionRecording, StartRouting)
    ROS_IMPORT_ERROR: Optional[ImportError] = None
except ImportError as error:  # Allows the backend-free GUI preview.
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[misc,assignment]
    QoSProfile = ReliabilityPolicy = None  # type: ignore[assignment]
    HeadState = ProprioceptionEvent = None  # type: ignore[assignment]
    AbortProprioceptionSession = None  # type: ignore[assignment]
    AcquireControl = Disable = None  # type: ignore[assignment]
    PrepareProprioceptionSession = None  # type: ignore[assignment]
    ReleaseControl = RenewControl = None  # type: ignore[assignment]
    SetProprioceptionRecording = None  # type: ignore[assignment]
    StartRouting = None  # type: ignore[assignment]
    ROS_IMPORT_ERROR = error


class SessionStateValues:
    """Constants shared by generated ROS messages and preview messages."""

    IDLE = 0
    ACQUIRING = 1
    HOMING = 2
    ENTERING_HOLD = 3
    WARMING_UP = 4
    ARMED = 5
    RECORDING = 6
    STOPPING = 7
    SAVED = 8
    ABORTING = 9
    ERROR = 10


class HeadStateValues:
    HOMING_REQUIRED = 2
    READY = 5
    FAULT = 7
    ROUTING = 10
    NAMES = {
        0: "BOOT",
        2: "HOMING_REQUIRED",
        3: "HOMING",
        4: "MAINTENANCE_CALIBRATION",
        5: "READY",
        6: "ENABLED",
        7: "FAULT",
        8: "PROPRIOCEPTION_SETTLING",
        9: "PROPRIOCEPTION_HOLD",
        10: "ROUTING",
    }

try:
    from python_qt_binding.QtCore import QTimer, Qt
    from python_qt_binding.QtGui import QKeySequence, QShortcut
    from python_qt_binding.QtWidgets import (
        QApplication, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
        QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton,
        QSpinBox, QTableWidget, QTableWidgetItem, QTextEdit, QVBoxLayout,
        QWidget,
    )
except ImportError:
    try:
        from PyQt5.QtCore import QTimer, Qt
        from PyQt5.QtGui import QKeySequence
        from PyQt5.QtWidgets import (
            QApplication, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
            QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton,
            QShortcut, QSpinBox, QTableWidget, QTableWidgetItem, QTextEdit,
            QVBoxLayout, QWidget,
        )
    except ImportError as error:  # pragma: no cover - environment dependency
        raise RuntimeError(
            "install ROS 2 python_qt_binding or PyQt5 to display the GUI") from error


class CollectionGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("proprioception_collection_gui")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.head: Optional[HeadState] = None
        self.session: Optional[ProprioceptionSessionState] = None
        self.create_subscription(HeadState, "/head/state", self._on_head, qos)
        self.create_subscription(ProprioceptionSessionState,
                                 "/proprioception/session_state",
                                 self._on_session, 20)
        self.event_pub = self.create_publisher(
            ProprioceptionEvent, "/proprioception/events", 20)
        self.prepare_client = self.create_client(
            PrepareProprioceptionSession, "/proprioception/prepare_session")
        self.record_client = self.create_client(
            SetProprioceptionRecording, "/proprioception/set_recording")
        self.abort_client = self.create_client(
            AbortProprioceptionSession, "/proprioception/abort_session")
        self.acquire_client = self.create_client(
            AcquireControl, "/head/acquire_control")
        self.routing_client = self.create_client(
            StartRouting, "/head/start_routing")
        self.renew_client = self.create_client(
            RenewControl, "/head/renew_control")
        self.disable_client = self.create_client(Disable, "/head/disable")
        self.release_client = self.create_client(
            ReleaseControl, "/head/release_control")
        self.head_callback: Any = None
        self.session_callback: Any = None
        self.result_callback: Any = None
        self.contact_event_id = ""
        self.routing_lease_token = 0
        self.routing_pending = False
        self.routing_seen_active = False
        self.routing_renew_future: Any = None
        self.create_timer(0.4, self._renew_routing)

    def _on_head(self, message: HeadState) -> None:
        self.head = message
        if message.state == HeadStateValues.ROUTING:
            self.routing_seen_active = True
        elif (self.routing_seen_active and
              message.state in (HeadStateValues.HOMING_REQUIRED,
                                HeadStateValues.FAULT)):
            self.routing_lease_token = 0
            self.routing_seen_active = False
        if self.head_callback:
            self.head_callback(message)

    def _on_session(self, message: ProprioceptionSessionState) -> None:
        self.session = message
        if self.session_callback:
            self.session_callback(message)

    def _report(self, future: Any) -> None:
        try:
            result = future.result()
            text = getattr(result, "reason", getattr(result, "status", "request complete"))
        except Exception as error:
            text = f"request failed: {error}"
        if self.result_callback:
            self.result_callback(text)

    def prepare(self, values: dict[str, Any]) -> None:
        request = PrepareProprioceptionSession.Request(**values)
        future = self.prepare_client.call_async(request)
        future.add_done_callback(self._report)

    def set_recording(self, recording: bool) -> None:
        request = SetProprioceptionRecording.Request(recording=recording)
        future = self.record_client.call_async(request)
        future.add_done_callback(self._report)

    def abort(self, reason: str) -> Any:
        request = AbortProprioceptionSession.Request(reason=reason)
        future = self.abort_client.call_async(request)
        future.add_done_callback(self._report)
        return future

    def start_routing(self) -> None:
        if self.routing_pending or self.routing_lease_token:
            return
        if not self.acquire_client.service_is_ready():
            self._notify("/head/acquire_control is unavailable")
            return
        self.routing_pending = True
        future = self.acquire_client.call_async(
            AcquireControl.Request(requester="routing_gui"))
        future.add_done_callback(self._on_routing_acquired)

    def _on_routing_acquired(self, future: Any) -> None:
        try:
            result = future.result()
            if not result.granted:
                raise RuntimeError(result.reason)
            self.routing_lease_token = int(result.lease_token)
            if not self.routing_client.service_is_ready():
                raise RuntimeError("/head/start_routing is unavailable")
            started = self.routing_client.call_async(
                StartRouting.Request(lease_token=self.routing_lease_token))
            started.add_done_callback(self._on_routing_started)
            return
        except Exception as error:
            self.routing_pending = False
            self._release_routing_lease()
            self._notify(f"routing request failed: {error}")

    def _on_routing_started(self, future: Any) -> None:
        self.routing_pending = False
        try:
            result = future.result()
            if not result.started:
                raise RuntimeError(result.reason)
            self._notify("Routing hold active: all servos are moving to 0 degrees")
        except Exception as error:
            self._release_routing_lease()
            self._notify(f"routing request failed: {error}")

    def stop_routing(self) -> Any:
        if self.routing_lease_token == 0:
            self._notify(
                "No local routing lease; firmware watchdog will disable if its owner is gone")
            return None
        if not self.disable_client.service_is_ready():
            self._notify(
                "/head/disable is unavailable; firmware watchdog will disable")
            return None
        future = self.disable_client.call_async(
            Disable.Request(lease_token=self.routing_lease_token))
        future.add_done_callback(self._on_routing_stopped)
        return future

    def _on_routing_stopped(self, future: Any) -> None:
        try:
            result = future.result()
            if not result.disabled:
                raise RuntimeError(result.reason)
            self.routing_lease_token = 0
            self._notify("Routing hold disabled; servo torque is off")
        except Exception as error:
            self._notify(f"routing disable failed: {error}")

    def _renew_routing(self) -> None:
        if (self.routing_lease_token == 0 or self.routing_pending or
                self.head is None or self.head.state != HeadStateValues.ROUTING or
                (self.routing_renew_future is not None and
                 not self.routing_renew_future.done())):
            return
        if not self.renew_client.service_is_ready():
            self.routing_lease_token = 0
            self._notify(
                "/head/renew_control is unavailable; firmware watchdog will disable")
            return
        self.routing_renew_future = self.renew_client.call_async(
            RenewControl.Request(lease_token=self.routing_lease_token))
        self.routing_renew_future.add_done_callback(self._on_routing_renewed)

    def _on_routing_renewed(self, future: Any) -> None:
        try:
            result = future.result()
            if result.renewed:
                return
            raise RuntimeError(result.reason)
        except Exception as error:
            self.routing_lease_token = 0
            self._notify(
                f"routing heartbeat failed: {error}; firmware watchdog will disable")

    def _release_routing_lease(self) -> None:
        token = self.routing_lease_token
        self.routing_lease_token = 0
        if token and self.release_client.service_is_ready():
            self.release_client.call_async(ReleaseControl.Request(lease_token=token))

    def _notify(self, text: str) -> None:
        if self.result_callback:
            self.result_callback(text)

    def mark_contact(self, active: bool, region: str) -> None:
        if active:
            self.contact_event_id = f"manual_{uuid.uuid4().hex}"
        elif not self.contact_event_id:
            if self.result_callback:
                self.result_callback("no manual contact is active")
            return
        message = ProprioceptionEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.kind = (ProprioceptionEvent.CONTACT_START if active
                        else ProprioceptionEvent.CONTACT_END)
        message.session_id = self.session.session_id if self.session else ""
        message.event_id = self.contact_event_id
        message.region = region
        message.source = "manual_gui"
        message.target_force_n = float("nan")
        message.measured_force_n = float("nan")
        message.indentation_mm = float("nan")
        message.mcu_uptime_ms = self.head.mcu_uptime_ms if self.head else 0
        message.notes = "manual operator marker"
        self.event_pub.publish(message)
        if not active:
            self.contact_event_id = ""


class CollectionWindow(QMainWindow):
    def __init__(self, node: CollectionGuiNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("Project HEAD — Proprioception Collection")
        self.resize(1100, 760)
        node.head_callback = self.update_head
        node.session_callback = self.update_session
        node.result_callback = self.set_notice

        root = QWidget()
        outer = QVBoxLayout(root)

        status_box = QGroupBox("Controller and collection status")
        status_layout = QGridLayout(status_box)
        self.head_state = QLabel("No /head/state")
        self.session_state = QLabel("Collector unavailable")
        self.notice = QLabel("")
        self.notice.setWordWrap(True)
        self.output_path = QLabel("")
        self.output_path.setTextInteractionFlags(Qt.TextSelectableByMouse)
        status_layout.addWidget(QLabel("Firmware:"), 0, 0)
        status_layout.addWidget(self.head_state, 0, 1)
        status_layout.addWidget(QLabel("Collection:"), 1, 0)
        status_layout.addWidget(self.session_state, 1, 1)
        status_layout.addWidget(QLabel("Output:"), 2, 0)
        status_layout.addWidget(self.output_path, 2, 1)
        status_layout.addWidget(self.notice, 3, 0, 1, 2)
        outer.addWidget(status_box)

        metadata_box = QGroupBox("Session metadata")
        metadata_layout = QFormLayout(metadata_box)
        self.operator = QLineEdit()
        self.skin = QLineEdit()
        self.pose = QLineEdit("neutral")
        self.fixture = QLineEdit()
        self.region = QLineEdit("cheek")
        self.notes = QTextEdit()
        self.notes.setMaximumHeight(70)
        self.warmup = QSpinBox()
        self.warmup.setRange(2000, 300000)
        self.warmup.setValue(10000)
        self.warmup.setSuffix(" ms")
        metadata_layout.addRow("Operator", self.operator)
        metadata_layout.addRow("Skin installation", self.skin)
        metadata_layout.addRow("Pose", self.pose)
        metadata_layout.addRow("Fixture", self.fixture)
        metadata_layout.addRow("Contact region", self.region)
        metadata_layout.addRow("Warm-up", self.warmup)
        metadata_layout.addRow("Notes", self.notes)
        outer.addWidget(metadata_box)

        buttons = QHBoxLayout()
        self.prepare_button = QPushButton("Prepare / Home / Hold")
        self.routing_button = QPushButton("Enter routing hold (0°)")
        self.record_button = QPushButton("Start recording [Space]")
        self.abort_button = QPushButton("ABORT / DISABLE")
        self.contact_start_button = QPushButton("Contact start")
        self.contact_end_button = QPushButton("Contact end")
        self.abort_button.setStyleSheet(
            "QPushButton { background-color: #a52020; color: white; font-weight: bold; }")
        self.prepare_button.clicked.connect(self.prepare)
        self.routing_button.clicked.connect(self.toggle_routing)
        self.record_button.clicked.connect(self.toggle_recording)
        self.abort_button.clicked.connect(self.abort_or_disable)
        self.contact_start_button.clicked.connect(
            lambda: self.node.mark_contact(True, self.region.text().strip()))
        self.contact_end_button.clicked.connect(
            lambda: self.node.mark_contact(False, self.region.text().strip()))
        buttons.addWidget(self.prepare_button)
        buttons.addWidget(self.routing_button)
        buttons.addWidget(self.record_button)
        buttons.addWidget(self.contact_start_button)
        buttons.addWidget(self.contact_end_button)
        buttons.addWidget(self.abort_button)
        outer.addLayout(buttons)

        self.servo_table = QTableWidget(0, 8)
        self.servo_table.setHorizontalHeaderLabels([
            "Servo", "Current mA", "Goal tick", "Present tick", "Velocity",
            "Temp C", "Age ms", "Status",
        ])
        self.servo_table.setAlternatingRowColors(True)
        outer.addWidget(self.servo_table)
        outer.addWidget(QLabel(
            "Software Disable is not an emergency stop. Keep physical actuator-power removal accessible."))
        self.setCentralWidget(root)

        self.space_shortcut = QShortcut(QKeySequence(Qt.Key_Space), self)
        self.space_shortcut.activated.connect(self.toggle_recording)
        self.update_buttons()

    def set_notice(self, text: str) -> None:
        self.notice.setText(text)

    def prepare(self) -> None:
        if not self.operator.text().strip() or not self.skin.text().strip():
            QMessageBox.warning(self, "Missing metadata",
                                "Operator and skin installation are required.")
            return
        self.node.prepare({
            "operator_id": self.operator.text().strip(),
            "skin_installation_id": self.skin.text().strip(),
            "pose_id": self.pose.text().strip(),
            "fixture_id": self.fixture.text().strip(),
            "contact_region": self.region.text().strip(),
            "notes": self.notes.toPlainText().strip(),
            "warmup_ms": self.warmup.value(),
        })

    def toggle_recording(self) -> None:
        state = self.node.session
        if state is None:
            self.set_notice("Collector state is unavailable")
        elif state.can_start_recording:
            self.node.set_recording(True)
        elif state.can_stop_recording:
            self.node.set_recording(False)
        else:
            self.set_notice(f"Space ignored: {state.status}")

    def toggle_routing(self) -> None:
        head = self.node.head
        if head is not None and head.state == HeadStateValues.ROUTING:
            self.node.stop_routing()
            return
        if not getattr(self.node, "preview_mode", False):
            answer = QMessageBox.question(
                self, "Enter routing hold?",
                "Every active servo will move to its 0-degree encoder position "
                "and hold torque for tendon routing. Keep actuator-power removal accessible.",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return
        self.node.start_routing()
        self.update_buttons()

    def abort_or_disable(self) -> None:
        head = self.node.head
        if head is not None and head.state == HeadStateValues.ROUTING:
            self.node.stop_routing()
        else:
            self.node.abort("operator pressed ABORT / DISABLE")

    def update_head(self, message: HeadState) -> None:
        state_name = HeadStateValues.NAMES.get(
            message.state, f"UNKNOWN({message.state})")
        self.head_state.setText(
            f"state={state_name} fault={message.fault} "
            f"torque={'on' if message.torque_enabled else 'off'} "
            f"boot={message.boot_session_id:08x} fan={message.fan_rpm} rpm")
        active = [servo for index, servo in enumerate(message.servos)
                  if message.active_servo_mask & (1 << index)]
        self.servo_table.setRowCount(len(active))
        for row, servo in enumerate(active):
            values = [
                servo.name, str(servo.current_ma), str(servo.goal_tick),
                str(servo.present_tick), str(servo.present_velocity_raw),
                str(servo.temperature_c), str(servo.feedback_age_ms),
                "OK" if servo.online and not servo.hardware_error else
                f"offline/error {servo.hardware_error}",
            ]
            for column, value in enumerate(values):
                self.servo_table.setItem(row, column, QTableWidgetItem(value))
        self.update_buttons()

    def update_session(self, message: ProprioceptionSessionState) -> None:
        self.session_state.setText(
            f"{message.status} — samples={message.samples_received}, "
            f"invalid={message.samples_invalid}, warm-up={message.warmup_remaining_ms} ms")
        self.output_path.setText(message.output_path)
        self.update_buttons()

    def update_buttons(self) -> None:
        session = self.node.session
        idle = session is None or session.state in (
            SessionStateValues.IDLE, SessionStateValues.SAVED,
            SessionStateValues.ERROR)
        head = self.node.head
        routing = bool(head and head.state == HeadStateValues.ROUTING)
        routing_pending = bool(getattr(self.node, "routing_pending", False))
        self.prepare_button.setEnabled(idle and not routing and not routing_pending)
        can_enter_routing = bool(
            idle and head and head.state in (
                HeadStateValues.HOMING_REQUIRED, HeadStateValues.READY))
        can_exit_routing = bool(
            routing and getattr(self.node, "routing_lease_token", 0))
        self.routing_button.setEnabled(
            not routing_pending and (can_enter_routing or can_exit_routing))
        self.routing_button.setText(
            "Disable routing hold" if routing else
            "Entering routing hold…" if routing_pending else
            "Enter routing hold (0°)")
        self.routing_button.setStyleSheet(
            "QPushButton { background-color: #d98200; color: white; font-weight: bold; }"
            if routing else "")
        can_start = bool(session and session.can_start_recording)
        can_stop = bool(session and session.can_stop_recording)
        self.record_button.setEnabled(can_start or can_stop)
        self.record_button.setText(
            "End recording [Space]" if can_stop else "Start recording [Space]")
        self.contact_start_button.setEnabled(can_stop and not self.node.contact_event_id)
        self.contact_end_button.setEnabled(can_stop and bool(self.node.contact_event_id))
        self.abort_button.setEnabled(not idle or routing)

    def closeEvent(self, event: Any) -> None:
        if getattr(self.node, "preview_mode", False):
            event.accept()
            return
        head = self.node.head
        if head is not None and head.state == HeadStateValues.ROUTING:
            answer = QMessageBox.question(
                self, "Disable routing hold?",
                "Closing will request torque disable. Keep this window open if "
                "the controller does not acknowledge it.",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                event.ignore()
                return
            future = self.node.stop_routing()
            if future is None:
                QMessageBox.critical(
                    self, "Disable unavailable",
                    "This GUI does not own the routing lease. Wait for the firmware "
                    "watchdog or remove actuator power.")
                event.ignore()
                return
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            try:
                acknowledged = bool(future.done() and future.result().disabled)
            except Exception:
                acknowledged = False
            if not acknowledged:
                QMessageBox.critical(
                    self, "Disable not acknowledged",
                    "Keep this window open and remove actuator power if needed.")
                event.ignore()
                return
        session = self.node.session
        if session and session.state not in (
                SessionStateValues.IDLE,
                SessionStateValues.SAVED,
                SessionStateValues.ERROR):
            answer = QMessageBox.question(
                self, "Abort active session?",
                "Closing will request torque disable and save an aborted session.",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                event.ignore()
                return
            future = self.node.abort("collection GUI closed")
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            try:
                acknowledged = bool(future.done() and future.result().accepted)
            except Exception:
                acknowledged = False
            if not acknowledged:
                QMessageBox.critical(
                    self, "Abort not acknowledged",
                    "The collector did not acknowledge the abort. Keep this "
                    "window open and remove actuator power if needed.")
                event.ignore()
                return
        event.accept()


class PreviewNode:
    """Small in-process backend used only to inspect and exercise the GUI."""

    preview_mode = True

    def __init__(self) -> None:
        self.head_callback: Any = None
        self.session_callback: Any = None
        self.result_callback: Any = None
        self.contact_event_id = ""
        self.routing_lease_token = 0
        self.routing_pending = False
        self.head = SimpleNamespace(
            state=5,
            fault=0,
            torque_enabled=False,
            boot_session_id=0x4A31C2D0,
            fan_rpm=1375,
            mcu_uptime_ms=84217,
            active_servo_mask=0b1111,
            servos=[
                SimpleNamespace(
                    name=name, current_ma=current, goal_tick=goal,
                    present_tick=present, present_velocity_raw=velocity,
                    temperature_c=temperature, feedback_age_ms=age,
                    online=online, hardware_error=hardware_error)
                for name, current, goal, present, velocity, temperature, age,
                online, hardware_error in (
                    ("jaw_left", 182, 2110, 2108, 2, 34, 5, True, 0),
                    ("jaw_right", 176, 1986, 1988, -1, 35, 4, True, 0),
                    ("neck_pitch", 91, 2048, 2047, 0, 32, 6, True, 0),
                    ("neck_yaw", 0, 2048, 2048, 0, 0, 128, False, 1),
                )
            ],
        )
        self.session = SimpleNamespace(
            state=SessionStateValues.IDLE,
            session_id="preview-20260903-001",
            status="idle — routing controls available",
            output_path="",
            can_start_recording=False,
            can_stop_recording=False,
            samples_received=12840,
            samples_invalid=3,
            warmup_remaining_ms=0,
        )

    def _refresh(self) -> None:
        if self.session_callback:
            self.session_callback(self.session)

    def prepare(self, values: dict[str, Any]) -> None:
        self.session.state = SessionStateValues.ARMED
        self.session.status = f"armed for {values['contact_region']}"
        self.session.can_start_recording = True
        self.session.can_stop_recording = False
        self._refresh()
        if self.result_callback:
            self.result_callback("Preview: session preparation complete")

    def set_recording(self, recording: bool) -> None:
        if recording:
            self.session.state = SessionStateValues.RECORDING
            self.session.status = "recording preview samples"
            self.session.can_start_recording = False
            self.session.can_stop_recording = True
        else:
            self.session.state = SessionStateValues.SAVED
            self.session.status = "saved preview session"
            self.session.output_path = "/preview/data/raw/preview-20260903-001"
            self.session.can_start_recording = False
            self.session.can_stop_recording = False
        self._refresh()
        if self.result_callback:
            self.result_callback(
                "Preview: recording started" if recording
                else "Preview: recording stopped and session saved")

    def abort(self, reason: str) -> None:
        self.session.state = SessionStateValues.SAVED
        self.session.status = "preview session aborted"
        self.session.can_start_recording = False
        self.session.can_stop_recording = False
        self.contact_event_id = ""
        self._refresh()
        if self.result_callback:
            self.result_callback(f"Preview: {reason}")

    def start_routing(self) -> None:
        self.routing_lease_token = 1
        self.head.state = HeadStateValues.ROUTING
        self.head.torque_enabled = True
        for servo in self.head.servos:
            servo.goal_tick = 0
            servo.present_tick = 0
        if self.head_callback:
            self.head_callback(self.head)
        if self.result_callback:
            self.result_callback(
                "Preview: routing hold active at 0 degrees (no hardware connected)")

    def stop_routing(self) -> None:
        self.routing_lease_token = 0
        self.head.state = HeadStateValues.HOMING_REQUIRED
        self.head.torque_enabled = False
        if self.head_callback:
            self.head_callback(self.head)
        if self.result_callback:
            self.result_callback("Preview: routing hold disabled")

    def mark_contact(self, active: bool, region: str) -> None:
        if active:
            self.contact_event_id = f"preview_{uuid.uuid4().hex}"
            notice = f"Preview: contact started in {region or 'unspecified region'}"
        elif self.contact_event_id:
            self.contact_event_id = ""
            notice = "Preview: contact ended"
        else:
            notice = "No preview contact is active"
        self._refresh()
        if self.result_callback:
            self.result_callback(notice)


def preview_main() -> None:
    """Display the real operator window with a local demonstration backend."""
    app = QApplication(sys.argv)
    node = PreviewNode()
    window = CollectionWindow(node)  # type: ignore[arg-type]
    window.operator.setText("preview_operator")
    window.skin.setText("demo_skin_v1")
    window.fixture.setText("bench_fixture")
    window.update_head(node.head)
    window.update_session(node.session)
    window.set_notice("PREVIEW MODE — no ROS services or hardware are connected")
    window.show()
    app.exec_()


def main() -> None:
    if "--preview" in sys.argv:
        sys.argv.remove("--preview")
        preview_main()
        return
    if rclpy is None:
        raise RuntimeError(
            "ROS 2 Python packages are unavailable; run with --preview to "
            "inspect the GUI without a backend") from ROS_IMPORT_ERROR
    rclpy.init()
    node = CollectionGuiNode()
    app = QApplication(sys.argv)
    window = CollectionWindow(node)
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    timer.start(10)
    window.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
