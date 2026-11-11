"""Qt panel for jogging individual joints through the ROS bridge.

The panel streams a complete normalized target for every active joint while
the firmware is ENABLED under its lease. It seeds that target from measured
`present_normalized` values and refuses Enable, and never streams, while those
are unavailable. There are two ways to ENABLED: Home then Enable, or Enable
here, which skips homing and maps q to the profile's raw min_tick..max_tick.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from typing import Any, Callable, Dict, List, Optional

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    from head_msgs.action import HomeHead
    from head_msgs.msg import HeadState, JointTargets
    from head_msgs.srv import (AcquireControl, ClearFault, Disable, Enable,
                               ReleaseControl, RenewControl)
    ROS_IMPORT_ERROR: Optional[ImportError] = None
except ImportError as error:  # Allows the backend-free GUI preview.
    rclpy = None  # type: ignore[assignment]
    ActionClient = None  # type: ignore[assignment,misc]
    Node = object  # type: ignore[misc,assignment]
    QoSProfile = ReliabilityPolicy = None  # type: ignore[assignment]
    HomeHead = HeadState = JointTargets = None  # type: ignore[assignment]
    AcquireControl = ClearFault = Disable = Enable = None  # type: ignore[assignment]
    ReleaseControl = RenewControl = None  # type: ignore[assignment]
    ROS_IMPORT_ERROR = error

try:
    from head_ros.jog_core import (JOG_STEPS, LEASE_RENEW_PERIOD_S,
                                   TARGET_PUBLISH_PERIOD_S, HeadLifecycle,
                                   JogPoseLibrary, JogSession,
                                   NormalizedPositionUnavailable, active_servos,
                                   lifecycle_name, present_positions_normalized)
except ImportError:  # Run directly as a script for --preview.
    from jog_core import (JOG_STEPS, LEASE_RENEW_PERIOD_S,  # type: ignore[no-redef]
                          TARGET_PUBLISH_PERIOD_S, HeadLifecycle, JogPoseLibrary,
                          JogSession, NormalizedPositionUnavailable, active_servos,
                          lifecycle_name, present_positions_normalized)

try:
    from python_qt_binding.QtCore import QTimer, Qt
    from python_qt_binding.QtGui import QFont, QKeySequence
    from python_qt_binding.QtWidgets import (
        QApplication, QDialog, QDialogButtonBox, QDoubleSpinBox, QFormLayout,
        QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget,
        QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QScrollArea,
        QShortcut, QSlider, QVBoxLayout, QWidget,
    )
except ImportError:
    try:
        from PyQt5.QtCore import QTimer, Qt
        from PyQt5.QtGui import QFont, QKeySequence
        from PyQt5.QtWidgets import (
            QApplication, QDialog, QDialogButtonBox, QDoubleSpinBox, QFormLayout,
            QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget,
            QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QScrollArea,
            QShortcut, QSlider, QVBoxLayout, QWidget,
        )
    except ImportError as error:  # pragma: no cover - environment dependency
        raise RuntimeError(
            "install ROS 2 python_qt_binding or PyQt5 to display the GUI") from error


DEFAULT_POSE_PATH = Path.home() / ".ros" / "head_jog_poses.json"
# The slider works in thousandths of the calibrated range, matching the
# smallest jog step; the spin box shows the same resolution.
SLIDER_STEPS = 1000


class JogGuiNode(Node):
    """ROS side of the panel. All jog state lives in `self.session`."""

    def __init__(self) -> None:
        super().__init__("head_jog_gui")
        self.session = JogSession()
        self.head_callback: Optional[Callable[[Any], None]] = None
        self.notice_callback: Optional[Callable[[str], None]] = None
        self.renew_future: Any = None
        self.home_pending = False
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        # Match the bridge's command subscription: the latest sample only.
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(HeadState, "/head/state", self._on_head, sensor_qos)
        self.target_publisher = self.create_publisher(
            JointTargets, "/head/joint_targets", command_qos)
        self.acquire_client = self.create_client(AcquireControl, "/head/acquire_control")
        self.release_client = self.create_client(ReleaseControl, "/head/release_control")
        self.enable_client = self.create_client(Enable, "/head/enable")
        self.enable_at_present_client = self.create_client(
            Enable, "/head/enable_at_present_pose")
        self.disable_client = self.create_client(Disable, "/head/disable")
        self.renew_client = self.create_client(RenewControl, "/head/renew_control")
        self.clear_fault_client = self.create_client(ClearFault, "/head/clear_fault")
        self.home_client = ActionClient(self, HomeHead, "/head/home")
        self.create_timer(TARGET_PUBLISH_PERIOD_S, self._publish_tick)
        self.create_timer(LEASE_RENEW_PERIOD_S, self._renew_tick)

    @property
    def head(self) -> Any:
        return self.session.head

    def _notify(self, text: str) -> None:
        if self.notice_callback:
            self.notice_callback(text)

    # --- state feed and streaming ------------------------------------------
    def _on_head(self, message: HeadState) -> None:
        observation = self.session.observe_head(message, time.monotonic())
        for notice in observation.notices:
            self._notify(notice)
        if observation.publish_now:
            # The firmware faults if no target arrives within about 150 ms of
            # entering ENABLED, so do not wait for the next timer tick.
            self._publish_targets()
        if observation.request_disable:
            self.disable()
        if self.head_callback:
            self.head_callback(message)

    def _publish_tick(self) -> None:
        notice = self.session.check_stale(time.monotonic())
        if notice:
            self._notify(notice)
            return
        if self.session.targets is not None:
            self._publish_targets()

    def _publish_targets(self) -> None:
        names, positions = self.session.targets.as_lists()
        message = JointTargets()
        message.header.stamp = self.get_clock().now().to_msg()
        message.lease_token = self.session.lease_token
        # 0 asks the bridge to assign the next monotonic sequence.
        message.sequence = 0
        message.mode = JointTargets.POSITION
        message.names = names
        message.position = positions
        message.velocity = []
        self.target_publisher.publish(message)

    # --- lease -------------------------------------------------------------
    def _renew_tick(self) -> None:
        if (not self.session.can_renew(time.monotonic()) or
                (self.renew_future is not None and not self.renew_future.done())):
            return
        if not self.renew_client.service_is_ready():
            self._notify("/head/renew_control is unavailable; the lease will expire")
            return
        token = self.session.lease_token
        self.renew_future = self.renew_client.call_async(RenewControl.Request(lease_token=token))
        self.renew_future.add_done_callback(lambda future: self._on_renewed(future, token))

    def _on_renewed(self, future: Any, token: int) -> None:
        if token != self.session.lease_token:
            return
        try:
            result = future.result()
            if result.renewed:
                return
            reason = result.reason
        except Exception as error:
            reason = str(error)
        self._notify(self.session.drop_lease(f"renewal failed ({reason})"))

    def acquire(self) -> None:
        if self.session.has_lease:
            return
        if self.session.head is None:
            self._notify("wait for /head/state before acquiring control")
            return
        if not self.acquire_client.service_is_ready():
            self._notify("/head/acquire_control is unavailable")
            return
        future = self.acquire_client.call_async(AcquireControl.Request(requester="jog_gui"))
        future.add_done_callback(self._on_acquired)

    def _on_acquired(self, future: Any) -> None:
        try:
            result = future.result()
            if not result.granted:
                raise RuntimeError(result.reason)
            self.session.grant_lease(result.lease_token)
            self._notify("control lease granted")
        except Exception as error:
            self._notify(f"acquire failed: {error}")

    def release(self) -> None:
        if not self.session.has_lease:
            return
        if self.head is not None and self.head.torque_enabled:
            self._notify("torque is on; Disable before releasing the lease")
            return
        token = self.session.lease_token
        self._notify(self.session.drop_lease("released by operator"))
        if self.release_client.service_is_ready():
            self.release_client.call_async(ReleaseControl.Request(lease_token=token))

    # --- lifecycle ---------------------------------------------------------
    def home(self) -> None:
        if not self.session.has_lease or self.home_pending:
            return
        if not self.home_client.server_is_ready():
            self._notify("/head/home action server is unavailable")
            return
        self.home_pending = True
        sent = self.home_client.send_goal_async(
            HomeHead.Goal(lease_token=self.session.lease_token))
        sent.add_done_callback(self._on_home_goal)
        self._notify("homing requested")

    def _on_home_goal(self, future: Any) -> None:
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError("home goal rejected")
            handle.get_result_async().add_done_callback(self._on_home_result)
        except Exception as error:
            self.home_pending = False
            self._notify(f"homing failed: {error}")

    def _on_home_result(self, future: Any) -> None:
        self.home_pending = False
        try:
            result = future.result().result
            self._notify("homing complete: READY, torque off" if result.success
                         else f"homing failed: {result.reason}")
        except Exception as error:
            self._notify(f"homing failed: {error}")

    def enable(self) -> None:
        blockers = self.session.enable_blockers(time.monotonic())
        if blockers:
            self._notify("Enable refused: " + "; ".join(blockers))
            return
        if not self.enable_client.service_is_ready():
            self._notify("/head/enable is unavailable")
            return
        future = self.enable_client.call_async(
            Enable.Request(lease_token=self.session.lease_token))
        future.add_done_callback(lambda done: self._report(done, "enabled", "Enable"))

    def enable_at_present_pose(self) -> None:
        blockers = self.session.enable_at_present_pose_blockers(time.monotonic())
        if blockers:
            self._notify("Enable here refused: " + "; ".join(blockers))
            return
        if not self.enable_at_present_client.service_is_ready():
            self._notify("/head/enable_at_present_pose is unavailable")
            return
        future = self.enable_at_present_client.call_async(
            Enable.Request(lease_token=self.session.lease_token))
        future.add_done_callback(lambda done: self._report(done, "enabled", "Enable here"))

    def disable(self) -> Any:
        """Request the verified whole-head torque shutdown.

        The current target keeps streaming, frozen, until the firmware leaves
        ENABLED; stopping it first would latch a watchdog fault instead of a
        clean disable. Without a lease or a reachable service, streaming stops
        so the firmware command watchdog removes torque instead.
        """
        if not self.session.has_lease:
            self.session.targets = None
            self._notify("no local lease: stopped streaming; the firmware watchdog "
                         "disables any torque this panel was commanding")
            return None
        if not self.disable_client.service_is_ready():
            self.session.targets = None
            self._notify("/head/disable is unavailable: stopped streaming so the "
                         "firmware command watchdog removes torque")
            return None
        self.session.freeze_motion()
        future = self.disable_client.call_async(
            Disable.Request(lease_token=self.session.lease_token))
        future.add_done_callback(lambda done: self._report(done, "disabled", "Disable"))
        return future

    def clear_fault(self) -> None:
        if not self.session.has_lease:
            self._notify("clear fault needs a control lease")
            return
        if not self.clear_fault_client.service_is_ready():
            self._notify("/head/clear_fault is unavailable")
            return
        future = self.clear_fault_client.call_async(
            ClearFault.Request(lease_token=self.session.lease_token))
        future.add_done_callback(lambda done: self._report(done, "cleared", "Clear fault"))

    def _report(self, future: Any, field_name: str, label: str) -> None:
        try:
            result = future.result()
            accepted = bool(getattr(result, field_name))
            self._notify(f"{label}: {'accepted' if accepted else 'rejected'} ({result.reason})")
        except Exception as error:
            self._notify(f"{label} failed: {error}")

    # --- motion ------------------------------------------------------------
    def can_jog(self) -> bool:
        return self.session.can_jog(time.monotonic())

    def set_target(self, name: str, value: float) -> Optional[float]:
        return self.session.set_target(name, value, time.monotonic())

    def step_target(self, name: str, delta: float) -> Optional[float]:
        return self.session.step_target(name, delta, time.monotonic())

    def recall(self, pose: Any) -> Any:
        return self.session.recall(pose, time.monotonic())


class JointRow(QGroupBox):
    """One joint: target slider, exact target entry, jog steps and telemetry."""

    def __init__(self, name: str, on_set: Callable[[str, float], Optional[float]],
                 on_step: Callable[[str, float], Optional[float]]) -> None:
        super().__init__(name)
        self.name = name
        self._on_set = on_set
        self._on_step = on_step
        layout = QGridLayout(self)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, SLIDER_STEPS)
        self.slider.valueChanged.connect(
            lambda value: self._request(value / SLIDER_STEPS))
        layout.addWidget(self.slider, 0, 0)

        self.target_spin = QDoubleSpinBox()
        self.target_spin.setRange(0.0, 1.0)
        self.target_spin.setDecimals(3)
        self.target_spin.setSingleStep(1.0 / SLIDER_STEPS)
        self.target_spin.setFixedWidth(90)
        self.target_spin.setToolTip("normalized target q, 0..1 over the calibrated range")
        # Qt emits every intermediate value while typing by default, so
        # replacing 0.300 with 0.75 would command 0 on the way. Apply only on
        # Enter, focus loss or the arrow buttons.
        self.target_spin.setKeyboardTracking(False)
        self.target_spin.valueChanged.connect(self._request)
        layout.addWidget(self.target_spin, 0, 1)

        jog_row = QHBoxLayout()
        self.jog_buttons: List[QPushButton] = []
        for step in JOG_STEPS:
            button = QPushButton(f"{step:+g}")
            button.setFixedWidth(64)
            button.clicked.connect(lambda _checked, delta=step: self._step(delta))
            jog_row.addWidget(button)
            self.jog_buttons.append(button)
        jog_row.addStretch(1)
        layout.addLayout(jog_row, 1, 0, 1, 2)

        self.telemetry_label = QLabel("waiting for telemetry")
        self.telemetry_label.setFont(QFont("monospace"))
        self.telemetry_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.telemetry_label, 2, 0, 1, 2)
        layout.setColumnStretch(0, 1)
        self.set_controls_enabled(False)

    def set_controls_enabled(self, enabled: bool) -> None:
        self.slider.setEnabled(enabled)
        self.target_spin.setEnabled(enabled)
        for button in self.jog_buttons:
            button.setEnabled(enabled)

    def _request(self, value: float) -> None:
        applied = self._on_set(self.name, value)
        if applied is not None:
            self.show_position(applied)

    def _step(self, delta: float) -> None:
        applied = self._on_step(self.name, delta)
        if applied is not None:
            self.show_position(applied)

    def show_position(self, value: float) -> None:
        """Display a target or measured q without re-emitting it."""
        if not self.slider.isSliderDown():
            self.slider.blockSignals(True)
            self.slider.setValue(round(value * SLIDER_STEPS))
            self.slider.blockSignals(False)
        if not self.target_spin.hasFocus():
            self.target_spin.blockSignals(True)
            self.target_spin.setValue(value)
            self.target_spin.blockSignals(False)

    def apply_servo(self, servo: Any) -> None:
        present = float(servo.present_normalized)
        present_text = f"{present:.3f}" if math.isfinite(present) else "  n/a"
        status = ("OK" if servo.online and not servo.hardware_error else
                  "OFFLINE" if not servo.online else f"HW ERROR {servo.hardware_error}")
        self.telemetry_label.setText(
            f"q {present_text}   tick {servo.present_tick:>8d} (goal {servo.goal_tick:>8d})   "
            f"{servo.current_ma:>5d} mA   {servo.voltage_mv / 1000:>5.2f} V   "
            f"{servo.temperature_c:>3d} °C   age {servo.feedback_age_ms:>4d} ms   {status}")
        self.telemetry_label.setStyleSheet(
            "" if status == "OK" else "color: #b00020; font-weight: bold;")


class CapturePoseDialog(QDialog):
    def __init__(self, existing: List[str], parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Capture pose")
        form = QFormLayout(self)
        self.name_edit = QLineEdit()
        self.note_edit = QLineEdit()
        form.addRow("Name", self.name_edit)
        form.addRow("Note", self.note_edit)
        if existing:
            form.addRow(QLabel("Existing: " + ", ".join(existing)))
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def values(self) -> tuple:
        return self.name_edit.text().strip(), self.note_edit.text().strip()


class JogWindow(QMainWindow):
    def __init__(self, node: Any, pose_path: Path) -> None:
        super().__init__()
        self.node = node
        self.rows: Dict[str, JointRow] = {}
        self.setWindowTitle("Project HEAD — Joint Jog")
        self.resize(1000, 820)
        node.head_callback = self.update_head
        node.notice_callback = self.log

        self.library = JogPoseLibrary(pose_path)
        try:
            self.library.load()
        except (OSError, ValueError) as error:
            QMessageBox.warning(self, "Pose library", str(error))

        root = QWidget()
        outer = QVBoxLayout(root)

        status_box = QGroupBox("Controller")
        status_layout = QGridLayout(status_box)
        self.head_state = QLabel("No /head/state")
        self.lease_state = QLabel("no lease")
        self.motion_state = QLabel("")
        self.motion_state.setWordWrap(True)
        status_layout.addWidget(QLabel("Firmware:"), 0, 0)
        status_layout.addWidget(self.head_state, 0, 1)
        status_layout.addWidget(QLabel("Lease:"), 1, 0)
        status_layout.addWidget(self.lease_state, 1, 1)
        status_layout.addWidget(QLabel("Motion:"), 2, 0)
        status_layout.addWidget(self.motion_state, 2, 1)
        status_layout.setColumnStretch(1, 1)
        outer.addWidget(status_box)

        buttons = QHBoxLayout()
        self.acquire_button = QPushButton("Acquire control")
        self.home_button = QPushButton("Home")
        self.enable_button = QPushButton("Enable")
        self.enable_here_button = QPushButton("Enable here (no homing)")
        self.enable_here_button.setToolTip(
            "Torque on where the head is now. q maps to the profile's raw "
            "min_tick..max_tick; refused if any servo is outside that range.")
        self.clear_fault_button = QPushButton("Clear fault")
        self.release_button = QPushButton("Release control")
        self.acquire_button.clicked.connect(self.node.acquire)
        self.home_button.clicked.connect(self._confirm_home)
        self.enable_button.clicked.connect(self._confirm_enable)
        self.enable_here_button.clicked.connect(self._confirm_enable_here)
        self.clear_fault_button.clicked.connect(self.node.clear_fault)
        self.release_button.clicked.connect(self.node.release)
        for button in (self.acquire_button, self.home_button, self.enable_button,
                       self.enable_here_button, self.clear_fault_button,
                       self.release_button):
            buttons.addWidget(button)
        outer.addLayout(buttons)

        self.disable_button = QPushButton("DISABLE   (Esc)")
        self.disable_button.setMinimumHeight(52)
        self.disable_button.setStyleSheet(
            "QPushButton { background-color: #c62828; color: white; font-size: 17px; "
            "font-weight: bold; }")
        self.disable_button.clicked.connect(self._disable)
        outer.addWidget(self.disable_button)

        self.joint_area = QScrollArea()
        self.joint_area.setWidgetResizable(True)
        self.joint_container = QWidget()
        self.joint_layout = QVBoxLayout(self.joint_container)
        self.joint_layout.addStretch(1)
        self.joint_area.setWidget(self.joint_container)
        outer.addWidget(self.joint_area, 1)

        pose_box = QGroupBox(f"Poses — {pose_path}")
        pose_layout = QHBoxLayout(pose_box)
        self.pose_list = QListWidget()
        self.pose_list.setMaximumHeight(120)
        pose_layout.addWidget(self.pose_list, 1)
        pose_buttons = QVBoxLayout()
        self.capture_button = QPushButton("Capture present…")
        self.recall_button = QPushButton("Go to pose")
        self.delete_button = QPushButton("Delete")
        self.capture_button.clicked.connect(self._capture_pose)
        self.recall_button.clicked.connect(self._recall_pose)
        self.delete_button.clicked.connect(self._delete_pose)
        for button in (self.capture_button, self.recall_button, self.delete_button):
            pose_buttons.addWidget(button)
        pose_buttons.addStretch(1)
        pose_layout.addLayout(pose_buttons)
        outer.addWidget(pose_box)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(300)
        self.log_view.setFixedHeight(110)
        self.log_view.setFont(QFont("monospace"))
        outer.addWidget(self.log_view)
        outer.addWidget(QLabel(
            "Software Disable is not an emergency stop. Keep physical actuator-power "
            "removal accessible."))
        self.setCentralWidget(root)

        self.disable_shortcut = QShortcut(QKeySequence(Qt.Key_Escape), self)
        self.disable_shortcut.activated.connect(self._disable)
        self._refresh_pose_list()
        self.update_buttons()

    # --- log and status ----------------------------------------------------
    def log(self, text: str) -> None:
        self.log_view.appendPlainText(time.strftime("%H:%M:%S  ") + text)

    def update_head(self, message: Any) -> None:
        self.head_state.setText(
            f"state={lifecycle_name(int(message.state))}  fault={message.fault}  "
            f"torque={'ON' if message.torque_enabled else 'off'}  "
            f"boot={message.boot_session_id:08x}  control={message.control_hz} Hz  "
            f"fan={message.fan_rpm} rpm  applied_sequence={message.applied_sequence}")
        servos = active_servos(message)
        names = [servo.name for servo in servos]
        if names != list(self.rows):
            self._rebuild_rows(names)
        targets = self.node.session.targets
        for servo in servos:
            row = self.rows[servo.name]
            row.apply_servo(servo)
            if targets is not None:
                row.show_position(targets.get(servo.name))
            elif math.isfinite(float(servo.present_normalized)):
                row.show_position(float(servo.present_normalized))
        self.update_buttons()

    def _rebuild_rows(self, names: List[str]) -> None:
        for row in self.rows.values():
            self.joint_layout.removeWidget(row)
            row.setParent(None)
            row.deleteLater()
        self.rows = {}
        for name in names:
            row = JointRow(name, self.node.set_target, self.node.step_target)
            self.joint_layout.insertWidget(self.joint_layout.count() - 1, row)
            self.rows[name] = row

    def update_buttons(self) -> None:
        session = self.node.session
        head = session.head
        state = int(head.state) if head is not None else -1
        has_lease = session.has_lease
        can_jog = self.node.can_jog()
        self.lease_state.setText(
            f"held (token {session.lease_token:08x}), renewed every "
            f"{LEASE_RENEW_PERIOD_S * 1000:.0f} ms" if has_lease else "no lease")
        self.acquire_button.setEnabled(head is not None and not has_lease)
        self.home_button.setEnabled(
            has_lease and not getattr(self.node, "home_pending", False) and
            state in (HeadLifecycle.HOMING_REQUIRED, HeadLifecycle.READY))
        self.enable_button.setEnabled(has_lease and state == HeadLifecycle.READY)
        self.enable_here_button.setEnabled(
            has_lease and state == HeadLifecycle.HOMING_REQUIRED and
            not (head is not None and head.torque_enabled))
        self.clear_fault_button.setEnabled(has_lease and state == HeadLifecycle.FAULT)
        self.release_button.setEnabled(has_lease and not (head and head.torque_enabled))
        for row in self.rows.values():
            row.set_controls_enabled(can_jog)
        self.recall_button.setEnabled(can_jog)
        if can_jog:
            self.motion_state.setText(
                f"streaming all {len(self.rows)} active joints every "
                f"{TARGET_PUBLISH_PERIOD_S * 1000:.0f} ms")
            self.motion_state.setStyleSheet("color: #1b5e20; font-weight: bold;")
        elif session.targets is not None:
            self.motion_state.setText("holding the last target while Disable completes")
            self.motion_state.setStyleSheet("color: #b35c00; font-weight: bold;")
        else:
            reason = "not ENABLED under this panel's lease"
            if head is not None:
                try:
                    present_positions_normalized(head)
                except NormalizedPositionUnavailable as error:
                    reason = (f"{error}. NaN means the bridge has no current position "
                              f"range (firmware without POSITION_RANGE, or not read yet).")
            self.motion_state.setText(f"locked — {reason}")
            self.motion_state.setStyleSheet("color: #555;")

    # --- operator actions --------------------------------------------------
    def _confirm_home(self) -> None:
        if not getattr(self.node, "preview_mode", False):
            answer = QMessageBox.question(
                self, "Home the head?",
                "Homing energises servos one at a time and pulls each tendon until "
                "sustained current indicates tension. Keep actuator-power removal "
                "accessible.", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return
        self.node.home()

    def _confirm_enable(self) -> None:
        blockers = self.node.session.enable_blockers(time.monotonic())
        if blockers:
            self.log("Enable refused: " + "; ".join(blockers))
            return
        if not getattr(self.node, "preview_mode", False):
            answer = QMessageBox.question(
                self, "Enable torque on every active servo?",
                "The panel will stream targets seeded from the measured positions. "
                "Esc requests Disable.", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return
        self.node.enable()

    def _confirm_enable_here(self) -> None:
        blockers = self.node.session.enable_at_present_pose_blockers(time.monotonic())
        if blockers:
            self.log("Enable here refused: " + "; ".join(blockers))
            return
        if not getattr(self.node, "preview_mode", False):
            answer = QMessageBox.question(
                self, "Enable torque here, without homing?",
                "Every active servo holds where it is now. q 0..1 is the profile's raw "
                "min_tick..max_tick, so targets can reach only that range: check the "
                "profile came from tuning. Esc requests Disable.",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return
        self.node.enable_at_present_pose()

    def _disable(self) -> None:
        self.log("DISABLE requested")
        self.node.disable()

    def _refresh_pose_list(self) -> None:
        self.pose_list.clear()
        for pose in self.library.poses:
            label = f"{pose.name}  ({len(pose.positions_normalized)} joints)"
            if pose.note:
                label += f"  — {pose.note}"
            self.pose_list.addItem(label)

    def _selected_pose(self) -> Any:
        index = self.pose_list.currentRow()
        if index < 0 or index >= len(self.library.poses):
            return None
        return self.library.poses[index]

    def _capture_pose(self) -> None:
        head = self.node.session.head
        if head is None:
            self.log("capture refused: no /head/state")
            return
        try:
            positions = present_positions_normalized(head)
        except NormalizedPositionUnavailable as error:
            self.log(f"capture refused: {error}")
            return
        dialog = CapturePoseDialog(self.library.names(), self)
        if dialog.exec_() != QDialog.Accepted:
            return
        name, note = dialog.values()
        if not name:
            self.log("capture refused: a pose needs a name")
            return
        try:
            self.library.store(name, positions, note)
        except (OSError, ValueError) as error:
            QMessageBox.critical(self, "Capture pose", str(error))
            return
        self._refresh_pose_list()
        self.log(f"pose '{name}' captured from {len(positions)} joints")

    def _recall_pose(self) -> None:
        pose = self._selected_pose()
        if pose is None:
            return
        outcome = self.node.recall(pose)
        if outcome is None:
            self.log(f"pose '{pose.name}' not recalled: jogging is locked")
            return
        applied, skipped = outcome
        self.log(f"pose '{pose.name}' recalled to {len(applied)} joints")
        if skipped:
            self.log("  not active on this controller: " + ", ".join(skipped))

    def _delete_pose(self) -> None:
        pose = self._selected_pose()
        if pose is None:
            return
        answer = QMessageBox.question(self, "Delete pose", f"Delete pose '{pose.name}'?",
                                      QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        try:
            self.library.delete(pose.name)
        except OSError as error:
            QMessageBox.critical(self, "Delete pose", str(error))
            return
        self._refresh_pose_list()
        self.log(f"pose '{pose.name}' deleted")

    def closeEvent(self, event: Any) -> None:
        if getattr(self.node, "preview_mode", False):
            event.accept()
            return
        head = self.node.session.head
        if self.node.session.has_lease and head is not None and head.torque_enabled:
            answer = QMessageBox.question(
                self, "Disable before closing?",
                "Torque is on under this panel's lease. Closing requests Disable. Keep "
                "this window open if the controller does not acknowledge it.",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                event.ignore()
                return
            future = self.node.disable()
            if future is not None:
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            try:
                acknowledged = bool(future is not None and future.done() and
                                    future.result().disabled)
            except Exception:
                acknowledged = False
            if not acknowledged:
                QMessageBox.critical(
                    self, "Disable not acknowledged",
                    "Keep this window open and remove actuator power if needed.")
                event.ignore()
                return
        # Torque is off, or was never this panel's. Release rather than leave
        # the lease to expire; the release is best effort either way.
        if self.node.session.has_lease and not (head is not None and head.torque_enabled):
            self.node.release()
            rclpy.spin_once(self.node, timeout_sec=0.2)
        event.accept()


class PreviewNode:
    """In-process stand-in for the bridge, used only to exercise the GUI.

    It simulates a firmware that reports position ranges, unless started with
    `--preview-without-normalized`, which reproduces a bridge connected to
    firmware without POSITION_RANGE.
    """

    preview_mode = True
    NAMES = ("jaw_left", "jaw_right", "brow_left", "brow_right")
    # Preview-only stand-in for the firmware rate limiter, in q per second.
    PREVIEW_RATE_PER_S = 0.4

    def __init__(self, reports_normalized: bool) -> None:
        self.session = JogSession()
        self.head_callback: Optional[Callable[[Any], None]] = None
        self.notice_callback: Optional[Callable[[str], None]] = None
        self.home_pending = False
        self.reports_normalized = reports_normalized
        self._homing_remaining_s = 0.0
        self._present = [0.42, 0.58, 0.30, 0.71]
        self.head = SimpleNamespace(
            state=HeadLifecycle.HOMING_REQUIRED, fault=0, torque_enabled=False,
            boot_session_id=0x5EED0001, control_hz=500, fan_rpm=1380,
            applied_sequence=0, active_servo_mask=0b1111, mcu_uptime_ms=0,
            servos=[SimpleNamespace(
                name=name, present_normalized=float("nan"), goal_normalized=float("nan"),
                present_tick=0, goal_tick=0, current_ma=0, voltage_mv=11800,
                temperature_c=31, feedback_age_ms=4, online=True, hardware_error=0)
                for name in self.NAMES])

    def _notify(self, text: str) -> None:
        if self.notice_callback:
            self.notice_callback("Preview: " + text)

    def tick(self, period_s: float) -> None:
        head = self.head
        head.mcu_uptime_ms += int(period_s * 1000)
        if head.state == HeadLifecycle.HOMING:
            self._homing_remaining_s -= period_s
            if self._homing_remaining_s <= 0.0:
                head.state = HeadLifecycle.READY
                self.home_pending = False
                self._notify("homing complete: READY, torque off")
        targets = self.session.targets
        for index, servo in enumerate(head.servos):
            if head.state == HeadLifecycle.ENABLED and targets is not None:
                error = targets.get(servo.name) - self._present[index]
                limit = self.PREVIEW_RATE_PER_S * period_s
                self._present[index] += max(-limit, min(limit, error))
            q = self._present[index]
            servo.present_normalized = q if self.reports_normalized else float("nan")
            servo.present_tick = 1000 + round(q * 6000)
            servo.goal_tick = (1000 + round(targets.get(servo.name) * 6000)
                               if targets is not None else servo.present_tick)
            servo.current_ma = 60 + round(abs(q - 0.5) * 120) if head.torque_enabled else 0
        observation = self.session.observe_head(head, time.monotonic())
        for notice in observation.notices:
            self._notify(notice)
        if observation.request_disable:
            self.disable()
        if self.head_callback:
            self.head_callback(head)

    def acquire(self) -> None:
        self.session.grant_lease(0x1234ABCD)
        self._notify("control lease granted")

    def release(self) -> None:
        self._notify(self.session.drop_lease("released by operator"))

    def home(self) -> None:
        self.home_pending = True
        self.head.state = HeadLifecycle.HOMING
        self._homing_remaining_s = 2.0
        self._notify("homing (2 s simulated)")

    def enable(self) -> None:
        blockers = self.session.enable_blockers(time.monotonic())
        if blockers:
            self._notify("Enable refused: " + "; ".join(blockers))
            return
        self.head.state = HeadLifecycle.ENABLED
        self.head.torque_enabled = True

    def enable_at_present_pose(self) -> None:
        blockers = self.session.enable_at_present_pose_blockers(time.monotonic())
        if blockers:
            self._notify("Enable here refused: " + "; ".join(blockers))
            return
        self.head.state = HeadLifecycle.ENABLED
        self.head.torque_enabled = True

    def disable(self) -> None:
        self.session.freeze_motion()
        self.head.state = HeadLifecycle.HOMING_REQUIRED
        self.head.torque_enabled = False
        self._notify("Disable accepted; torque off")

    def clear_fault(self) -> None:
        self.head.fault = 0
        self.head.state = HeadLifecycle.HOMING_REQUIRED

    def can_jog(self) -> bool:
        return self.session.can_jog(time.monotonic())

    def set_target(self, name: str, value: float) -> Optional[float]:
        return self.session.set_target(name, value, time.monotonic())

    def step_target(self, name: str, delta: float) -> Optional[float]:
        return self.session.step_target(name, delta, time.monotonic())

    def recall(self, pose: Any) -> Any:
        return self.session.recall(pose, time.monotonic())


def parse_arguments(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--poses", type=Path, default=DEFAULT_POSE_PATH,
                        help=f"jog pose file (default {DEFAULT_POSE_PATH})")
    parser.add_argument("--preview", action="store_true",
                        help="run against an in-process simulated controller")
    parser.add_argument("--preview-without-normalized", action="store_true",
                        help="preview a bridge whose firmware has no POSITION_RANGE")
    # ROS launch tooling appends --ros-args; rclpy consumes those itself.
    arguments, _ = parser.parse_known_args(argv)
    return arguments


def preview_main(arguments: argparse.Namespace) -> None:
    app = QApplication(sys.argv)
    node = PreviewNode(reports_normalized=not arguments.preview_without_normalized)
    window = JogWindow(node, arguments.poses.expanduser())
    window.log("PREVIEW MODE — no ROS services or hardware are connected")
    period_s = 0.02
    timer = QTimer()
    timer.timeout.connect(lambda: node.tick(period_s))
    timer.start(int(period_s * 1000))
    window.show()
    app.exec_()


def main() -> None:
    arguments = parse_arguments(sys.argv[1:])
    if arguments.preview or arguments.preview_without_normalized:
        preview_main(arguments)
        return
    if rclpy is None:
        raise RuntimeError(
            "ROS 2 Python packages are unavailable; run with --preview to "
            "inspect the GUI without a backend") from ROS_IMPORT_ERROR
    rclpy.init()
    node = JogGuiNode()
    app = QApplication(sys.argv)
    window = JogWindow(node, arguments.poses.expanduser())
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    timer.start(5)
    window.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
