"""Headless ROS 2 controller for safe static-proprioception data sessions."""

from __future__ import annotations

import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
from typing import Any, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from head_msgs.action import HomeHead
from head_msgs.msg import (HeadState, ProprioceptionEvent,
                           ProprioceptionSessionState)
from head_msgs.srv import (AbortProprioceptionSession, AcquireControl, Disable,
                           PrepareProprioceptionSession,
                           RenewControl, SetProprioceptionRecording,
                           StartProprioception)

from .collection_core import (CollectionState, SessionPhase, SessionWriter,
                              new_session_id, resolve_data_root)


class BagRecorder:
    def __init__(self, output_dir: Path, topics: list[str], enabled: bool):
        self.output_dir = output_dir
        self.topics = topics
        self.enabled = enabled
        self.process: Optional[subprocess.Popen[bytes]] = None
        self.log_stream: Any = None

    def start(self) -> None:
        if not self.enabled:
            return
        ros2 = shutil.which("ros2")
        if ros2 is None:
            raise RuntimeError("ros2 executable is unavailable")
        self.log_stream = (self.output_dir / "rosbag.log").open("wb")
        command = [ros2, "bag", "record", "--storage", "mcap", "-o",
                   str(self.output_dir / "bag"), *self.topics]
        self.process = subprocess.Popen(
            command, stdout=self.log_stream, stderr=subprocess.STDOUT,
            start_new_session=True)

    def failed(self) -> bool:
        return self.process is not None and self.process.poll() is not None

    def stop(self) -> None:
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                try:
                    self.process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=2.0)
        if self.log_stream is not None and not self.log_stream.closed:
            self.log_stream.flush()
            os.fsync(self.log_stream.fileno())
            self.log_stream.close()


class ProprioceptionCollector(Node):
    def __init__(self) -> None:
        super().__init__("proprioception_collector")
        self.declare_parameter("data_root", "")
        self.declare_parameter("record_rosbag", True)
        self.declare_parameter("require_rosbag", True)
        self.declare_parameter("default_warmup_ms", 10000)
        self.declare_parameter("max_feedback_age_ms", 30)
        self.declare_parameter("topics", [
            "/head/state", "/head/diagnostics", "/proprioception/events",
            "/proprioception/session_state", "/fixture/contact_schedule",
            "/fixture/force",
        ])

        sensor_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state = CollectionState()
        self.writer: Optional[SessionWriter] = None
        self.bag: Optional[BagRecorder] = None
        self.session_id = ""
        self.output_path = ""
        self.lease_token = 0
        self.lease_boot_session_id: Optional[int] = None
        self.renew_pending = False
        self.last_head_received_monotonic_s = 0.0
        self.latest_head: Optional[HeadState] = None
        self.warmup_ms = int(self.get_parameter("default_warmup_ms").value)
        self.warmup_started_mcu: Optional[int] = None
        self.last_reason = ""
        self.pending_disable = False
        self.disable_requested_monotonic_s: Optional[float] = None
        self.finalizing = False

        self.session_pub = self.create_publisher(
            ProprioceptionSessionState, "/proprioception/session_state", 10)
        self.event_pub = self.create_publisher(
            ProprioceptionEvent, "/proprioception/events", 20)
        self.create_subscription(HeadState, "/head/state", self.on_head_state,
                                 sensor_qos)
        self.create_subscription(ProprioceptionEvent, "/proprioception/events",
                                 self.on_external_event, 20)

        self.acquire_client = self.create_client(
            AcquireControl, "/head/acquire_control")
        self.start_hold_client = self.create_client(
            StartProprioception, "/head/start_proprioception")
        self.disable_client = self.create_client(Disable, "/head/disable")
        self.renew_client = self.create_client(RenewControl, "/head/renew_control")
        self.home_client = ActionClient(self, HomeHead, "/head/home")

        self.create_service(PrepareProprioceptionSession,
                            "/proprioception/prepare_session", self.prepare)
        self.create_service(SetProprioceptionRecording,
                            "/proprioception/set_recording", self.set_recording)
        self.create_service(AbortProprioceptionSession,
                            "/proprioception/abort_session", self.abort)
        self.create_timer(0.1, self.tick)

    def _transition(self, phase: SessionPhase, status: str) -> None:
        self.state.transition(phase, status)
        self.get_logger().info(f"collection state={phase.name}: {status}")

    def prepare(self, request: PrepareProprioceptionSession.Request,
                response: PrepareProprioceptionSession.Response
                ) -> PrepareProprioceptionSession.Response:
        if self.state.phase in (SessionPhase.SAVED, SessionPhase.ERROR):
            self.state.transition(SessionPhase.IDLE, "idle")
        if self.state.phase != SessionPhase.IDLE:
            response.reason = f"collector is {self.state.phase.name.lower()}"
            return response
        if self.latest_head is None or time.monotonic() - self.last_head_received_monotonic_s > 0.3:
            response.reason = "no fresh /head/state feedback"
            return response
        if int(self.latest_head.active_servo_mask) == 0:
            response.reason = "firmware reports no active servos"
            return response
        if not self.acquire_client.service_is_ready():
            response.reason = "/head/acquire_control service is unavailable"
            return response
        self.output_path = ""
        self.last_reason = ""
        self.disable_requested_monotonic_s = None
        self.renew_pending = False
        self.lease_boot_session_id = int(self.latest_head.boot_session_id)
        self.session_id = new_session_id()
        self.warmup_ms = int(request.warmup_ms or
                             self.get_parameter("default_warmup_ms").value)
        metadata = {
            "operator_id": request.operator_id,
            "skin_installation_id": request.skin_installation_id,
            "pose_id": request.pose_id,
            "fixture_id": request.fixture_id,
            "contact_region": request.contact_region,
            "notes": request.notes,
            "warmup_ms": self.warmup_ms,
        }
        try:
            root = resolve_data_root(str(self.get_parameter("data_root").value))
            self.writer = SessionWriter(root, self.session_id, metadata)
            record_rosbag = bool(self.get_parameter("record_rosbag").value)
            self.bag = BagRecorder(
                self.writer.partial_dir,
                list(self.get_parameter("topics").value), record_rosbag)
            self.bag.start()
        except Exception as error:
            if self.writer is not None:
                self.output_path = str(self.writer.finalize("error", str(error)))
            self.writer = None
            self.bag = None
            response.reason = f"could not start session: {error}"
            return response
        self._transition(SessionPhase.ACQUIRING, "acquiring controller lease")
        future = self.acquire_client.call_async(
            AcquireControl.Request(requester=f"proprioception:{self.session_id}"))
        session_id = self.session_id
        future.add_done_callback(
            lambda result: self._on_acquired(result, session_id))
        response.accepted = True
        response.session_id = self.session_id
        response.reason = "session preparation started"
        return response

    def _on_acquired(self, future: Any, session_id: str) -> None:
        try:
            result = future.result()
        except Exception as error:
            if session_id == self.session_id:
                self._begin_abort(f"control acquisition failed: {error}")
            return
        if session_id != self.session_id:
            if result.granted:
                self.disable_client.call_async(
                    Disable.Request(lease_token=int(result.lease_token)))
            return
        if not result.granted:
            self._begin_abort(f"control acquisition rejected: {result.reason}")
            return
        self.lease_token = int(result.lease_token)
        self.lease_boot_session_id = int(self.latest_head.boot_session_id) if self.latest_head else None
        if self.state.phase != SessionPhase.ACQUIRING:
            self._request_disable()
            return
        if self.latest_head is None:
            self._begin_abort("head feedback disappeared during acquisition")
        elif self.latest_head.state == HeadState.HOMING_REQUIRED:
            self._transition(SessionPhase.HOMING, "homing head")
            if not self.home_client.wait_for_server(timeout_sec=1.0):
                self._begin_abort("home action server unavailable")
                return
            goal = HomeHead.Goal(lease_token=self.lease_token)
            sent = self.home_client.send_goal_async(goal)
            sent.add_done_callback(
                lambda result: self._on_home_goal(result, session_id))
        elif self.latest_head.state == HeadState.READY:
            self._request_hold()
        else:
            self._begin_abort(
                f"head must be HOMING_REQUIRED or READY, got state {self.latest_head.state}")

    def _on_home_goal(self, future: Any, session_id: str) -> None:
        if session_id != self.session_id or self.state.phase != SessionPhase.HOMING:
            return
        try:
            handle = future.result()
        except Exception as error:
            self._begin_abort(f"home goal failed: {error}")
            return
        if not handle.accepted:
            self._begin_abort("home goal rejected")
            return
        result = handle.get_result_async()
        result.add_done_callback(
            lambda response: self._on_home_result(response, session_id))

    def _on_home_result(self, future: Any, session_id: str) -> None:
        if session_id != self.session_id or self.state.phase != SessionPhase.HOMING:
            return
        try:
            result = future.result().result
        except Exception as error:
            self._begin_abort(f"home action failed: {error}")
            return
        if not result.success:
            self._begin_abort(f"homing failed: {result.reason}")
            return
        self._request_hold(session_id)

    def _request_hold(self, session_id: Optional[str] = None) -> None:
        operation_session = session_id or self.session_id
        if operation_session != self.session_id:
            return
        if not self.start_hold_client.service_is_ready():
            self._begin_abort("/head/start_proprioception service is unavailable")
            return
        self._transition(SessionPhase.ENTERING_HOLD,
                         "requesting static proprioception hold")
        request = StartProprioception.Request(lease_token=self.lease_token)
        future = self.start_hold_client.call_async(request)
        future.add_done_callback(
            lambda result: self._on_hold_requested(result, operation_session))

    def _on_hold_requested(self, future: Any, session_id: str) -> None:
        if (session_id != self.session_id or
                self.state.phase != SessionPhase.ENTERING_HOLD):
            return
        try:
            result = future.result()
        except Exception as error:
            self._begin_abort(f"hold request failed: {error}")
            return
        if not result.started:
            self._begin_abort(f"hold request rejected: {result.reason}")

    @staticmethod
    def _stamp_ns(message: HeadState | ProprioceptionEvent) -> int:
        return (int(message.header.stamp.sec) * 1_000_000_000 +
                int(message.header.stamp.nanosec))

    def _head_record(self, message: HeadState) -> tuple[dict[str, Any], bool]:
        max_age = int(self.get_parameter("max_feedback_age_ms").value)
        servo_rows = []
        valid = message.fault == 0
        for index, servo in enumerate(message.servos):
            active = bool(message.active_servo_mask & (1 << index))
            servo_valid = bool(servo.online and servo.hardware_error == 0 and
                               servo.feedback_age_ms <= max_age)
            if active:
                valid = valid and servo_valid
            row = {
                "name": servo.name,
                "active": active,
                "goal_tick": int(servo.goal_tick),
                "present_tick": int(servo.present_tick),
                "present_velocity_raw": int(servo.present_velocity_raw),
                "current_ma": int(servo.current_ma),
                "voltage_mv": int(servo.voltage_mv),
                "temperature_c": int(servo.temperature_c),
                "moving_status": int(servo.moving_status),
                "hardware_error": int(servo.hardware_error),
                "online": bool(servo.online),
                "feedback_age_ms": int(servo.feedback_age_ms),
            }
            servo_rows.append(row)
        return ({
            "session_id": self.session_id,
            "host_stamp_ns": self._stamp_ns(message),
            "receipt_time_ns": int(self.get_clock().now().nanoseconds),
            "mcu_uptime_ms": int(message.mcu_uptime_ms),
            "boot_session_id": int(message.boot_session_id),
            "state": int(message.state),
            "fault": int(message.fault),
            "torque_enabled": bool(message.torque_enabled),
            "torque_state": int(message.torque_state),
            "shutdown_pending": bool(message.shutdown_pending),
            "fan_stalled": bool(message.fan_stalled),
            "fan_rpm": int(message.fan_rpm),
            "applied_sequence": int(message.applied_sequence),
            "control_hz": int(message.control_hz),
            "active_servo_mask": int(message.active_servo_mask),
            "servos": servo_rows,
        }, valid)

    def on_head_state(self, message: HeadState) -> None:
        if (self.state.phase not in (SessionPhase.IDLE, SessionPhase.SAVED,
                                     SessionPhase.ERROR) and
                self.lease_boot_session_id is not None and
                int(message.boot_session_id) != self.lease_boot_session_id):
            self.latest_head = message
            self.last_head_received_monotonic_s = time.monotonic()
            stale_reason = "controller rebooted; collection session is stale"
            if self.state.phase == SessionPhase.STOPPING:
                self.last_reason = stale_reason
                self._transition(SessionPhase.ABORTING, stale_reason)
                self.renew_pending = False
                self._request_disable()
            else:
                self._begin_abort(stale_reason)
            return
        self.latest_head = message
        self.last_head_received_monotonic_s = time.monotonic()
        if self.writer is not None:
            record, valid = self._head_record(message)
            self.writer.append_telemetry(record, valid)
        if self.state.phase in (SessionPhase.IDLE, SessionPhase.SAVED,
                                SessionPhase.ERROR):
            return
        if message.state == HeadState.FAULT:
            if self.state.phase == SessionPhase.STOPPING:
                self.last_reason = f"firmware fault {message.fault} during stop"
                self._transition(SessionPhase.ABORTING, self.last_reason)
            elif self.state.phase != SessionPhase.ABORTING:
                self._begin_abort(f"firmware fault {message.fault}")
            if (self.state.phase == SessionPhase.ABORTING and
                    message.torque_state == 1 and not message.torque_enabled and not message.shutdown_pending):
                self._finish_session("aborted", self.last_reason)
            return
        if (self.state.phase == SessionPhase.ENTERING_HOLD and
                message.state == HeadState.PROPRIOCEPTION_HOLD):
            self.warmup_started_mcu = int(message.mcu_uptime_ms)
            if self.writer is not None:
                self.writer.update_manifest(
                    boot_session_id=int(message.boot_session_id),
                    hold_ready_mcu_ms=int(message.mcu_uptime_ms),
                    servo_order=[servo.name for index, servo in enumerate(message.servos)
                                 if message.active_servo_mask & (1 << index)],
                    active_servo_mask=int(message.active_servo_mask))
            self._transition(SessionPhase.WARMING_UP,
                             "holding still while baseline settles")
        elif self.state.phase == SessionPhase.WARMING_UP:
            if message.state != HeadState.PROPRIOCEPTION_HOLD:
                self._begin_abort("firmware left proprioception hold during warm-up")
            elif (self.warmup_started_mcu is not None and
                  (int(message.mcu_uptime_ms) - self.warmup_started_mcu) >=
                  self.warmup_ms):
                self._transition(SessionPhase.ARMED,
                                 "ready; press Space to start recording")
                if self.writer is not None:
                    self.writer.update_manifest(
                        armed_mcu_ms=int(message.mcu_uptime_ms))
        elif self.state.phase in (SessionPhase.ARMED, SessionPhase.RECORDING):
            if message.state != HeadState.PROPRIOCEPTION_HOLD:
                self._begin_abort("firmware left proprioception hold during collection")
        elif self.state.phase in (SessionPhase.STOPPING, SessionPhase.ABORTING):
            if message.torque_state == 1 and not message.torque_enabled and not message.shutdown_pending:
                self._finish_session(
                    "complete" if self.state.phase == SessionPhase.STOPPING else "aborted",
                    self.last_reason)

    def _event_record(self, message: ProprioceptionEvent) -> dict[str, Any]:
        return {
            "session_id": message.session_id or self.session_id,
            "host_stamp_ns": self._stamp_ns(message),
            "kind": int(message.kind),
            "event_id": message.event_id,
            "region": message.region,
            "source": message.source,
            "target_force_n": None if math.isnan(message.target_force_n) else float(message.target_force_n),
            "measured_force_n": None if math.isnan(message.measured_force_n) else float(message.measured_force_n),
            "indentation_mm": None if math.isnan(message.indentation_mm) else float(message.indentation_mm),
            "mcu_uptime_ms": int(message.mcu_uptime_ms),
            "notes": message.notes,
        }

    def on_external_event(self, message: ProprioceptionEvent) -> None:
        if (self.writer is not None and message.source != "head_collection" and
                (not message.session_id or message.session_id == self.session_id)):
            self.writer.append_event(self._event_record(message))

    def _publish_event(self, kind: int, notes: str = "") -> None:
        message = ProprioceptionEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.kind = kind
        message.session_id = self.session_id
        message.event_id = f"{self.session_id}:{kind}:{message.header.stamp.nanosec}"
        message.region = ""
        message.source = "head_collection"
        message.target_force_n = float("nan")
        message.measured_force_n = float("nan")
        message.indentation_mm = float("nan")
        message.mcu_uptime_ms = (int(self.latest_head.mcu_uptime_ms)
                                 if self.latest_head is not None else 0)
        message.notes = notes
        if self.writer is not None:
            self.writer.append_event(self._event_record(message))
        self.event_pub.publish(message)

    def set_recording(self, request: SetProprioceptionRecording.Request,
                      response: SetProprioceptionRecording.Response
                      ) -> SetProprioceptionRecording.Response:
        if request.recording:
            if not self.state.can_start_recording:
                response.reason = f"recording cannot start while {self.state.phase.name.lower()}"
            else:
                self._publish_event(ProprioceptionEvent.RECORDING_START)
                self._transition(SessionPhase.RECORDING, "recording contact trials")
                if self.writer is not None and self.latest_head is not None:
                    self.writer.update_manifest(
                        recording_start_mcu_ms=int(self.latest_head.mcu_uptime_ms))
                response.accepted = True
                response.reason = "recording started"
        else:
            if not self.state.can_stop_recording:
                response.reason = f"recording cannot stop while {self.state.phase.name.lower()}"
            else:
                self._publish_event(ProprioceptionEvent.RECORDING_END)
                self._transition(SessionPhase.STOPPING,
                                 "disabling torque and finalizing session")
                self.last_reason = "normal stop"
                self._request_disable()
                response.accepted = True
                response.reason = "recording stopped; finalization in progress"
        response.state = self.state.phase.name
        response.output_path = self.output_path
        return response

    def abort(self, request: AbortProprioceptionSession.Request,
              response: AbortProprioceptionSession.Response
              ) -> AbortProprioceptionSession.Response:
        if self.state.phase in (SessionPhase.IDLE, SessionPhase.SAVED,
                                SessionPhase.ERROR):
            response.status = f"nothing active ({self.state.phase.name.lower()})"
            response.output_path = self.output_path
            return response
        self._begin_abort(request.reason or "operator abort")
        response.accepted = True
        response.status = "abort requested; torque disable and save in progress"
        response.output_path = self.output_path
        return response

    def _begin_abort(self, reason: str) -> None:
        if self.state.phase in (SessionPhase.ABORTING, SessionPhase.STOPPING,
                                SessionPhase.SAVED, SessionPhase.ERROR,
                                SessionPhase.IDLE):
            return
        self.last_reason = reason
        self._publish_event(ProprioceptionEvent.ABORT, reason)
        self._transition(SessionPhase.ABORTING, reason)
        self.renew_pending = False
        self._request_disable()

    def _request_disable(self) -> None:
        if self.pending_disable or self.lease_token == 0:
            if self.lease_token == 0:
                self._finish_session("aborted", self.last_reason or "no lease")
            return
        if not self.disable_client.service_is_ready():
            self.last_reason = "/head/disable service is unavailable"
            self.disable_requested_monotonic_s = time.monotonic()
            return
        self.pending_disable = True
        self.disable_requested_monotonic_s = time.monotonic()
        future = self.disable_client.call_async(
            Disable.Request(lease_token=self.lease_token))
        session_id = self.session_id
        future.add_done_callback(
            lambda result: self._on_disabled(result, session_id))

    def _renewal_is_active(self) -> bool:
        return (self.lease_token != 0 and not self.renew_pending and
                self.state.phase in (SessionPhase.ACQUIRING, SessionPhase.HOMING,
                                     SessionPhase.ENTERING_HOLD, SessionPhase.WARMING_UP,
                                     SessionPhase.ARMED, SessionPhase.RECORDING) and
                self.latest_head is not None and
                self.lease_boot_session_id == int(self.latest_head.boot_session_id) and
                time.monotonic() - self.last_head_received_monotonic_s < 0.3)

    def _renew_control_lease(self) -> None:
        """The collector owns renewal while its session is live.

        Renewal is deliberately disabled as soon as stop or abort begins, so
        a dead or failed collector cannot keep a firmware hold alive.
        """
        if not self._renewal_is_active() or not self.renew_client.service_is_ready():
            return
        self.renew_pending = True
        session_id = self.session_id
        token = self.lease_token
        future = self.renew_client.call_async(RenewControl.Request(lease_token=token))
        future.add_done_callback(lambda result: self._on_renewed(result, session_id, token))

    def _on_renewed(self, future: Any, session_id: str, token: int) -> None:
        self.renew_pending = False
        if (session_id != self.session_id or token != self.lease_token or
                self.state.phase in (SessionPhase.STOPPING, SessionPhase.ABORTING,
                                     SessionPhase.SAVED, SessionPhase.ERROR)):
            return
        try:
            result = future.result()
        except Exception as error:
            self._begin_abort(f"control lease renewal failed: {error}")
            return
        if not result.renewed:
            self._begin_abort(f"control lease renewal rejected: {result.reason}")

    def _on_disabled(self, future: Any, session_id: str) -> None:
        if session_id != self.session_id:
            return
        self.pending_disable = False
        try:
            result = future.result()
        except Exception as error:
            self.last_reason = f"disable request failed: {error}"
            return
        if not result.disabled:
            self.last_reason = f"disable rejected: {result.reason}"

    def _finish_session(self, status: str, reason: str) -> None:
        if self.finalizing or self.writer is None:
            return
        self.finalizing = True
        try:
            if self.bag is not None:
                self.bag.stop()
            validation = {
                "valid": status == "complete" and
                         self.writer.samples_received > self.writer.samples_invalid,
                "samples_received": self.writer.samples_received,
                "samples_invalid": self.writer.samples_invalid,
                "firmware_state_at_finish": (
                    int(self.latest_head.state) if self.latest_head is not None else None),
                "torque_enabled_at_finish": (
                    bool(self.latest_head.torque_enabled) if self.latest_head is not None else None),
            }
            self.output_path = str(self.writer.finalize(status, reason, validation))
            self._transition(SessionPhase.SAVED,
                             f"{status} session saved to {self.output_path}")
        except Exception as error:
            self.get_logger().error(f"session finalization failed: {error}")
            self._transition(SessionPhase.ERROR, f"finalization failed: {error}")
        finally:
            self.finalizing = False
            self.writer = None
            self.bag = None
            self.lease_token = 0
            self.lease_boot_session_id = None
            self.renew_pending = False
            self.pending_disable = False
            self.disable_requested_monotonic_s = None

    def tick(self) -> None:
        if (self.lease_token and self.state.phase in
                (SessionPhase.HOMING, SessionPhase.ENTERING_HOLD, SessionPhase.WARMING_UP,
                 SessionPhase.ARMED, SessionPhase.RECORDING) and
                time.monotonic() - self.last_head_received_monotonic_s > 0.5):
            self._begin_abort("controller feedback stream is stale")
        self._renew_control_lease()
        if (self.bag is not None and self.bag.failed() and
                self.state.phase not in (SessionPhase.STOPPING,
                                         SessionPhase.ABORTING,
                                         SessionPhase.SAVED,
                                         SessionPhase.ERROR)):
            if bool(self.get_parameter("require_rosbag").value):
                self._begin_abort("rosbag recorder exited unexpectedly")
        if (self.state.phase in (SessionPhase.STOPPING, SessionPhase.ABORTING) and
                self.disable_requested_monotonic_s is not None and
                time.monotonic() - self.disable_requested_monotonic_s >= 10.0):
            self.last_reason = (
                f"{self.last_reason}; " if self.last_reason else "") + (
                "timed out waiting for confirmed torque disable; "
                "firmware lease watchdog remains the safety fallback")
            if self.state.phase == SessionPhase.STOPPING:
                self._transition(SessionPhase.ABORTING, self.last_reason)
            self._finish_session("aborted", self.last_reason)
        message = ProprioceptionSessionState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.state = int(self.state.phase)
        message.session_id = self.session_id
        message.status = self.state.status
        message.output_path = self.output_path
        message.can_start_recording = self.state.can_start_recording
        message.can_stop_recording = self.state.can_stop_recording
        message.samples_received = self.writer.samples_received if self.writer else 0
        message.samples_invalid = self.writer.samples_invalid if self.writer else 0
        if (self.state.phase == SessionPhase.WARMING_UP and
                self.latest_head is not None and self.warmup_started_mcu is not None):
            elapsed = int(self.latest_head.mcu_uptime_ms) - self.warmup_started_mcu
            message.warmup_remaining_ms = max(0, self.warmup_ms - elapsed)
        self.session_pub.publish(message)

    def destroy_node(self) -> bool:
        if self.writer is not None:
            try:
                self._publish_event(ProprioceptionEvent.ABORT,
                                    "collector process shutting down")
                if self.lease_token and self.disable_client.service_is_ready():
                    future = self.disable_client.call_async(
                        Disable.Request(lease_token=self.lease_token))
                    rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            except Exception as error:
                self.get_logger().error(f"shutdown disable request failed: {error}")
            try:
                if self.bag is not None:
                    self.bag.stop()
            except Exception as error:
                self.get_logger().error(f"shutdown rosbag stop failed: {error}")
            try:
                self.output_path = str(self.writer.finalize(
                    "aborted", "collector process shutting down"))
            except Exception as error:
                self.get_logger().error(f"shutdown finalization failed: {error}")
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ProprioceptionCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
