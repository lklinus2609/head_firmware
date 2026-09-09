"""ROS 2 Jazzy bridge for the Teensy USB CDC control protocol."""

from __future__ import annotations

import queue
import struct
import time
import zlib
import math
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Header

from head_msgs.action import HomeHead, MaintenanceCalibrate
from head_msgs.msg import HeadDiagnostics, HeadState, JointTargets, ServoState
from head_msgs.srv import (AcquireControl, ClearFault, Disable, Enable,
                           ConfirmCalibration, GetConfiguration, ReleaseControl,
                           RenewControl, SetFanOverride, StartProprioception,
                           StartRouting)
from .serial_transport import (SerialTransport, TransportDisconnected,
                               TransportError, TransportTimeout)

try:
    import serial
except ImportError as error:  # pragma: no cover - reported at launch
    raise RuntimeError("Install pyserial: sudo apt install python3-serial") from error


SOF = 0xA5
VERSION = 2
MAX_PAYLOAD = 512
MSG_HELLO = 1
MSG_HELLO_REPLY = 2
MSG_ACQUIRE = 3
MSG_RELEASE = 4
MSG_TARGETS = 5
MSG_HOME = 6
MSG_MAINTENANCE_CALIBRATE = 7
MSG_ENABLE = 8
MSG_DISABLE = 9
MSG_CLEAR_FAULT = 10
MSG_FAN_OVERRIDE = 11
MSG_STATE = 12
MSG_DIAGNOSTICS = 13
MSG_ACK = 14
MSG_NACK = 15
MSG_CALIBRATION_CONFIRM = 16
MSG_GET_CONFIGURATION_INFO = 17
MSG_CONFIGURATION_INFO = 18
MSG_RENEW_LEASE = 32
MSG_START_PROPRIOCEPTION = 33
MSG_START_ROUTING = 34
CAP_PROPRIOCEPTION_HOLD = 1 << 8
CAP_ROUTING_HOLD = 1 << 9
SERVO_COUNT = 20


def encode_frame(message_type: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds Teensy protocol limit")
    body = bytes((VERSION, message_type)) + struct.pack("<H", len(payload)) + payload
    return bytes((SOF,)) + body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.last_byte_monotonic_s = 0.0

    def recover(self) -> Optional[tuple[int, bytes]]:
        raw = bytes(self.buffer)
        self.buffer.clear()
        incomplete = b""
        for start in range(1, len(raw)):
            if raw[start] != SOF:
                continue
            candidate = raw[start:]
            if len(candidate) < 5:
                incomplete = candidate
                continue
            if candidate[1] != VERSION:
                continue
            length = struct.unpack_from("<H", candidate, 3)[0]
            if length > MAX_PAYLOAD:
                continue
            total = 9 + length
            if len(candidate) < total:
                incomplete = candidate
                continue
            body = candidate[1:total - 4]
            received = struct.unpack_from("<I", candidate, total - 4)[0]
            if zlib.crc32(body) & 0xFFFFFFFF == received:
                return candidate[2], candidate[5:total - 4]
        self.buffer.extend(incomplete)
        return None

    def push(self, byte: int) -> Optional[tuple[int, bytes]]:
        now = time.monotonic()
        if self.buffer and now - self.last_byte_monotonic_s > 0.05:
            self.buffer.clear()
        self.last_byte_monotonic_s = now
        if not self.buffer and byte != SOF:
            return None
        self.buffer.append(byte)
        if len(self.buffer) < 5:
            return None
        if self.buffer[1] != VERSION:
            return self.recover()
        length = struct.unpack_from("<H", self.buffer, 3)[0]
        total = 9 + length
        if length > MAX_PAYLOAD or len(self.buffer) > total:
            return self.recover()
        if len(self.buffer) != total:
            return None
        body = bytes(self.buffer[1:-4])
        received = struct.unpack_from("<I", self.buffer, total - 4)[0]
        message_type = self.buffer[2]
        payload = bytes(self.buffer[5:-4])
        if zlib.crc32(body) & 0xFFFFFFFF != received:
            return self.recover()
        self.buffer.clear()
        return message_type, payload


class HeadBridge(Node):
    def __init__(self) -> None:
        super().__init__("head_bridge")
        # CDC0 is the Zephyr diagnostic console. CDC1 is the dedicated,
        # framed head-control endpoint; keep protocol traffic off the console.
        self.declare_parameter("port", "/dev/ttyACM1")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("joint_names", [f"servo_{index}" for index in range(SERVO_COUNT)])
        # Twelve actuators (IDs 0..11) are installed today.  The value is a
        # launch-time expectation only; the controller's stored profile is
        # queried and becomes authoritative as soon as it responds.
        self.declare_parameter("active_servo_mask", 0x00000FFF)
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.active_servo_mask = int(self.get_parameter("active_servo_mask").value)
        if len(self.joint_names) != SERVO_COUNT or len(set(self.joint_names)) != SERVO_COUNT:
            raise ValueError("joint_names must contain 20 unique names")

        self.parser = FrameParser()
        self.callback_group = ReentrantCallbackGroup()
        self.transport_events = queue.Queue(maxsize=256)
        self.last_state_monotonic_s = 0.0
        self.lease_token = 0
        self.sequence = 0
        self.latest_state: Optional[HeadState] = None
        self.packet_errors = 0
        self.firmware_control_deadline_misses = 0
        self.firmware_control_max_period_us = 0
        self.firmware_protocol_errors = 0
        self.branch_diagnostics: dict[int, tuple[int, int, int]] = {}
        self.configuration_version = 0
        self.configuration_synced = False
        self.hello_validated = False
        self.controller_session_id: Optional[int] = None
        self.negotiation_generation = 0
        self.negotiation_in_progress = False

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state_publisher = self.create_publisher(HeadState, "/head/state", sensor_qos)
        self.diagnostic_publisher = self.create_publisher(HeadDiagnostics, "/head/diagnostics", sensor_qos)
        # Commands are a sampled stream. Retrying or queueing an old face pose
        # is unsafe; the firmware watchdog handles a missed latest sample.
        self.create_subscription(JointTargets, "/head/joint_targets", self.on_targets, command_qos)
        self.create_service(AcquireControl, "/head/acquire_control", self.acquire_control, callback_group=self.callback_group)
        self.create_service(ReleaseControl, "/head/release_control", self.release_control, callback_group=self.callback_group)
        self.create_service(Enable, "/head/enable", self.enable, callback_group=self.callback_group)
        self.create_service(StartProprioception, "/head/start_proprioception",
                            self.start_proprioception, callback_group=self.callback_group)
        self.create_service(StartRouting, "/head/start_routing",
                            self.start_routing, callback_group=self.callback_group)
        self.create_service(RenewControl, "/head/renew_control",
                            self.renew_control, callback_group=self.callback_group)
        self.create_service(Disable, "/head/disable", self.disable, callback_group=self.callback_group)
        self.create_service(ClearFault, "/head/clear_fault", self.clear_fault, callback_group=self.callback_group)
        self.create_service(SetFanOverride, "/head/set_fan_override", self.set_fan_override, callback_group=self.callback_group)
        self.create_service(ConfirmCalibration, "/head/confirm_calibration_servo",
                            self.confirm_calibration_servo, callback_group=self.callback_group)
        self.create_service(GetConfiguration, "/head/get_configuration", self.get_configuration, callback_group=self.callback_group)
        self.home_action = ActionServer(self, HomeHead, "/head/home", self.home_execute,
                                        goal_callback=self.lease_goal, cancel_callback=self.cancel_goal, callback_group=self.callback_group)
        self.calibration_action = ActionServer(self, MaintenanceCalibrate,
                                               "/head/maintenance_calibrate",
                                               self.calibration_execute,
                                               goal_callback=self.lease_goal,
                                               cancel_callback=self.cancel_goal, callback_group=self.callback_group)
        self.transport = SerialTransport(
            str(self.get_parameter("port").value),
            int(self.get_parameter("baud").value),
            encode_frame,
            self._on_transport_frame,
            self._on_transport_disconnect,
            lambda: Future(executor=self.executor),
            session_id_provider=lambda: self.controller_session_id,
        )
        self.transport.set_byte_handler(self._on_serial_byte)
        self.transport.start()
        self.create_timer(0.001, self._drain_transport_events)
        # Start negotiation from an executor callback so request futures are
        # attached to the Jazzy executor and wake it when the transport thread
        # completes them.
        self.negotiation_timer = self.create_timer(0.2, self._retry_negotiation)
        self.create_timer(1.0, self.publish_diagnostics)

    def destroy_node(self) -> bool:
        self.transport.close()
        return super().destroy_node()

    def send(self, message_type: int, payload: bytes = b"") -> None:
        if not self.transport.send(message_type, payload,
                                   coalesce_key="targets" if message_type == MSG_TARGETS else None):
            self.get_logger().warning("serial outbound queue is full; dropped stream frame")

    async def request(self, message_type: int, payload: bytes = b"", timeout: float = 1.0,
                      response_types: tuple[int, ...] = (MSG_ACK, MSG_NACK)) -> tuple[bool, bytes]:
        """Await one bounded transport transaction without blocking ROS."""
        generation = self.negotiation_generation
        try:
            result = await self.transport.request(message_type, payload, timeout,
                                                  response_types)
        except TransportError:
            return False, b""
        if generation != self.negotiation_generation:
            return False, b""
        return bool(result[0]), bytes(result[1])

    def _on_serial_byte(self, byte: int) -> None:
        frame = self.parser.push(byte)
        if frame is not None:
            self.transport.handle_frame(*frame)

    def _queue_transport_event(self, event) -> None:
        try:
            self.transport_events.put_nowait(event)
        except queue.Full:
            # Losing continuity fails closed. Only the executor mutates ROS state.
            while True:
                try:
                    self.transport_events.get_nowait()
                except queue.Empty:
                    break
            self.transport_events.put_nowait((None, "transport event overflow"))

    def _on_transport_frame(self, message_type: int, payload: bytes) -> None:
        self._queue_transport_event((message_type, payload))

    def _on_transport_disconnect(self, error: BaseException) -> None:
        self._queue_transport_event((None, str(error)))

    def _drain_transport_events(self) -> None:
        for _ in range(64):
            try:
                message_type, payload = self.transport_events.get_nowait()
            except queue.Empty:
                break
            if message_type is None:
                self._reset_controller_session(f"serial disconnected: {payload}")
            elif message_type == MSG_STATE:
                self.on_state(payload)
            elif message_type == MSG_DIAGNOSTICS:
                self.on_diagnostics(payload)
            # HELLO/configuration are applied only by their matched futures.

    def _reset_controller_session(self, reason: str) -> None:
        self.lease_token = 0
        self.sequence = 0
        self.latest_state = None
        self.configuration_version = 0
        self.configuration_synced = False
        self.hello_validated = False
        self.controller_session_id = None
        self.negotiation_generation += 1
        self.negotiation_in_progress = False
        self.last_state_monotonic_s = 0.0
        self.capabilities = 0
        self.transport.reset_pending(reason)

    def _retry_negotiation(self) -> None:
        if not self.configuration_synced and not self.negotiation_in_progress:
            self._begin_negotiation()

    def _begin_negotiation(self) -> None:
        if self.negotiation_in_progress or self.configuration_synced:
            return
        self.negotiation_in_progress = True
        generation = self.negotiation_generation
        future = self.transport.request(MSG_HELLO, timeout_s=1.0,
                                        response_types=(MSG_HELLO_REPLY,))

        def completed(result: Future) -> None:
            if generation != self.negotiation_generation:
                return
            try:
                ok, payload, _ = result.result()
                if not ok or len(payload) != 52:
                    raise TransportError("controller HELLO negotiation failed")
                self.on_hello_reply(payload)
                if self.controller_session_id is None or not self.capabilities:
                    raise TransportError("invalid controller identity")
                config_generation = self.negotiation_generation
                config_future = self.transport.request(
                    MSG_GET_CONFIGURATION_INFO, timeout_s=1.0,
                    response_types=(MSG_CONFIGURATION_INFO,))
                config_future.add_done_callback(
                    lambda config: self._on_configuration_negotiated(config, config_generation))
            except Exception as error:
                self.negotiation_in_progress = False
                self.get_logger().warning(f"controller negotiation pending: {error}")

        future.add_done_callback(completed)

    def _on_configuration_negotiated(self, future: Future, generation: int) -> None:
        if generation != self.negotiation_generation:
            return
        try:
            ok, payload, _ = future.result()
            if not ok:
                raise TransportError("configuration request rejected")
            self.on_configuration_info(payload)
            self.negotiation_in_progress = not self.configuration_synced
        except Exception as error:
            self.negotiation_in_progress = False
            self.get_logger().warning(f"controller configuration pending: {error}")

    def on_configuration_info(self, payload: bytes) -> None:
        if not self.hello_validated or self.controller_session_id is None:
            return
        if len(payload) != 12:
            self.packet_errors += 1
            return
        version, expected_count, _, _, _, active_mask = struct.unpack("<IBBBBI", payload)
        if active_mask == 0 or active_mask & ~0x000FFFFF or active_mask.bit_count() != expected_count:
            self.get_logger().error("controller returned an invalid active-servo inventory")
            self.packet_errors += 1
            return
        if active_mask != self.active_servo_mask:
            self.get_logger().warning(
                f"using controller active mask 0x{active_mask:05x}; "
                f"launch expectation was 0x{self.active_servo_mask:05x}")
        self.active_servo_mask = active_mask
        self.configuration_version = version
        self.configuration_synced = True

    def on_hello_reply(self, payload: bytes) -> None:
        if len(payload) != 52:
            self.packet_errors += 1
            return
        (_, hello_schema, protocol_version, fw_major, fw_minor, fw_patch,
         hardware_revision, schema_hash, capabilities, build_id, hardware_id,
         session_id, reset_cause, generation, control_hz, telemetry_hz,
         usb_vid, usb_pid) = struct.unpack("<IHHBBBBIQIIIIIHHHH", payload)
        if hello_schema != 1 or protocol_version != VERSION or not capabilities & 1 or session_id == 0:
            self.get_logger().error("controller protocol capabilities are incompatible")
            self.packet_errors += 1
            return
        if (self.controller_session_id is not None and
                self.controller_session_id != session_id):
            # A HELLO from a different boot invalidates every local lease and
            # cached profile.  The current HELLO is the new negotiation, so
            # preserve its identity after clearing the old session.
            self.lease_token = 0
            self.sequence = 0
            self.latest_state = None
            self.configuration_version = 0
            self.configuration_synced = False
            self.hello_validated = False
            self.negotiation_generation += 1
            self.transport.reset_pending("controller HELLO session changed")
        self.firmware_identity = (
            f"firmware={fw_major}.{fw_minor}.{fw_patch} build={build_id:08x} "
            f"hardware={hardware_id:08x}/r{hardware_revision} schema={schema_hash:08x} "
            f"session={session_id:08x} rates={control_hz}/{telemetry_hz}Hz "
            f"usb={usb_vid:04x}:{usb_pid:04x}")
        self.hello_validated = True
        self.capabilities = capabilities
        self.controller_session_id = session_id
        self.reset_cause = reset_cause
        self.calibration_generation = generation

    def on_targets(self, message: JointTargets) -> None:
        if not self.configuration_synced or self.lease_token == 0 or message.lease_token != self.lease_token or message.mode != JointTargets.POSITION:
            self.get_logger().warning("only POSITION mode is enabled in this firmware revision")
            return
        if len(message.names) != len(message.position) or len(message.velocity) != 0:
            self.get_logger().warning("rejected target with an invalid lease or name/value lengths")
            return
        expected_names = {self.joint_names[index] for index in range(SERVO_COUNT)
                          if self.active_servo_mask & (1 << index)}
        if set(message.names) != expected_names or len(message.names) != len(expected_names):
            self.get_logger().warning("each target must contain every active joint exactly once")
            return
        values = [0.5] * SERVO_COUNT
        try:
            for name, value in zip(message.names, message.position):
                numeric_value = float(value)
                if not math.isfinite(numeric_value):
                    raise ValueError(f"non-finite target for {name}")
                if numeric_value < 0.0 or numeric_value > 1.0:
                    raise ValueError(f"target outside normalized range for {name}")
                values[self.joint_names.index(name)] = numeric_value
        except ValueError as error:
            self.get_logger().warning(f"invalid joint target: {error}")
            return
        requested_sequence = int(message.sequence) & 0xFFFFFFFF
        if requested_sequence != 0:
            if self.sequence != 0:
                sequence_delta = (requested_sequence - self.sequence) & 0xFFFFFFFF
                if sequence_delta == 0 or sequence_delta >= 0x80000000:
                    self.get_logger().warning("rejected non-increasing target sequence")
                    return
            self.sequence = requested_sequence
        else:
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        payload = struct.pack("<IIIB3x20f20f", self.lease_token, self.sequence,
                              self.active_servo_mask, message.mode, *values,
                              *([0.0] * SERVO_COUNT))
        self.send(MSG_TARGETS, payload)

    async def acquire_control(self, request: AcquireControl.Request, response: AcquireControl.Response) -> AcquireControl.Response:
        if self.controller_session_id is None or not self.configuration_synced:
            response.granted = False
            response.lease_token = 0
            response.reason = "controller session/configuration is not synchronized"
            return response
        ok, payload = await self.request(MSG_ACQUIRE, request.requester.encode("utf-8")[:64])
        if ok and len(payload) >= 6:
            self.lease_token = struct.unpack_from("<I", payload, 2)[0]
            self.sequence = 0
        response.granted = ok and self.lease_token != 0
        response.lease_token = self.lease_token
        response.reason = "granted" if response.granted else "controller already leased or not responding"
        return response

    async def release_control(self, request: ReleaseControl.Request, response: ReleaseControl.Response) -> ReleaseControl.Response:
        try:
            ok, _ = await self.request(MSG_RELEASE, struct.pack("<I", request.lease_token))
        except TransportError:
            ok = False
        if ok and request.lease_token == self.lease_token:
            self.lease_token = 0
        response.released, response.reason = ok, "released" if ok else "lease release rejected"
        return response

    async def _lease_command(self, message_type: int, token: int) -> tuple[bool, str]:
        if token == 0 or token != self.lease_token:
            return False, "invalid local lease token"
        try:
            ok, _ = await self.request(message_type, struct.pack("<I", token))
        except TransportError as error:
            return False, str(error)
        return ok, "accepted" if ok else "firmware rejected command"

    async def enable(self, request: Enable.Request, response: Enable.Response) -> Enable.Response:
        response.enabled, response.reason = await self._lease_command(MSG_ENABLE, request.lease_token)
        return response

    async def start_proprioception(self, request: StartProprioception.Request,
                             response: StartProprioception.Response) -> StartProprioception.Response:
        if not getattr(self, "capabilities", 0) & CAP_PROPRIOCEPTION_HOLD:
            response.started, response.reason = False, "firmware does not advertise proprioception hold"
            return response
        response.started, response.reason = await self._lease_command(
            MSG_START_PROPRIOCEPTION, request.lease_token)
        return response

    async def start_routing(self, request: StartRouting.Request,
                      response: StartRouting.Response) -> StartRouting.Response:
        if not getattr(self, "capabilities", 0) & CAP_ROUTING_HOLD:
            response.started, response.reason = (
                False, "firmware does not advertise routing hold")
            return response
        response.started, response.reason = await self._lease_command(
            MSG_START_ROUTING, request.lease_token)
        return response

    async def renew_control(self, request: RenewControl.Request,
                      response: RenewControl.Response) -> RenewControl.Response:
        response.renewed, response.reason = await self._lease_command(
            MSG_RENEW_LEASE, request.lease_token)
        return response

    async def disable(self, request: Disable.Request, response: Disable.Response) -> Disable.Response:
        response.disabled, response.reason = await self._lease_command(MSG_DISABLE, request.lease_token)
        return response

    async def clear_fault(self, request: ClearFault.Request, response: ClearFault.Response) -> ClearFault.Response:
        response.cleared, response.reason = await self._lease_command(MSG_CLEAR_FAULT, request.lease_token)
        return response

    async def set_fan_override(self, request: SetFanOverride.Request,
                         response: SetFanOverride.Response) -> SetFanOverride.Response:
        if request.percent > 100:
            response.accepted, response.reason = False, "fan override must be 0..100"
            return response
        if request.lease_token != self.lease_token:
            response.accepted, response.reason = False, "invalid local lease token"
            return response
        response.accepted, _ = await self.request(MSG_FAN_OVERRIDE,
                                                  struct.pack("<IB", request.lease_token, request.percent))
        response.reason = "accepted" if response.accepted else "firmware rejected fan override"
        return response

    async def get_configuration(self, request: GetConfiguration.Request,
                          response: GetConfiguration.Response) -> GetConfiguration.Response:
        del request
        if not self.configuration_synced:
            try:
                ok, payload = await self.request(
                    MSG_GET_CONFIGURATION_INFO, timeout=1.0,
                    response_types=(MSG_CONFIGURATION_INFO,))
                if ok:
                    self.on_configuration_info(payload)
            except TransportError:
                pass
        response.calibration_version = self.configuration_version
        response.joint_names = self.joint_names
        response.status = (
            f"controller profile synchronized; active mask=0x{self.active_servo_mask:05x}"
            if self.configuration_synced else
            "controller profile not yet available; retry after the firmware finishes booting")
        return response

    async def confirm_calibration_servo(self, request: ConfirmCalibration.Request,
                                  response: ConfirmCalibration.Response) -> ConfirmCalibration.Response:
        if request.lease_token != self.lease_token or request.servo_index >= SERVO_COUNT:
            response.accepted, response.reason = False, "invalid lease token or servo index"
            return response
        response.accepted, _ = await self.request(MSG_CALIBRATION_CONFIRM,
                                                  struct.pack("<IB", request.lease_token,
                                                              request.servo_index))
        response.reason = "accepted" if response.accepted else "firmware is not awaiting this servo review"
        return response

    def lease_goal(self, goal_request) -> GoalResponse:
        return (GoalResponse.ACCEPT
                if goal_request.lease_token != 0 and
                goal_request.lease_token == self.lease_token
                else GoalResponse.REJECT)

    @staticmethod
    def cancel_goal(goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    async def _wait_for_ready(self, goal_handle, action_type, sent_type: int):
        initial_calibration_generation = getattr(self, "calibration_generation", 0)
        ok, _ = await self.request(sent_type, struct.pack("<I", goal_handle.request.lease_token))
        result = action_type.Result()
        if not ok:
            result.success, result.reason = False, "firmware rejected request"
            goal_handle.abort()
            return result
        deadline_monotonic_s = time.monotonic() + 120.0
        observed_running = False
        action_generation = self.negotiation_generation
        poll_tick = Future(executor=self.executor)

        def wake_poll() -> None:
            if not poll_tick.done():
                poll_tick.set_result(None)

        poll_timer = self.create_timer(0.05, wake_poll)
        try:
            while True:
                state = self.latest_state
                if action_generation != self.negotiation_generation:
                    result.success, result.reason = False, "controller session changed"
                    goal_handle.abort()
                    return result
                if state and state.state in (HeadState.HOMING, HeadState.MAINTENANCE_CALIBRATION):
                    observed_running = True
                feedback = action_type.Feedback()
                # Protocol v2 does not expose the servo currently being homed.
                feedback.active_servo = 255
                feedback.phase = "waiting for firmware state (active servo unavailable)"
                goal_handle.publish_feedback(feedback)
                # Maintenance also has to finish its asynchronous flash save.
                # A previous successful save is not evidence for this action.
                persistence_complete = (sent_type != MSG_MAINTENANCE_CALIBRATE or
                    (getattr(self, "storage_state", 0) == 3 and
                     getattr(self, "calibration_generation", 0) != initial_calibration_generation))
                if (observed_running and state and state.state == HeadState.READY and
                        time.monotonic() - self.last_state_monotonic_s < 0.25 and
                        not state.torque_enabled and
                        state.torque_state == 1 and not state.shutdown_pending and
                        persistence_complete):
                    result.success, result.reason = True, "completed"
                    goal_handle.succeed()
                    return result
                if state and state.state == HeadState.FAULT:
                    self.lease_token = 0
                    result.success, result.reason = False, "firmware fault"
                    goal_handle.abort()
                    return result
                if goal_handle.is_cancel_requested:
                    self.transport.request(MSG_DISABLE, struct.pack("<I", goal_handle.request.lease_token))
                    self.lease_token = 0
                    goal_handle.canceled()
                    result.success, result.reason = False, "cancelled"
                    return result
                if time.monotonic() >= deadline_monotonic_s:
                    self.transport.request(MSG_DISABLE, struct.pack("<I", goal_handle.request.lease_token))
                    self.lease_token = 0
                    result.success, result.reason = False, "timed out; firmware disable requested"
                    goal_handle.abort()
                    return result
                # Renewal belongs to the action client/collector. Keeping an
                # action coroutine alive is not evidence its owner is alive.
                await poll_tick
                poll_tick = Future(executor=self.executor)
        finally:
            self.destroy_timer(poll_timer)

    async def home_execute(self, goal_handle):
        return await self._wait_for_ready(goal_handle, HomeHead, MSG_HOME)

    async def calibration_execute(self, goal_handle):
        result = await self._wait_for_ready(goal_handle, MaintenanceCalibrate,
                                            MSG_MAINTENANCE_CALIBRATE)
        result.exported_profile_path = ""
        return result

    def on_state(self, payload: bytes) -> None:
        if len(payload) != 32 + 24 * SERVO_COUNT:
            self.packet_errors += 1
            return
        state = HeadState()
        state.header = Header()
        state.header.stamp = self.get_clock().now().to_msg()
        (state.state, state.fault, torque, stalled, state.fan_rpm,
         state.mcu_uptime_ms, state.applied_sequence,
         state.control_hz) = struct.unpack_from("<BBBBHIIH", payload, 0)
        state.torque_enabled, state.fan_stalled = bool(torque), bool(stalled)
        (state.torque_state, state.shutdown_confirmations, shutdown_pending, _,
         state.shutdown_attempts, state.shutdown_failures) = struct.unpack_from(
            "<BBBBII", payload, 16)
        state.shutdown_pending = bool(shutdown_pending)
        incoming_boot_session_id = struct.unpack_from("<I", payload, 28)[0]
        if (self.controller_session_id is not None and
                incoming_boot_session_id != self.controller_session_id):
            self._reset_controller_session("controller reboot detected in state telemetry")
        self.controller_session_id = incoming_boot_session_id
        state.boot_session_id = incoming_boot_session_id
        state.active_servo_mask = self.active_servo_mask
        offset = 32
        for index, name in enumerate(self.joint_names):
            goal, present, velocity, current, voltage, temperature, moving, hw_error, online, age = struct.unpack_from(
                "<iiihHBBBBI", payload, offset)
            offset += 24
            servo = ServoState()
            servo.name = name
            servo.goal_tick, servo.present_tick = goal, present
            servo.present_velocity_raw = velocity
            servo.goal_normalized = float("nan")
            servo.present_normalized = float("nan")
            servo.current_ma, servo.voltage_mv = current, voltage
            servo.temperature_c, servo.moving_status, servo.hardware_error = temperature, moving, hw_error
            servo.online, servo.feedback_age_ms = bool(online), age
            state.servos.append(servo)
        self.latest_state = state
        self.last_state_monotonic_s = time.monotonic()
        self.state_publisher.publish(state)

    def on_diagnostics(self, payload: bytes) -> None:
        if len(payload) < 14:
            self.packet_errors += 1
            return
        (_, _, self.firmware_control_deadline_misses, _,
         self.firmware_control_max_period_us) = struct.unpack_from("<BBIII", payload, 0)
        offset = 14
        self.firmware_protocol_errors = 0
        self.branch_diagnostics.clear()
        while offset + 36 <= len(payload) - 12:
            branch, _, _, _ = struct.unpack_from("<BBBB", payload, offset)
            (_, _, _, protocol_errors, bus_errors, _, transmission_errors,
             deferred) = struct.unpack_from("<IIIIIIII", payload, offset + 4)
            self.firmware_protocol_errors += protocol_errors
            self.branch_diagnostics[branch] = (
                protocol_errors, bus_errors, transmission_errors, deferred)
            offset += 36
        if len(payload) - offset >= 12:
            (self.usb_dropped_frames, self.usb_dropped_bytes,
             self.usb_coalesced_frames) = struct.unpack_from("<III", payload, offset)
            offset += 12
        if len(payload) - offset >= 12:
            self.storage_state = payload[offset]
            (self.storage_result,
             self.calibration_generation) = struct.unpack_from("<iI", payload, offset + 4)
            offset += 12
        if len(payload) - offset >= 4:
            self.reset_cause = struct.unpack_from("<I", payload, offset)[0]

    def publish_diagnostics(self) -> None:
        if not self.configuration_synced:
            self._begin_negotiation()
        diagnostic = HeadDiagnostics()
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        diagnostic.lease_token = self.lease_token
        diagnostic.control_deadline_misses = self.firmware_control_deadline_misses
        diagnostic.control_max_period_us = self.firmware_control_max_period_us
        diagnostic.packet_errors = self.packet_errors + self.firmware_protocol_errors
        diagnostic.stale_servo_count = 0 if self.latest_state is None else sum(
            not servo.online or servo.feedback_age_ms > 30 for servo in self.latest_state.servos)
        diagnostic.usb_dropped_frames = getattr(self, "usb_dropped_frames", 0)
        diagnostic.usb_dropped_bytes = getattr(self, "usb_dropped_bytes", 0)
        diagnostic.usb_coalesced_frames = getattr(self, "usb_coalesced_frames", 0)
        diagnostic.storage_state = getattr(self, "storage_state", 0)
        diagnostic.storage_result = getattr(self, "storage_result", 0)
        diagnostic.calibration_generation = getattr(self, "calibration_generation", 0)
        diagnostic.reset_cause = getattr(self, "reset_cause", 0)
        diagnostic.detail = getattr(self, "firmware_identity", "firmware identity pending") + "; " + "; ".join(
            f"branch {branch}: protocol={protocol} bus={bus} tx={transmission} deferred={deferred}"
            for branch, (protocol, bus, transmission, deferred) in
            sorted(self.branch_diagnostics.items()))
        self.diagnostic_publisher.publish(diagnostic)


def main() -> None:
    rclpy.init()
    node = HeadBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
