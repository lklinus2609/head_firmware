"""Host protocol regressions using real PTYs and production transport/parser code.

Uses a small serial adapter because pySerial and ROS are not installed. This
exercises OS byte streams/threads, but does not emulate a Jazzy executor.
"""
import ast
import asyncio
import os
from pathlib import Path
import runpy
import select
import struct
import threading
import time
import tty
import zlib
import importlib.util
import math
import sys

ROOT = Path(__file__).resolve().parents[1]
helpers = runpy.run_path(str(ROOT / 'tests/host_transport_test.py'))
Future = helpers['TestFuture']
module = helpers['transport_module']
bridge_source = (ROOT / 'host_ros/src/head_ros/head_ros/bridge.py').read_text()
bridge_ast = ast.parse(bridge_source)
parser_node = next(node for node in bridge_ast.body if isinstance(node, ast.ClassDef) and node.name == 'FrameParser')
namespace = {'time': time, 'struct': struct, 'zlib': zlib, 'SOF': 0xA5, 'VERSION': 2,
             'MAX_PAYLOAD': 512, 'Optional': __import__('typing').Optional}
exec(compile(ast.Module(body=[parser_node], type_ignores=[]), '<production FrameParser>', 'exec'), namespace)
Parser = namespace['FrameParser']

def encode(kind, payload=b''):
    body = bytes((2, kind)) + struct.pack('<H', len(payload)) + payload
    return b'\xa5' + body + struct.pack('<I', zlib.crc32(body))

def wait_until(predicate, timeout_s=1.0):
    deadline_monotonic_s = time.monotonic() + timeout_s
    while not predicate():
        assert time.monotonic() < deadline_monotonic_s, 'timed out'
        time.sleep(0.001)

class PtySerial:
    def __init__(self, path, baud, timeout, write_timeout):
        del baud
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.timeout = timeout
        self.write_timeout = write_timeout
    @property
    def in_waiting(self):
        return 1 if select.select([self.fd], [], [], 0)[0] else 0
    def read(self, size):
        if not select.select([self.fd], [], [], self.timeout)[0]: return b''
        return os.read(self.fd, size)
    def write(self, data):
        return os.write(self.fd, data[:3])  # deliberately fragment every frame
    def close(self):
        os.close(self.fd)


def load_headctl():
    sys.modules.setdefault('serial', __import__('types').SimpleNamespace(Serial=object))
    spec = importlib.util.spec_from_file_location(
        'headctl_for_integration', ROOT / 'host_ros/src/head_ros/head_ros/headctl.py')
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


headctl = load_headctl()


class CallbackFuture:
    def __init__(self):
        self.callbacks, self.value, self.error, self.is_done = [], None, None, False
    def add_done_callback(self, callback):
        if self.is_done: callback(self)
        else: self.callbacks.append(callback)
    def set_result(self, value):
        self.value, self.is_done = value, True
        for callback in self.callbacks: callback(self)
    def set_exception(self, error):
        self.error, self.is_done = error, True
        for callback in self.callbacks: callback(self)
    def result(self):
        if self.error: raise self.error
        return self.value


def position_range_payload(transaction_id, generation, ranges, homed_mask=0):
    payload = bytearray(headctl.POSITION_RANGE_PAYLOAD_LENGTH)
    valid_mask = sum(1 << index for index in ranges)
    struct.pack_into('<IB3xII', payload, 0, transaction_id, generation, valid_mask, homed_mask)
    for index, (minimum, maximum) in ranges.items():
        struct.pack_into('<ii', payload, 16 + index * 8, minimum, maximum)
    return bytes(payload)


def test_position_range_cache():
    from types import SimpleNamespace, MethodType
    class State:
        def __init__(self): self.servos = []
    class Fields: pass
    test_namespace = {'HeadState': State, 'Header': Fields, 'ServoState': Fields,
                      'SERVO_COUNT': 20, 'struct': struct, 'time': time,
                      'STATE_POSITION_RANGE_GENERATION_OFFSET':
                          headctl.STATE_POSITION_RANGE_GENERATION_OFFSET,
                      'CAP_POSITION_RANGE': headctl.CAP_POSITION_RANGE,
                      'MSG_GET_POSITION_RANGE': headctl.GET_POSITION_RANGE,
                      'MSG_POSITION_RANGE': headctl.POSITION_RANGE,
                      'decode_position_range': headctl.decode_position_range,
                      'normalized_position': headctl.normalized_position,
                      'TransportError': module.TransportError, 'Future': CallbackFuture}
    bridge_class = next(node for node in bridge_ast.body if isinstance(node, ast.ClassDef) and node.name == 'HeadBridge')
    methods = [node for node in bridge_class.body if isinstance(node, ast.FunctionDef) and
               node.name in ('on_state', '_current_position_ranges', '_request_position_range')]
    exec(compile(ast.Module(body=methods, type_ignores=[]), '<production bridge ranges>', 'exec'), test_namespace)
    requests = []
    def request(message_type, timeout_s, response_types):
        future = CallbackFuture()
        requests.append((message_type, response_types, future))
        return future
    warnings = []
    bridge = SimpleNamespace(controller_session_id=17, negotiation_generation=0,
        active_servo_mask=0b11, packet_errors=0,
        joint_names=[f'servo_{index}' for index in range(20)],
        state_publisher=SimpleNamespace(publish=lambda state: None),
        transport=SimpleNamespace(request=request),
        get_logger=lambda: SimpleNamespace(warning=warnings.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)),
        capabilities=headctl.CAP_POSITION_RANGE, position_range=None,
        position_range_request_pending=False)
    for name in ('_current_position_ranges', '_request_position_range'):
        setattr(bridge, name, MethodType(test_namespace[name], bridge))
    on_state = lambda payload: test_namespace['on_state'](bridge, bytes(payload))

    payload = bytearray(512)
    struct.pack_into('<I', payload, 28, 17)
    payload[headctl.STATE_POSITION_RANGE_GENERATION_OFFSET] = 5
    struct.pack_into('<ii', payload, 32, -450, -1200)          # servo 0 goal, present
    struct.pack_into('<ii', payload, 32 + 24, 0, 0)            # servo 1
    # No range yet: NaN, and exactly one request however many samples arrive.
    on_state(payload)
    on_state(payload)
    assert math.isnan(bridge.latest_state.servos[0].present_normalized)
    assert len(requests) == 1 and requests[0][0] == headctl.GET_POSITION_RANGE
    assert requests[0][1] == (headctl.POSITION_RANGE,)
    requests[0][2].set_result((True, position_range_payload(9, 5, {0: (-1200, 300)}), 46))
    on_state(payload)
    servo0, servo1 = bridge.latest_state.servos[0], bridge.latest_state.servos[1]
    assert servo0.present_normalized == 0.0 and servo0.goal_normalized == 0.5
    # A servo without a valid range stays NaN.
    assert math.isnan(servo1.present_normalized) and len(requests) == 1
    # Outside the range is reported unclamped.
    struct.pack_into('<i', payload, 36, 330)
    on_state(payload)
    assert bridge.latest_state.servos[0].present_normalized == 1.02
    # A new generation (homing, profile applied) invalidates the cache at once.
    payload[headctl.STATE_POSITION_RANGE_GENERATION_OFFSET] = 6
    on_state(payload)
    assert math.isnan(bridge.latest_state.servos[0].present_normalized) and len(requests) == 2
    # A failed request is logged, and the next sample asks again.
    requests[1][2].set_exception(module.TransportTimeout('no reply'))
    assert bridge.position_range is None and warnings
    on_state(payload)
    assert len(requests) == 3
    # Firmware without the capability is never asked.
    bridge.capabilities = 0
    on_state(payload)
    assert len(requests) == 3 and math.isnan(bridge.latest_state.servos[0].present_normalized)

    # Decoder and transport matching.
    decoded = headctl.decode_position_range(position_range_payload(9, 5, {0: (-1200, 300)}, 1))
    assert decoded['ranges'] == {0: (-1200, 300)} and decoded['homed_mask'] == 1
    for bad in (b'\0' * 10, position_range_payload(9, 5, {0: (300, 300)})):
        try:
            headctl.decode_position_range(bad)
        except RuntimeError:
            pass
        else:
            raise AssertionError('accepted a malformed position range')
    assert module.SerialTransport._response_transaction(
        46, position_range_payload(0x01020304, 5, {})) == 0x01020304


def test_state_and_owner_lifecycle():
    from types import SimpleNamespace, MethodType
    class State:
        HOMING = 3
        MAINTENANCE_CALIBRATION = 4
        READY = 5
        FAULT = 7
        def __init__(self): self.servos = []
    class Fields: pass
    test_namespace = {'HeadState': State, 'Header': Fields, 'ServoState': Fields,
                      'SERVO_COUNT': 20, 'struct': struct, 'time': time,
                      'STATE_POSITION_RANGE_GENERATION_OFFSET':
                          headctl.STATE_POSITION_RANGE_GENERATION_OFFSET,
                      'CAP_POSITION_RANGE': headctl.CAP_POSITION_RANGE,
                      'MSG_GET_POSITION_RANGE': headctl.GET_POSITION_RANGE,
                      'MSG_POSITION_RANGE': headctl.POSITION_RANGE,
                      'decode_position_range': headctl.decode_position_range,
                      'normalized_position': headctl.normalized_position,
                      'TransportError': module.TransportError, 'Future': Future}
    bridge_class = next(node for node in bridge_ast.body if isinstance(node, ast.ClassDef) and node.name == 'HeadBridge')
    methods = [node for node in bridge_class.body if isinstance(node, ast.FunctionDef) and
               node.name in ('on_state', '_reset_controller_session',
                             '_current_position_ranges', '_request_position_range')]
    exec(compile(ast.Module(body=methods, type_ignores=[]), '<production bridge lifecycle>', 'exec'), test_namespace)
    resets = []
    published = []
    bridge = SimpleNamespace(controller_session_id=17, lease_token=123, sequence=5,
        negotiation_generation=0, negotiation_in_progress=False, configuration_synced=True,
        hello_validated=True, active_servo_mask=1, packet_errors=0,
        joint_names=[f'servo_{index}' for index in range(20)],
        state_publisher=SimpleNamespace(publish=published.append),
        transport=SimpleNamespace(reset_pending=resets.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)),
        capabilities=0, position_range=None, position_range_request_pending=False)
    bridge._reset_controller_session = MethodType(test_namespace['_reset_controller_session'], bridge)
    for name in ('_current_position_ranges', '_request_position_range'):
        setattr(bridge, name, MethodType(test_namespace[name], bridge))
    payload = bytearray(512)
    struct.pack_into('<BBBBHIIH', payload, 0, 9, 0, 1, 0, 5000, 123456, 1, 500)
    payload[16] = 2
    struct.pack_into('<I', payload, 28, 17)
    test_namespace['on_state'](bridge, bytes(payload))
    assert bridge.lease_token == 123 and bridge.latest_state.boot_session_id == 17
    struct.pack_into('<I', payload, 28, 18)
    test_namespace['on_state'](bridge, bytes(payload))
    assert bridge.lease_token == 0 and not bridge.configuration_synced and not bridge.hello_validated
    assert bridge.latest_state.boot_session_id == 18 and len(resets) == 1

    collector_source = (ROOT / 'host_ros/src/head_ros/head_ros/collector.py').read_text()
    collector_ast = ast.parse(collector_source)
    renewal = next(node for node in ast.walk(collector_ast) if isinstance(node, ast.FunctionDef) and node.name == '_renewal_is_active')
    phases = SimpleNamespace(**{name: index for index, name in enumerate(
        ('ACQUIRING','HOMING','ENTERING_HOLD','WARMING_UP','ARMED','RECORDING','STOPPING','ABORTING','SAVED','ERROR'))})
    collector_namespace = {'SessionPhase': phases, 'time': time}
    exec(compile(ast.Module(body=[renewal], type_ignores=[]), '<production collector ownership>', 'exec'), collector_namespace)
    collector = SimpleNamespace(lease_token=123, renew_pending=False, state=SimpleNamespace(phase=phases.RECORDING),
        latest_head=SimpleNamespace(boot_session_id=17), lease_boot_session_id=17,
        last_head_received_monotonic_s=time.monotonic())
    can_renew = collector_namespace['_renewal_is_active']
    assert can_renew(collector)
    for phase in (phases.STOPPING, phases.ABORTING, phases.SAVED, phases.ERROR):
        collector.state.phase = phase
        assert not can_renew(collector)
    collector.state.phase = phases.RECORDING
    collector.last_head_received_monotonic_s -= 1
    assert not can_renew(collector)
    collector.last_head_received_monotonic_s = time.monotonic()
    collector.latest_head.boot_session_id = 18
    assert not can_renew(collector)


def test_action_completion_and_cancellation():
    from types import SimpleNamespace
    action_node = next(node for node in ast.walk(bridge_ast)
                       if isinstance(node, ast.AsyncFunctionDef) and node.name == '_wait_for_ready')
    state_codes = SimpleNamespace(HOMING=3, MAINTENANCE_CALIBRATION=4, READY=5, FAULT=7)
    namespace = {'struct': struct, 'time': time, 'HeadState': state_codes,
                 'MSG_MAINTENANCE_CALIBRATE': 20, 'MSG_DISABLE': 9}
    exec(compile(ast.Module(body=[action_node], type_ignores=[]),
                 '<production action completion>', 'exec'), namespace)
    for cancel in (False, True):
        events = []
        polls = []
        disables = []
        state = SimpleNamespace(state=4, torque_enabled=True, torque_state=2,
                                shutdown_pending=False)
        async def request(*args): return True, b''
        bridge = SimpleNamespace(request=request, calibration_generation=9, storage_state=3,
            negotiation_generation=1, executor=None, latest_state=state, lease_token=123,
            last_state_monotonic_s=time.monotonic(),
            create_timer=lambda *args: object(), destroy_timer=lambda timer: events.append('timer destroyed'),
            transport=SimpleNamespace(request=lambda kind, payload: disables.append((kind, payload))))
        class PollFuture:
            def __init__(self, **kwargs): pass
            def __await__(self):
                polls.append(1)
                state.state, state.torque_enabled, state.torque_state = 5, False, 1
                # First READY still carries an old successful-save diagnostic.
                if len(polls) == 2: bridge.storage_state = 2
                if len(polls) == 3:
                    bridge.storage_state = 3
                    bridge.calibration_generation = 10
                assert len(polls) <= 3, 'maintenance did not complete after new save'
                yield from ()
            def done(self): return False
        namespace['Future'] = PollFuture
        goal = SimpleNamespace(request=SimpleNamespace(lease_token=123),
            is_cancel_requested=cancel, publish_feedback=lambda feedback: None,
            abort=lambda: events.append('abort'), succeed=lambda: events.append('success'),
            canceled=lambda: events.append('cancel'))
        action = SimpleNamespace(Result=SimpleNamespace, Feedback=SimpleNamespace)
        result = asyncio.run(namespace['_wait_for_ready'](bridge, goal, action, 20))
        if cancel:
            assert not result.success and 'cancel' in events
            assert disables == [(9, struct.pack('<I', 123))]
            assert bridge.lease_token == 0
        else:
            assert result.success and 'success' in events and len(polls) == 3
        assert events[-1] == 'timer destroyed'


def test_bridge_diagnostics_layout():
    # Production on_diagnostics with the shared headctl decoder, on a payload
    # built field by field in the order send_diagnostics() (src/main.c) writes.
    import importlib.util
    import sys
    from types import SimpleNamespace
    sys.modules.setdefault('serial', SimpleNamespace(Serial=object))
    spec = importlib.util.spec_from_file_location(
        'headctl_for_bridge_test', ROOT / 'host_ros/src/head_ros/head_ros/headctl.py')
    headctl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(headctl)
    bridge_class = next(node for node in bridge_ast.body if isinstance(node, ast.ClassDef) and node.name == 'HeadBridge')
    method = next(node for node in bridge_class.body if isinstance(node, ast.FunctionDef) and
                  node.name == 'on_diagnostics')
    test_namespace = {'decode_diagnostics': headctl.decode_diagnostics}
    exec(compile(ast.Module(body=[method], type_ignores=[]), '<production bridge diagnostics>', 'exec'), test_namespace)
    payload = bytearray(209)
    struct.pack_into('<BBIII', payload, 0, 5, 0, 7, 1234, 2100)
    for branch in range(4):
        struct.pack_into('<BBBBIIIIIIII', payload, 14 + 36 * branch, branch, 0, 0x1F, 0x1F,
                         0, 0, 0, 10 + branch, 20 + branch, 0, 30 + branch, 40 + branch)
    struct.pack_into('<III', payload, 158, 1, 2, 3)        # USB drops/bytes/coalesced
    payload[170] = 3                                        # storage succeeded
    struct.pack_into('<iI', payload, 174, 0, 42)            # storage result, generation
    struct.pack_into('<I', payload, 182, 0x10)              # boot reset cause: watchdog
    bridge = SimpleNamespace(packet_errors=0, branch_diagnostics={})
    test_namespace['on_diagnostics'](bridge, bytes(payload))
    assert bridge.packet_errors == 0
    assert sorted(bridge.branch_diagnostics) == [0, 1, 2, 3]
    assert bridge.branch_diagnostics[3] == (13, 23, 33, 43)
    assert bridge.firmware_protocol_errors == 10 + 11 + 12 + 13
    assert bridge.firmware_control_deadline_misses == 7
    assert bridge.firmware_control_max_period_us == 2100
    assert (bridge.usb_dropped_frames, bridge.usb_dropped_bytes, bridge.usb_coalesced_frames) == (1, 2, 3)
    assert bridge.storage_state == 3 and bridge.storage_result == 0
    assert bridge.calibration_generation == 42 and bridge.reset_cause == 0x10
    test_namespace['on_diagnostics'](bridge, bytes(payload[:186]))
    assert bridge.packet_errors == 1
    print('bridge DIAGNOSTICS layout: PASS')


def main():
    test_state_and_owner_lifecycle()
    test_position_range_cache()
    print('bridge position range cache: PASS')
    test_action_completion_and_cancellation()
    test_bridge_diagnostics_layout()
    # Services/actions use reentrant groups; suspended serial futures must not
    # retain the target/timer callback group. This is a structural guard, not ROS QA.
    for node in ast.walk(bridge_ast):
        if isinstance(node, ast.Call) and ((isinstance(node.func, ast.Attribute) and
                node.func.attr == 'create_service') or
                (isinstance(node.func, ast.Name) and node.func.id == 'ActionServer')):
            assert any(keyword.arg == 'callback_group' for keyword in node.keywords)
    owner = module.SerialTransport('/unused', 115200, encode, lambda *args: None,
                                  lambda error: None, Future, session_id_provider=lambda: 17)
    assert owner.send(5, b'old', coalesce_key='targets')
    assert owner.send(5, b'new', coalesce_key='targets')
    request = owner.request(9, struct.pack('<I', 123))
    queued = sorted(owner._commands.queue)
    assert len(queued) == 2 and queued[0][2].message_type == 9
    assert queued[1][2].frame == encode(5, b'new')
    owner.reset_pending()
    assert owner._commands.empty() and request.done()
    try: request.result()
    except module.TransportDisconnected: pass
    else: raise AssertionError('reset did not fail queued request')

    master, slave = os.openpty()
    tty.setraw(master); tty.setraw(slave)
    stop = threading.Event()
    received = []
    delayed = []
    session_id = 17
    parser = Parser()
    transport = module.SerialTransport(os.ttyname(slave), 115200, encode,
        lambda *args: None, lambda error: None, Future,
        session_id_provider=lambda: session_id, serial_factory=PtySerial)
    def parse_byte(byte):
        frame = parser.push(byte)
        if frame: transport.handle_frame(*frame)
    transport.set_byte_handler(parse_byte)
    def peer():
        peer_parser = Parser()
        while not stop.is_set():
            if select.select([master], [], [], 0.005)[0]:
                for byte in os.read(master, 4096):
                    frame = peer_parser.push(byte)
                    if not frame: continue
                    received.append(frame)
                    kind, payload = frame
                    if kind == 3:
                        transaction = payload[-4:]
                        # Both wrong-request and truncated ACKs must be ignored.
                        os.write(master, encode(14, b'\x09\0' + struct.pack('<I', 123) + transaction + struct.pack('<I', 17)))
                        os.write(master, encode(14, b'\x03\0' + struct.pack('<I', 123) + transaction))
                        delayed.append((time.monotonic() + 0.10,
                            encode(14, b'\x03\0' + struct.pack('<I', 123) + transaction + struct.pack('<I', 17))))
                    elif kind == 9:
                        os.write(master, encode(14, bytes((kind, 0)) + b'\0'*4 + payload[-4:] + struct.pack('<I', 17)))
            for due, frame in list(delayed):
                if time.monotonic() >= due:
                    os.write(master, frame); delayed.remove((due, frame))
    thread = threading.Thread(target=peer)
    thread.start(); transport.start()
    try:
        wait_until(lambda: transport.connected)
        acquire = transport.request(3, b'owner', timeout_s=0.5)
        wait_until(lambda: any(frame[0] == 3 for frame in received))
        time.sleep(0.02)
        assert not acquire.done(), 'malformed/wrong ACK was accepted'
        assert transport.send(5, b'latest', coalesce_key='targets')
        disable = transport.request(9, struct.pack('<I', 123), timeout_s=0.2)
        wait_until(disable.done)
        assert disable.result()[0] and not acquire.done()
        wait_until(acquire.done)
        assert acquire.result()[0]
        assert any(frame == (5, b'latest') for frame in received)
        assert len(next(payload for kind, payload in received if kind == 9)) == 8
    finally:
        transport.close(); stop.set(); thread.join(timeout=1)
        os.close(master); os.close(slave)
    print('host PTY fragmentation/delayed ACK/priority/reset: PASS')

if __name__ == '__main__': main()
