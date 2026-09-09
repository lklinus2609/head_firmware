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
                      'SERVO_COUNT': 20, 'struct': struct, 'time': time}
    bridge_class = next(node for node in bridge_ast.body if isinstance(node, ast.ClassDef) and node.name == 'HeadBridge')
    methods = [node for node in bridge_class.body if isinstance(node, ast.FunctionDef) and
               node.name in ('on_state', '_reset_controller_session')]
    exec(compile(ast.Module(body=methods, type_ignores=[]), '<production bridge lifecycle>', 'exec'), test_namespace)
    resets = []
    published = []
    bridge = SimpleNamespace(controller_session_id=17, lease_token=123, sequence=5,
        negotiation_generation=0, negotiation_in_progress=False, configuration_synced=True,
        hello_validated=True, active_servo_mask=1, packet_errors=0,
        joint_names=[f'servo_{index}' for index in range(20)],
        state_publisher=SimpleNamespace(publish=published.append),
        transport=SimpleNamespace(reset_pending=resets.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)))
    bridge._reset_controller_session = MethodType(test_namespace['_reset_controller_session'], bridge)
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


def main():
    test_state_and_owner_lifecycle()
    test_action_completion_and_cancellation()
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
