"""ROS-free checks for the bounded host serial transport."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import threading
import time
import types


class TestFuture:
    def __init__(self):
        self._event = threading.Event()
        self._value = None
        self._error = None

    def done(self):
        return self._event.is_set()

    def set_result(self, value):
        self._value = value
        self._event.set()

    def set_exception(self, error):
        self._error = error
        self._event.set()

    def result(self):
        if self._error:
            raise self._error
        return self._value


rclpy_task = types.ModuleType("rclpy.task")
rclpy_task.Future = TestFuture
sys.modules.setdefault("rclpy", types.ModuleType("rclpy"))
sys.modules["rclpy.task"] = rclpy_task
MODULE_PATH = (Path(__file__).parents[1] / "host_ros" / "src" / "head_ros" /
               "head_ros" / "serial_transport.py")
SPEC = importlib.util.spec_from_file_location("serial_transport_under_test", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
transport_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = transport_module
SPEC.loader.exec_module(transport_module)


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.output = bytearray()
        self.closed = False

    @property
    def in_waiting(self):
        return 0

    def write(self, data):
        chunk = bytes(data[:2])
        self.output.extend(chunk)
        return len(chunk)

    def read(self, size):
        time.sleep(0.001)
        return b""

    def close(self):
        self.closed = True


class StalledSerial(FakeSerial):
    def write(self, data):
        return 0


def main() -> None:
    frames = []
    serial_device = None

    def factory(*args, **kwargs):
        nonlocal serial_device
        serial_device = FakeSerial(*args, **kwargs)
        return serial_device

    transport = transport_module.SerialTransport(
        "/dev/fake", 115200, lambda kind, payload: bytes((kind,)) + payload,
        lambda kind, payload: frames.append((kind, payload)), lambda error: None,
        TestFuture, serial_factory=factory, write_timeout_s=0.01)
    transport.start()
    future = transport.request(3, b"payload", timeout_s=0.2,
                               response_types=(14,))
    deadline = time.monotonic() + 0.2
    while serial_device is None or len(serial_device.output) < 1:
        assert time.monotonic() < deadline
        time.sleep(0.001)
    # The request was fully emitted through short writes and has a transaction
    # ID in its final four bytes.
    assert bytes(serial_device.output).startswith(bytes((3,)))
    transaction_id = int.from_bytes(serial_device.output[-4:], "little")
    transport.handle_frame(14, bytes((3, 0)) + (0).to_bytes(4, "little") +
                           transaction_id.to_bytes(4, "little") +
                           (1).to_bytes(4, "little"))
    assert future.result()[0]
    transport.close()

    stalled = transport_module.SerialTransport(
        "/dev/fake", 115200, lambda kind, payload: b"frame",
        lambda kind, payload: None, lambda error: None,
        TestFuture, serial_factory=lambda *args, **kwargs: StalledSerial(),
        write_timeout_s=0.005)
    stalled.start()
    stalled_future = stalled.request(3, timeout_s=0.02)
    deadline = time.monotonic() + 0.5
    while not stalled_future.done():
        assert time.monotonic() < deadline
        time.sleep(0.001)
    try:
        stalled_future.result()
    except transport_module.TransportError:
        pass
    else:
        raise AssertionError("stalled output was not bounded")
    stalled.close()


if __name__ == "__main__":
    main()
