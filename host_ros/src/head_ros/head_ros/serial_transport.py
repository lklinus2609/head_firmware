"""Bounded, single-owner transport for the framed head-control protocol.

The transport thread is the only code that touches the pySerial object.  ROS
callbacks enqueue work and await the returned rclpy future; they never read,
write, or sleep on the serial endpoint themselves.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import queue
import threading
import time
from typing import Any, Callable, Optional

from rclpy.task import Future


class TransportError(RuntimeError):
    """Base class for transport failures."""


class TransportDisconnected(TransportError):
    """The serial endpoint disconnected or could not be used."""


class TransportTimeout(TransportError):
    """A queued request did not receive its response before its deadline_monotonic_s."""


@dataclass
class _Request:
    message_type: int
    payload: bytes
    deadline_monotonic_s: float
    response_types: frozenset[int]
    future: Future
    transaction_id: int = 0
    epoch: int = 0


@dataclass
class _Send:
    frame: bytes
    deadline_monotonic_s: float
    priority: int = 20
    coalesce_key: Optional[str] = None
    epoch: int = 0


class SerialTransport:
    """A reconnecting serial transport with bounded requests and writes.

    ``encode_frame`` is injected to keep this module independent of the wire
    parser and easy to exercise with a PTY or a fake serial object.
    """

    def __init__(
        self,
        port: str,
        baud: int,
        encode_frame: Callable[[int, bytes], bytes],
        on_frame: Callable[[int, bytes], None],
        on_disconnect: Callable[[BaseException], None],
        future_factory: Callable[[], Future],
        session_id_provider: Optional[Callable[[], Optional[int]]] = None,
        serial_factory: Optional[Callable[..., Any]] = None,
        queue_size: int = 32,
        write_timeout_s: float = 0.05,
        request_timeout_s: float = 1.0,
        reconnect_delay_s: float = 0.25,
    ) -> None:
        self.port = port
        self.baud = int(baud)
        self._encode_frame = encode_frame
        self._on_frame = on_frame
        self._on_disconnect = on_disconnect
        self._future_factory = future_factory
        self._session_id_provider = session_id_provider
        self._serial_factory = serial_factory
        self.write_timeout_s = max(0.001, float(write_timeout_s))
        self.request_timeout_s = max(0.01, float(request_timeout_s))
        self.reconnect_delay_s = max(0.01, float(reconnect_delay_s))
        self._commands: queue.PriorityQueue[tuple[int, int, _Request | _Send | None]] = queue.PriorityQueue(maxsize=queue_size)
        self._queue_order = 0
        self._queue_size = queue_size
        self._pending: dict[int, _Request] = {}
        self._transaction_id = 0
        self._epoch = 0
        self._state_lock = threading.Lock()
        self._serial: Any = None
        self._byte_handler: Callable[[int], None] = lambda byte: None
        self._closed = threading.Event()
        self._thread = threading.Thread(target=self._run, name="head_serial_transport", daemon=True)

    def start(self) -> None:
        self._thread.start()

    @property
    def connected(self) -> bool:
        with self._state_lock:
            return self._serial is not None

    def _new_future(self) -> Future:
        return self._future_factory()

    def _next_transaction_id(self) -> int:
        self._transaction_id = (self._transaction_id + 1) & 0xFFFFFFFF
        if self._transaction_id == 0:
            self._transaction_id = 1
        return self._transaction_id

    def _enqueue(self, command: _Request | _Send | None, priority: int) -> None:
        with self._commands.mutex:
            if priority == 0 and len(self._commands.queue) >= self._queue_size:
                for index, item in enumerate(self._commands.queue):
                    if isinstance(item[2], _Send):
                        self._commands.queue.pop(index)
                        heapq.heapify(self._commands.queue)
                        break
            if len(self._commands.queue) >= self._queue_size - (1 if priority > 0 else 0):
                raise queue.Full
            self._queue_order += 1
            self._commands.queue.append((priority, self._queue_order, command))
            heapq.heapify(self._commands.queue)
            self._commands.unfinished_tasks += 1
            self._commands.not_empty.notify()

    def request(
        self,
        message_type: int,
        payload: bytes = b"",
        timeout_s: Optional[float] = None,
        response_types: tuple[int, ...] = (14, 15),
    ) -> Future:
        """Queue one transaction and return its nonblocking future.

        The transaction ID is appended to the request payload, preserving the
        existing protocol.  A full queue fails immediately instead of making
        a ROS callback wait behind an unbounded backlog.
        """
        future = self._new_future()
        timeout = self.request_timeout_s if timeout_s is None else max(0.001, float(timeout_s))
        request = _Request(
            int(message_type), bytes(payload), time.monotonic() + timeout,
            frozenset(int(value) for value in response_types), future, epoch=self._epoch)
        try:
            # Safety commands bypass sampled target traffic in the queue.
            priority = 0 if int(message_type) in (4, 9) else 10
            self._enqueue(request, priority)
        except queue.Full:
            future.set_exception(TransportError("serial outbound queue is full"))
        return future

    def send(self, message_type: int, payload: bytes = b"", timeout_s: float = 0.1,
             coalesce_key: Optional[str] = None) -> bool:
        """Queue a fire-and-forget frame without blocking the caller."""
        command = _Send(self._encode_frame(int(message_type), bytes(payload)),
                        time.monotonic() + max(0.001, float(timeout_s)),
                        coalesce_key=coalesce_key, epoch=self._epoch)
        try:
            with self._commands.mutex:
                if coalesce_key is not None:
                    retained = [item for item in self._commands.queue
                                if not (isinstance(item[2], _Send) and
                                        item[2].coalesce_key == coalesce_key)]
                    self._commands.queue[:] = retained
                    heapq.heapify(self._commands.queue)
                if len(self._commands.queue) >= self._queue_size:
                    return False
                self._queue_order += 1
                self._commands.queue.append((command.priority, self._queue_order, command))
                heapq.heapify(self._commands.queue)
                self._commands.unfinished_tasks += 1
                self._commands.not_empty.notify()
        except queue.Full:
            return False
        return True

    def reset_pending(self, reason: str = "controller session reset") -> None:
        """Fail all outstanding requests, retaining the serial connection."""
        with self._state_lock:
            self._epoch += 1
            pending = list(self._pending.values())
            self._pending.clear()
        error = TransportDisconnected(reason)
        for request in pending:
            if not request.future.done():
                request.future.set_exception(error)
        queued_requests: list[_Request] = []
        with self._commands.mutex:
            retained = []
            for item in self._commands.queue:
                command = item[2]
                if isinstance(command, _Request):
                    queued_requests.append(command)
                    self._commands.unfinished_tasks -= 1
                elif command is not None:
                    self._commands.unfinished_tasks -= 1
            self._commands.queue[:] = retained
            self._commands.not_empty.notify_all()
        for request in queued_requests:
            if not request.future.done():
                request.future.set_exception(error)

    def close(self) -> None:
        self._closed.set()
        try:
            self._enqueue(None, -100)
        except queue.Full:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self.reset_pending("serial transport closed")

    def _open(self) -> Any:
        factory = self._serial_factory
        if factory is None:
            import serial
            factory = serial.Serial
        return factory(self.port, self.baud, timeout=0.01,
                       write_timeout=self.write_timeout_s)

    def _set_serial(self, serial_device: Any) -> None:
        with self._state_lock:
            self._serial = serial_device

    def _disconnect(self, error: BaseException) -> None:
        serial_device = self._serial
        self._set_serial(None)
        if serial_device is not None:
            try:
                serial_device.close()
            except Exception:
                pass
        self.reset_pending("serial endpoint disconnected")
        try:
            self._on_disconnect(error)
        except Exception:
            pass

    def _write_frame(self, serial_device: Any, frame: bytes, deadline_monotonic_s: float, epoch: int) -> None:
        offset = 0
        while offset < len(frame):
            if epoch != self._epoch:
                raise TransportDisconnected("controller epoch changed during output")
            if self._closed.is_set() or time.monotonic() >= deadline_monotonic_s:
                raise TransportTimeout("serial write deadline_monotonic_s expired")
            try:
                serial_device.write_timeout = min(self.write_timeout_s, max(0.001, deadline_monotonic_s - time.monotonic()))
                written = serial_device.write(frame[offset:])
            except Exception as error:
                raise TransportDisconnected(str(error)) from error
            if not isinstance(written, int) or written < 0 or written > len(frame) - offset:
                raise TransportError("invalid serial write byte count")
            if written <= 0:
                # pySerial may report zero progress when its bounded write
                # timeout expires.  Yield briefly, then enforce our deadline_monotonic_s.
                time.sleep(min(0.001, max(0.0, deadline_monotonic_s - time.monotonic())))
                continue
            offset += int(written)

    def _expire_requests(self, now_monotonic_s: float) -> None:
        expired: list[_Request] = []
        with self._state_lock:
            for transaction_id, request in list(self._pending.items()):
                if now_monotonic_s >= request.deadline_monotonic_s:
                    expired.append(self._pending.pop(transaction_id))
        for request in expired:
            if not request.future.done():
                request.future.set_exception(TransportTimeout("serial response deadline_monotonic_s expired"))

    @staticmethod
    def _response_transaction(message_type: int, payload: bytes) -> Optional[int]:
        if message_type in (14, 15) and len(payload) >= 10:
            return int.from_bytes(payload[6:10], "little")
        # HELLO_REPLY carries the transaction ID at the beginning.  The
        # configuration response intentionally has no transaction ID in v2.
        if message_type == 2 and len(payload) >= 4:
            return int.from_bytes(payload[:4], "little")
        return None

    def _resolve_response(self, message_type: int, payload: bytes) -> None:
        transaction_id = self._response_transaction(message_type, payload)
        selected: Optional[_Request] = None
        if transaction_id is not None:
            with self._state_lock:
                request = self._pending.get(transaction_id)
                session_matches = (request is not None and request.epoch == self._epoch and
                                   time.monotonic() < request.deadline_monotonic_s)
                if request is not None and message_type in (14, 15):
                    session_matches = session_matches and len(payload) == 14 and payload[0] == request.message_type
                    if self._session_id_provider is not None:
                        session_id = self._session_id_provider()
                        session_matches = (session_matches and session_id is not None and
                                           int.from_bytes(payload[10:14], "little") == int(session_id))
                if (request is not None and message_type in request.response_types and
                        session_matches):
                    selected = self._pending.pop(transaction_id)
        elif message_type == 18:
            # GET_CONFIGURATION_INFO is the only v2 request whose successful
            # data response has no transaction field.  Resolve the oldest
            # pending request of that type; its ACK is harmlessly late.
            with self._state_lock:
                for transaction_id, request in self._pending.items():
                    if (request.message_type == 17 and message_type in request.response_types and
                            request.epoch == self._epoch and time.monotonic() < request.deadline_monotonic_s):
                        selected = self._pending.pop(transaction_id)
                        break
        if selected is not None and not selected.future.done():
            ok = message_type not in (15,) and not (
                message_type == 14 and len(payload) >= 2 and payload[1] != 0)
            selected.future.set_result((ok, payload, message_type))

    def handle_frame(self, message_type: int, payload: bytes) -> None:
        """Deliver one decoded frame to transactions and the bridge."""
        self._resolve_response(message_type, payload)
        self._on_frame(message_type, payload)

    def _run(self) -> None:
        next_open_monotonic_s = 0.0
        while not self._closed.is_set():
            now_monotonic_s = time.monotonic()
            if self._serial is None and now_monotonic_s >= next_open_monotonic_s:
                try:
                    self._set_serial(self._open())
                except Exception as error:
                    next_open_monotonic_s = now_monotonic_s + self.reconnect_delay_s
                    try:
                        self._on_disconnect(error)
                    except Exception:
                        pass
            serial_device = self._serial
            if serial_device is None:
                self.reset_pending("serial endpoint unavailable")
                self._closed.wait(0.01)
                continue
            try:
                # Give queued commands priority while still polling RX often.
                for _ in range(4):
                    try:
                        _, _, command = self._commands.get_nowait()
                    except queue.Empty:
                        break
                    if command is None:
                        self._closed.set()
                        break
                    if command.epoch != self._epoch:
                        if isinstance(command, _Request) and not command.future.done():
                            command.future.set_exception(TransportDisconnected("stale queued command"))
                        continue
                    if isinstance(command, _Request):
                        if command.future.done():
                            continue
                        pending_limit = self._queue_size if command.message_type in (4, 9) else self._queue_size - 1
                        if len(self._pending) >= pending_limit:
                            command.future.set_exception(TransportError("too many outstanding requests"))
                            continue
                        if time.monotonic() >= command.deadline_monotonic_s:
                            command.future.set_exception(TransportTimeout("request expired before transmit"))
                            continue
                        transaction_id = self._next_transaction_id()
                        command.transaction_id = transaction_id
                        with self._state_lock:
                            self._pending[transaction_id] = command
                        self._write_frame(serial_device,
                                          self._encode_frame(command.message_type,
                                                             command.payload + transaction_id.to_bytes(4, "little")),
                                          min(command.deadline_monotonic_s,
                                              time.monotonic() + self.write_timeout_s), command.epoch)
                    else:
                        if time.monotonic() >= command.deadline_monotonic_s:
                            continue  # stale sampled targets are never transmitted
                        self._write_frame(serial_device, command.frame,
                                          min(command.deadline_monotonic_s, time.monotonic() + self.write_timeout_s),
                                          command.epoch)
                waiting = int(getattr(serial_device, "in_waiting", 0) or 0)
                data = serial_device.read(max(1, min(waiting, 4096)))
                for byte in data:
                    self._on_frame_byte(int(byte))
                self._expire_requests(time.monotonic())
            except BaseException as error:
                if self._closed.is_set():
                    break
                self._disconnect(error)
                next_open_monotonic_s = time.monotonic() + self.reconnect_delay_s
        self._disconnect(TransportDisconnected("serial transport stopped"))

    def _on_frame_byte(self, byte: int) -> None:
        # Installed by the bridge after construction.  Kept as a method so a
        # test can inject a parser without needing pySerial.
        self._byte_handler(byte)

    def set_byte_handler(self, byte_handler: Callable[[int], None]) -> None:
        self._byte_handler = byte_handler
