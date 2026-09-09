"""ROS-independent collection state and durable session storage helpers."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import IntEnum
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Iterable, TextIO


class SessionPhase(IntEnum):
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


_ALLOWED_TRANSITIONS = {
    SessionPhase.IDLE: {SessionPhase.ACQUIRING},
    SessionPhase.ACQUIRING: {
        SessionPhase.HOMING, SessionPhase.ENTERING_HOLD,
        SessionPhase.ABORTING, SessionPhase.ERROR,
    },
    SessionPhase.HOMING: {
        SessionPhase.ENTERING_HOLD, SessionPhase.ABORTING, SessionPhase.ERROR,
    },
    SessionPhase.ENTERING_HOLD: {
        SessionPhase.WARMING_UP, SessionPhase.ABORTING, SessionPhase.ERROR,
    },
    SessionPhase.WARMING_UP: {
        SessionPhase.ARMED, SessionPhase.ABORTING, SessionPhase.ERROR,
    },
    SessionPhase.ARMED: {
        SessionPhase.RECORDING, SessionPhase.ABORTING, SessionPhase.ERROR,
    },
    SessionPhase.RECORDING: {
        SessionPhase.STOPPING, SessionPhase.ABORTING, SessionPhase.ERROR,
    },
    SessionPhase.STOPPING: {
        SessionPhase.ABORTING, SessionPhase.SAVED, SessionPhase.ERROR,
    },
    SessionPhase.ABORTING: {SessionPhase.SAVED, SessionPhase.ERROR},
    SessionPhase.SAVED: {SessionPhase.IDLE},
    SessionPhase.ERROR: {SessionPhase.IDLE},
}


@dataclass
class CollectionState:
    phase: SessionPhase = SessionPhase.IDLE
    status: str = "idle"
    history: list[tuple[str, str]] = field(default_factory=list)

    def transition(self, target: SessionPhase, status: str) -> None:
        if target not in _ALLOWED_TRANSITIONS[self.phase]:
            raise ValueError(f"invalid collection transition {self.phase.name} -> {target.name}")
        self.phase = target
        self.status = status
        self.history.append((target.name, status))

    @property
    def can_start_recording(self) -> bool:
        return self.phase == SessionPhase.ARMED

    @property
    def can_stop_recording(self) -> bool:
        return self.phase == SessionPhase.RECORDING


def new_session_id(now: datetime | None = None) -> str:
    timestamp = now or datetime.now(timezone.utc)
    return timestamp.strftime("session_%Y%m%dT%H%M%S_%fZ")


def resolve_data_root(configured: str = "") -> Path:
    if configured:
        return Path(configured).expanduser().resolve()
    env_root = os.environ.get("HEAD_PROPRIOCEPTION_DATA_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve()
    project_root = os.environ.get("HEAD_PROJECT_ROOT")
    if project_root:
        return (Path(project_root).expanduser().resolve() /
                "proprioception" / "data" / "raw")
    for start in (Path.cwd(), Path(__file__).resolve()):
        for parent in (start, *start.parents):
            candidate = parent / "proprioception" / "data"
            if candidate.is_dir():
                return (candidate / "raw").resolve()
    raise RuntimeError(
        "cannot locate proprioception/data; set HEAD_PROJECT_ROOT or "
        "HEAD_PROPRIOCEPTION_DATA_ROOT"
    )


def _write_json(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class SessionWriter:
    """Streams loss-resistant JSONL and atomically publishes complete sessions."""

    def __init__(self, data_root: Path, session_id: str, metadata: dict[str, Any]):
        self.data_root = data_root.resolve()
        self.session_id = session_id
        self.partial_dir = self.data_root / ".partial" / session_id
        self.final_dir = self.data_root / session_id
        if self.partial_dir.exists() or self.final_dir.exists():
            raise FileExistsError(f"session path already exists: {session_id}")
        self.partial_dir.mkdir(parents=True)
        self.telemetry_path = self.partial_dir / "telemetry.jsonl"
        self.events_path = self.partial_dir / "events.jsonl"
        self._telemetry: TextIO = self.telemetry_path.open(
            "x", encoding="utf-8", buffering=1)
        self._events: TextIO = self.events_path.open(
            "x", encoding="utf-8", buffering=1)
        self.samples_received = 0
        self.samples_invalid = 0
        self.closed = False
        self.manifest: dict[str, Any] = {
            "schema_version": 1,
            "session_id": session_id,
            "status": "collecting",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            **metadata,
        }
        _write_json(self.partial_dir / "session.json", self.manifest)

    @staticmethod
    def _append(stream: TextIO, record: dict[str, Any]) -> None:
        stream.write(json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n")

    def append_telemetry(self, record: dict[str, Any], valid: bool) -> None:
        if self.closed:
            return
        self._append(self._telemetry, record)
        self.samples_received += 1
        if not valid:
            self.samples_invalid += 1

    def append_event(self, record: dict[str, Any]) -> None:
        if not self.closed:
            self._append(self._events, record)

    def update_manifest(self, **fields: Any) -> None:
        self.manifest.update(fields)
        _write_json(self.partial_dir / "session.json", self.manifest)

    def _close_streams(self) -> None:
        for stream in (self._telemetry, self._events):
            if not stream.closed:
                stream.flush()
                os.fsync(stream.fileno())
                stream.close()

    @staticmethod
    def _jsonl_records(path: Path) -> Iterable[dict[str, Any]]:
        with path.open(encoding="utf-8") as stream:
            for line in stream:
                if line.strip():
                    yield json.loads(line)

    def _write_parquet_if_available(self) -> bool:
        try:
            import pyarrow as pa
            import pyarrow.parquet as pq
        except ImportError:
            return False
        for source, destination in (
            (self.telemetry_path, self.partial_dir / "telemetry.parquet"),
            (self.events_path, self.partial_dir / "events.parquet"),
        ):
            rows = list(self._jsonl_records(source))
            if rows:
                pq.write_table(pa.Table.from_pylist(rows), destination,
                               compression="zstd")
        return True

    def finalize(self, status: str, reason: str = "",
                 validation: dict[str, Any] | None = None) -> Path:
        if self.closed:
            return self.final_dir if self.final_dir.exists() else self.partial_dir
        self._close_streams()
        parquet_written = self._write_parquet_if_available()
        self.manifest.update({
            "status": status,
            "reason": reason,
            "completed_utc": datetime.now(timezone.utc).isoformat(),
            "samples_received": self.samples_received,
            "samples_invalid": self.samples_invalid,
            "parquet_written": parquet_written,
        })
        _write_json(self.partial_dir / "session.json", self.manifest)
        _write_json(self.partial_dir / "validation.json", validation or {})
        checksums = {
            path.name: _sha256(path)
            for path in sorted(self.partial_dir.iterdir())
            if path.is_file() and path.name != "checksums.json"
        }
        _write_json(self.partial_dir / "checksums.json", checksums)
        self.final_dir.parent.mkdir(parents=True, exist_ok=True)
        os.replace(self.partial_dir, self.final_dir)
        self.closed = True
        return self.final_dir
