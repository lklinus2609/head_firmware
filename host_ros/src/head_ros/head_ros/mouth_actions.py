"""The mouth action vector and the mouth trajectory CSV file.

Standard library only: head-face, the viseme tool, the babbling generator and
the lip-sync training code (lipsync/) all import this module.

A mouth action vector has these values, all relative to the session neutral
(face_stream / workspace_guard turn them into ticks):

    jaw              0 closed .. 1 open (the session's open limit)
    right_d, left_d  modiolus diag, 0 .. 1
    right_v, left_v  modiolus up (> 0) or down (< 0), -1 .. 1
    lip_right/left   lower lip 3 / 4, 0 .. 1 of its range
    upper_lip_right/left  upper lip 13 / 18, 0 .. 1 of its range
                     (driven only when configured from the panel marks)

A trajectory file is a CSV with '#' metadata lines first:

    # format=head-mouth-trajectory-1
    # audio_start_s=0.500
    t_s,jaw,right_d,right_v,left_d,left_v,lip_right,lip_left,upper_lip_right,upper_lip_left
    0.000,0,0,0,0,0,0,0,0,0
    0.020,0.1,...

Times are seconds from the start of streaming, strictly increasing, the
first at 0 or later. A column left out is 0. Between rows the values are
interpolated linearly; after the last row the script ends.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

MOUTH_COLUMNS = ("jaw", "right_d", "right_v", "left_d", "left_v",
                 "lip_right", "lip_left", "upper_lip_right", "upper_lip_left")
BOUNDS = {"jaw": (0.0, 1.0), "right_d": (0.0, 1.0), "right_v": (-1.0, 1.0),
          "left_d": (0.0, 1.0), "left_v": (-1.0, 1.0), "lip_right": (0.0, 1.0),
          "lip_left": (0.0, 1.0), "upper_lip_right": (0.0, 1.0), "upper_lip_left": (0.0, 1.0)}
TRAJECTORY_FORMAT = "head-mouth-trajectory-1"
TIME_COLUMN = "t_s"
# A trajectory longer than this is refused (a typo in a generator, not speech).
MAX_DURATION_S = 900.0
# The face_stream action names (workspace_guard roles) for the paired values.
UPPER_LIP_ACTION = "upper_lip"

Vector = Dict[str, float]
Actions = Dict[str, Tuple[float, float]]


def rest() -> Vector:
    return {name: 0.0 for name in MOUTH_COLUMNS}


def check_vector(values: Vector) -> Vector:
    """A full vector (missing names are 0); ValueError when a value is not
    finite, is outside its bounds, or the name is unknown."""
    unknown = sorted(set(values) - set(MOUTH_COLUMNS))
    if unknown:
        raise ValueError(f"unknown mouth action {unknown}; known: {', '.join(MOUTH_COLUMNS)}")
    out = rest()
    for name, value in values.items():
        value = float(value)
        low, high = BOUNDS[name]
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{name}={value} is outside {low}..{high}")
        out[name] = value
    return out


def clip_vector(values: Vector) -> Vector:
    """Every value moved inside its bounds (for generators and searches)."""
    out = rest()
    for name in MOUTH_COLUMNS:
        low, high = BOUNDS[name]
        out[name] = min(high, max(low, float(values.get(name, 0.0))))
    return out


def to_actions(values: Vector) -> Tuple[Actions, float]:
    """(face_stream actions, jaw j) for a vector."""
    v = check_vector(values)
    actions: Actions = {"right": (v["right_d"], v["right_v"]), "left": (v["left_d"], v["left_v"])}
    if v["lip_right"] or v["lip_left"]:
        actions["lip"] = (v["lip_right"], v["lip_left"])
    if v["upper_lip_right"] or v["upper_lip_left"]:
        actions[UPPER_LIP_ACTION] = (v["upper_lip_right"], v["upper_lip_left"])
    return actions, v["jaw"]


def from_actions(actions: Actions, j: float) -> Vector:
    """The mouth vector of face_stream actions (brow and oculi are left out)."""
    right = actions.get("right", (0.0, 0.0))
    left = actions.get("left", (0.0, 0.0))
    lip = actions.get("lip", (0.0, 0.0))
    upper = actions.get(UPPER_LIP_ACTION, (0.0, 0.0))
    return {"jaw": float(j), "right_d": right[0], "right_v": right[1], "left_d": left[0],
            "left_v": left[1], "lip_right": lip[0], "lip_left": lip[1],
            "upper_lip_right": upper[0], "upper_lip_left": upper[1]}


def parse_pose(text: str) -> Vector:
    """'jaw=0.3 right_d=0.5 lip_left=1' (missing names are 0)."""
    values: Vector = {}
    for item in text.split():
        name, _, value = item.partition("=")
        if not value:
            raise ValueError(f"bad pose part {item!r}")
        values[name] = float(value)
    return check_vector(values)


def format_pose(values: Vector) -> str:
    return " ".join(f"{name}={values[name]:.3f}" for name in MOUTH_COLUMNS if values.get(name))


# Trajectory files ------------------------------------------------------------------
Row = Tuple[float, Vector]


def write_trajectory(path: Path, rows: Sequence[Row], metadata: Optional[Dict[str, str]] = None) -> Path:
    check_rows(rows)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    meta = {"format": TRAJECTORY_FORMAT, **{k: str(v) for k, v in (metadata or {}).items()}}
    with open(path, "w", encoding="utf-8", newline="") as stream:
        for key, value in meta.items():
            if "\n" in key or "\n" in value or "=" in key:
                raise ValueError(f"bad metadata {key!r}={value!r}")
            stream.write(f"# {key}={value}\n")
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow((TIME_COLUMN, *MOUTH_COLUMNS))
        for t_s, values in rows:
            full = check_vector(values)
            writer.writerow((f"{t_s:.4f}", *(f"{full[name]:.5f}" for name in MOUTH_COLUMNS)))
    return path


def read_trajectory(path: Path) -> Tuple[List[Row], Dict[str, str]]:
    """(rows, metadata). ValueError on any format problem."""
    metadata: Dict[str, str] = {}
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    body = []
    for line in lines:
        if line.startswith("#"):
            key, sep, value = line[1:].strip().partition("=")
            if not sep:
                raise ValueError(f"{path}: bad metadata line {line!r}")
            metadata[key.strip()] = value.strip()
        elif line.strip():
            body.append(line)
    if metadata.get("format") != TRAJECTORY_FORMAT:
        raise ValueError(f"{path}: not a {TRAJECTORY_FORMAT} file (format={metadata.get('format')!r})")
    reader = csv.reader(body)
    header = next(reader, None)
    if not header or header[0] != TIME_COLUMN:
        raise ValueError(f"{path}: the first column must be {TIME_COLUMN}")
    names = header[1:]
    unknown = sorted(set(names) - set(MOUTH_COLUMNS))
    if unknown or len(set(names)) != len(names):
        raise ValueError(f"{path}: bad columns {header}")
    rows: List[Row] = []
    for number, record in enumerate(reader, start=2):
        if len(record) != len(header):
            raise ValueError(f"{path}: row {number} has {len(record)} values, not {len(header)}")
        try:
            t_s = float(record[0])
            values = check_vector({name: float(value) for name, value in zip(names, record[1:])})
        except ValueError as error:
            raise ValueError(f"{path}: row {number}: {error}") from error
        rows.append((t_s, values))
    check_rows(rows)
    return rows, metadata


def check_rows(rows: Sequence[Row]) -> None:
    if not rows:
        raise ValueError("a trajectory needs at least one row")
    last = -math.inf
    for t_s, _values in rows:
        if not math.isfinite(t_s) or t_s < 0.0 or t_s <= last:
            raise ValueError(f"trajectory times must start at 0 or later and increase (t_s={t_s})")
        last = t_s
    if last > MAX_DURATION_S:
        raise ValueError(f"trajectory is {last:.0f} s long, over {MAX_DURATION_S:.0f} s")


def sample(rows: Sequence[Row], t_s: float) -> Optional[Vector]:
    """The vector at t_s (linear between rows; the first row before it);
    None after the last row."""
    if t_s > rows[-1][0]:
        return None
    if t_s <= rows[0][0]:
        return dict(rows[0][1])
    low, high = 0, len(rows) - 1
    while high - low > 1:                      # rows[low][0] < t_s <= rows[high][0]
        middle = (low + high) // 2
        if rows[middle][0] < t_s:
            low = middle
        else:
            high = middle
    (t0, a), (t1, b) = rows[low], rows[high]
    u = (t_s - t0) / (t1 - t0)
    return {name: a[name] + (b[name] - a[name]) * u for name in MOUTH_COLUMNS}


def trajectory_script(rows: Sequence[Row]) -> Callable[[float], Optional[Tuple[Actions, float]]]:
    """A face_stream Script for the rows (total_s and an empty marks list set)."""
    rows = list(rows)
    check_rows(rows)

    def script(t_s: float):
        values = sample(rows, t_s)
        return None if values is None else to_actions(values)
    script.total_s = rows[-1][0]               # type: ignore[attr-defined]
    script.marks = []                          # type: ignore[attr-defined]
    return script
