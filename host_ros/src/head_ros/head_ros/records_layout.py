"""Where the tools keep their files in the records folder (tuning_runs/).

    tuning_runs/
      STOP                         stop file (the tools check it while they run)
      config/                      servo_notes.json, jaw_sweep_settings.json,
                                   face_markers.json, face_reference.jpg
      maps/                        workspace maps and the runs they came from
      panel/                       tuning-panel records: <date>_<time>_tuning.json
                                   (+ .telemetry.csv); Home reads its marks here
      videos/                      kept videos and pictures, named by hand
      runs/<date>/<time>_<kind>[_<label>]/
                                   one folder per tool run, plain names inside:
                                   record.json, telemetry.csv, video.webm,
                                   video.frames.csv, stills/, neutrals.json,
                                   commands.csv (head-face, head-viseme),
                                   viseme_trials.json, composites/ (head-viseme), ...

<date> is YYYY-MM-DD and <time> HHMMSS, local time (the operator's clock);
times inside the files stay UTC. Organized this way on 2026-10-23 (before,
every file sat flat in tuning_runs/ under a UTC stamp).
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import re
from typing import Optional

CONFIG, MAPS, PANEL, VIDEOS, RUNS = "config", "maps", "panel", "videos", "runs"
# The file names inside a run folder.
RECORD = "record.json"
TELEMETRY = "telemetry.csv"
VIDEO = "video.mp4"            # converted to video.webm when the run ends
STILLS = "stills"
NEUTRALS = "neutrals.json"


def _folder(records: Path, name: str) -> Path:
    path = Path(records) / name
    path.mkdir(parents=True, exist_ok=True)
    return path


def config_dir(records: Path) -> Path:
    return _folder(records, CONFIG)


def panel_dir(records: Path) -> Path:
    return _folder(records, PANEL)


def local_stamp(now: Optional[datetime] = None) -> str:
    """YYYY-MM-DD_HHMMSS in local time."""
    return (now or datetime.now()).strftime("%Y-%m-%d_%H%M%S")


def run_dir(records: Path, kind: str, label: str = "", now: Optional[datetime] = None) -> Path:
    """A new folder runs/<date>/<time>_<kind>[_<label>] for one run."""
    now = now or datetime.now()
    name = f"{now:%H%M%S}_{kind}" + (f"_{slug(label)}" if label else "")
    path = Path(records) / RUNS / f"{now:%Y-%m-%d}" / name
    number = 2
    while path.exists():                       # two runs in one second
        path = path.with_name(f"{name}-{number}")
        number += 1
    path.mkdir(parents=True)
    return path


def slug(text: str) -> str:
    """A label safe for a folder name: lower case, words joined by '-'."""
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")[:40]


def newest(records: Path, file_name: str) -> Optional[Path]:
    """The newest runs/*/*/file_name (folders sort by date and time)."""
    found = sorted((Path(records) / RUNS).glob(f"*/*/{file_name}"))
    return found[-1] if found else None
