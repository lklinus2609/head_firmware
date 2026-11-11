"""Tests for the babbling trajectory generator (babble.py)."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace

sys.modules.setdefault("serial", SimpleNamespace(Serial=object))
FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))

import babble  # noqa: E402
import mouth_actions as ma  # noqa: E402

LIBRARY = {"m": ma.check_vector({"lip_right": 0.8, "lip_left": 0.8}),
           "o": ma.check_vector({"jaw": 0.8, "right_d": 0.0, "right_v": 0.2, "left_v": 0.2})}


def test_rows_are_valid_paced_and_deterministic() -> None:
    settings = babble.BabbleSettings(minutes=1.0, seed=3, block_s=20, pause_s=5,
                                     line_speed_tps=1200, jaw_speed_tps=480)
    rows = babble.generate(LIBRARY, settings)
    ma.check_rows(rows)
    assert rows[0][1] == ma.rest() and rows[-1][1] == ma.rest()
    assert all(abs((b[0] - a[0]) - babble.STEP_S) < 1e-6 for a, b in zip(rows, rows[1:]))
    # Motion time is one minute; three 5 s pauses and the rest at both ends come on top.
    assert 60 + 2 * 5 <= rows[-1][0] <= 60 + 3 * 5 + 10
    for a, b in zip(rows, rows[1:]):
        for name in ma.MOUTH_COLUMNS:
            speed = settings.jaw_speed_tps if name == "jaw" else settings.line_speed_tps
            ticks = abs(b[1][name] - a[1][name]) * babble.DEFAULT_UNIT_TICKS[name]
            assert ticks <= speed * babble.STEP_S + 1e-6, (name, a, b)
    assert all(v["upper_lip_right"] == 0 and v["upper_lip_left"] == 0 for _t, v in rows)
    assert babble.generate(LIBRARY, settings) == rows
    other = babble.generate(LIBRARY, babble.BabbleSettings(minutes=1.0, seed=4, block_s=20, pause_s=5,
                                                           line_speed_tps=1200, jaw_speed_tps=480))
    assert other != rows


def test_jaw_duty_is_kept() -> None:
    always_open = {"o": ma.check_vector({"jaw": 1.0})}
    settings = babble.BabbleSettings(minutes=1.0, seed=1, sigma=0.0, block_s=30, pause_s=2, jaw_duty=0.3,
                                     line_speed_tps=1200, jaw_speed_tps=480, rest_weight=0.0)
    rows = babble.generate(always_open, settings)
    assert babble.jaw_open_fraction(rows) < 0.45, babble.jaw_open_fraction(rows)
    loose = babble.generate(always_open, babble.BabbleSettings(**{**settings.__dict__, "jaw_duty": 1.0}))
    assert babble.jaw_open_fraction(loose) > babble.jaw_open_fraction(rows)


def test_bad_settings_and_command_line() -> None:
    for change in ({"minutes": 0}, {"sigma": -1}, {"dwell_s": (0.3, 0.1)}, {"jaw_duty": 0}):
        try:
            babble.generate(LIBRARY, babble.BabbleSettings(**change))
        except ValueError:
            continue
        raise AssertionError(f"{change} accepted")
    folder = Path(tempfile.mkdtemp())
    library = folder / "library.json"
    library.write_text(json.dumps({"format": "head-viseme-library-1",
                                   "visemes": {k: {"pose": v} for k, v in LIBRARY.items()}}))
    out = folder / "b.csv"
    assert babble.main(["--library", str(library), "--out", str(out), "--minutes", "0.2",
                        "--line-speed", "1200", "--jaw-speed", "480"]) == 0
    rows, meta = ma.read_trajectory(out)
    assert meta["made_by"] == "babble" and rows[-1][1] == ma.rest()
    assert babble.main(["--library", str(folder / "none.json"), "--out", str(out)]) == 2


def main() -> None:
    test_rows_are_valid_paced_and_deterministic()
    test_jaw_duty_is_kept()
    test_bad_settings_and_command_line()
    print("babble: all tests passed")


if __name__ == "__main__":
    main()
