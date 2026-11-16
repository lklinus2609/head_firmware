"""Tests for mouth_depth.py (schedule and the slit finder)."""

from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace

sys.modules.setdefault("serial", SimpleNamespace(Serial=object))
FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))

import numpy  # noqa: E402
import mouth_depth as md  # noqa: E402


def test_schedule_alternates_and_ends_closed() -> None:
    plan = md.schedule()
    kinds = [k for _s, _e, k in plan]
    assert kinds == ["closed", "open"] * md.HOLDS + ["closed"]
    assert all(abs(b - a - (md.OPEN_S if k == "open" else md.CLOSED_S)) < 1e-9 for a, b, k in plan)
    assert all(plan[i][1] == plan[i + 1][0] for i in range(len(plan) - 1))


def test_find_slit_and_deproject() -> None:
    rgb = numpy.full((100, 60, 3), 180, numpy.uint8)
    rgb[40:52, 20:40] = 20                                  # a dark slit, rows 40-51
    assert md.find_slit(rgb, 30, 10, 90) == (40, 51)
    assert md.find_slit(numpy.full((100, 60, 3), 180, numpy.uint8), 30, 10, 90) is None
    intr = (1280, 720, 640.0, 360.0, 900.0, 900.0)
    x, y, z = md.deproject(intr, 640 + 90, 360 - 45, 0.3)
    assert abs(x - 0.03) < 1e-12 and abs(y + 0.015) < 1e-12 and z == 0.3


if __name__ == "__main__":
    for name, test in sorted(globals().items()):
        if name.startswith("test_") and callable(test):
            test()
            print(f"ok {name}")
