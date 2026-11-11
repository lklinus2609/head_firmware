"""Tests for the ML face workspace layer (face_workspace.py)."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace

sys.modules.setdefault("serial", SimpleNamespace(Serial=object))
FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))

import face_workspace as fw  # noqa: E402
import natural_face as nf  # noqa: E402


def test_spec_round_trip_and_flags() -> None:
    spec = fw.WorkspaceSpec()
    path = Path(tempfile.mkdtemp()) / "ws.json"
    path.write_text(json.dumps(spec.to_json()))
    back = fw.WorkspaceSpec.from_json(json.loads(path.read_text()))
    assert back == spec
    assert spec.release_args() == ["--jaw-release", "8=30", "--jaw-release", "9=15"]
    try:
        fw.WorkspaceSpec.from_json({"format": "other"})
        raise AssertionError("wrong format accepted")
    except ValueError:
        pass


def test_project_clips_every_dimension() -> None:
    budget = fw.JawBudget(fw.WorkspaceSpec())
    out = fw.project({"right": (1.5, -2.0), "left": (-0.2, 0.5), "gaze": (3.0, -3.0),
                      "lids": (1.2, -0.1), "brow": (0.5, 2.0), "jaw": 0.2}, budget, 0.0)
    assert out["right"] == (1.0, -1.0) and out["left"] == (0.0, 0.5)
    assert out["gaze"] == (1.0, -1.0) and out["lids"] == (1.0, 0.0) and out["brow"] == (0.5, 1.0)
    assert out["jaw"] == 0.2
    assert fw.project({}, budget, 0.02)["jaw"] == 0.0
    try:
        fw.project({"jawz": 1.0}, budget, 0.04)
        raise AssertionError("unknown key accepted")
    except ValueError:
        pass


def test_jaw_budget_allows_pulses_not_holds() -> None:
    spec = fw.WorkspaceSpec()
    budget = fw.JawBudget(spec)
    dt = 0.02
    held = [budget.limit(1.0, n * dt) for n in range(100)]          # ask 1.0 for 2 s
    over = [j for j in held if j > spec.jaw_static_max]
    assert len(over) * dt <= spec.jaw_pulse_max_s + dt                # one pulse only
    assert held[-1] == spec.jaw_static_max
    # After coming down below the static limit, the next pulse is allowed.
    t = 100 * dt
    for n in range(20):
        budget.limit(0.0, t + n * dt)
    assert budget.limit(1.0, t + 21 * dt) == 1.0


def test_jaw_budget_keeps_the_tested_talking_and_caps_more() -> None:
    """The talking that ran on the head (seeds 7-9, 120 s) passes almost
    unchanged; continuous fast syllables are cut to the duty limit."""
    spec = fw.WorkspaceSpec()
    for seed in (7, 8, 9):
        plan = nf.NaturalPlan.make(nf.NaturalSettings(seconds=120.0, seed=seed))
        budget = fw.JawBudget(spec)
        changed = sum(abs(budget.limit(plan.jaw(n / 50), n / 50) - plan.jaw(n / 50)) > 1e-9
                      for n in range(6000))
        assert changed / 6000 < 0.02, (seed, changed)
    budget = fw.JawBudget(spec)
    flags = []
    for n in range(1500):                                             # 30 s of 0.2 s pulses, no rest
        t = n / 50
        flags.append(budget.limit(1.0 if (t % 0.3) < 0.2 else 0.0, t) > spec.jaw_static_max)
    window = int(spec.jaw_window_s * 50)
    worst = max(sum(flags[i:i + window]) for i in range(0, len(flags) - window))
    assert worst / window <= spec.jaw_duty_max + 0.03, worst / window


def main() -> None:
    test_spec_round_trip_and_flags()
    test_project_clips_every_dimension()
    test_jaw_budget_allows_pulses_not_holds()
    test_jaw_budget_keeps_the_tested_talking_and_caps_more()
    print("face workspace: all tests passed")


if __name__ == "__main__":
    main()
