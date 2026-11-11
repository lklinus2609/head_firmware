"""Tests for the viseme tool (viseme_tuner.py): targets from photos, the
pose sequencer, the search and the library, on the simulated board."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace

sys.modules.setdefault("serial", SimpleNamespace(Serial=object))
FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))

import numpy  # noqa: E402

import face_stream as fs  # noqa: E402
import mouth_actions as ma  # noqa: E402
import mouth_shape as ms  # noqa: E402
import viseme_tuner as vt  # noqa: E402
import workspace_guard as wg  # noqa: E402
from mouth_shape_test import Connections  # noqa: E402


def synthetic_targets(records: Path, goals: dict) -> None:
    """A targets file whose visemes are the simulated mouth's measures of the
    given poses (so the search has an exact answer)."""
    shape = vt.SimulatedMeasurer.shape_of
    rest = shape(ma.rest())
    calibration = [shape(p) for _n, p in vt.calibration_poses(True)]
    span = ms.spans(rest, calibration)
    data = {"format": vt.TARGETS_FORMAT, "rest": rest.as_dict(), "spans": span,
            "visemes": {name: {"normalized": ms.normalized(shape(pose), rest, span),
                               "mouth_box": [0, 0, 10, 10]} for name, pose in goals.items()}}
    config = records / "preview" / "config"
    config.mkdir(parents=True, exist_ok=True)
    (config / vt.TARGETS_FILE).write_text(json.dumps(data))


def test_targets_from_photos() -> None:
    from PIL import Image
    folder = Path(tempfile.mkdtemp())
    for name in ("rest", "m", "o"):
        Image.new("RGB", (64, 64), (128, 128, 128)).save(folder / f"{name}.jpg")
    opening = {"rest": 20.0, "m": 18.0, "o": 40.0}
    width = {"rest": 100.0, "m": 104.0, "o": 70.0}
    calls = []

    def fake_finder(rgb):
        name = ("rest", "m", "o")[len(calls)]
        calls.append(name)
        p = numpy.zeros((478, 2))
        p[[33, 133]] = [[90, 100], [110, 100]]
        p[[362, 263]] = [[290, 100], [310, 100]]
        p[ms.LIP_CORNER_RIGHT] = [200 - width[name] / 2, 300]
        p[ms.LIP_CORNER_LEFT] = [200 + width[name] / 2, 300]
        p[ms.OUTER_UPPER_CENTRE], p[ms.OUTER_LOWER_CENTRE] = [200, 290], [200, 290 + opening[name]]
        p[ms.INNER_UPPER_CENTRE], p[ms.INNER_LOWER_CENTRE] = [200, 300], [200, 300 + opening[name] - 18]
        return p
    photos = {"rest": folder / "rest.jpg", "m": folder / "m.jpg", "o": folder / "o.jpg"}
    data = vt.build_targets(photos, fake_finder, folder / "crops", connections=Connections)
    assert set(data["visemes"]) == {"m", "o"} and (folder / "crops" / "o.jpg").exists()
    o, m = data["visemes"]["o"]["normalized"], data["visemes"]["m"]["normalized"]
    assert abs(o["width"] + 1) < 1e-9 and abs(o["opening"] - 1) < 1e-9 and abs(o["gap"] - 1) < 1e-9
    assert m["width"] > 0 and m["opening"] < 0
    assert vt.find_photos(folder) == photos
    (folder / "rest.jpg").unlink()
    try:
        vt.find_photos(folder)
    except ValueError as error:
        assert "rest" in str(error)
    else:
        raise AssertionError("no rest photo accepted")


def test_search_planner_finds_the_simulated_goal() -> None:
    goal = vt.search_vector({"jaw": 0.5, "d": 0.5, "lip": 0.25})
    shape = vt.SimulatedMeasurer.shape_of
    rest = shape(ma.rest())
    span = ms.spans(rest, [shape(p) for _n, p in vt.calibration_poses(True)])
    target = ms.normalized(shape(goal), rest, span)
    score = lambda m: ms.score(ms.normalized(m.shape, rest, span), target)[0]  # noqa: E731
    planner = vt.SearchPlanner(score, ma.rest(), max_trials=80)
    measurer = vt.SimulatedMeasurer(noise=0.0)
    step = planner.first()
    while step is not None:
        label, pose = step
        step = planner.observe(label, pose, measurer.measure(pose))
    assert planner.best_score < 1e-6, planner.best_score
    best = vt.search_values(planner.best_pose)
    assert abs(best["jaw"] - 0.5) < 1e-6 and abs(best["d"] - 0.5) < 1e-6 and abs(best["lip"] - 0.25) < 1e-6
    assert planner.trials <= 80


def test_search_respects_bounds_and_free_values() -> None:
    planner = vt.SearchPlanner(lambda m: 0.0, vt.search_vector({"jaw": 1.0}), free=("jaw",), max_trials=10)
    step = planner.first()
    seen = []
    while step is not None:
        seen.append(step[1])
        step = planner.observe(step[0], step[1], vt.Measurement(None))
    assert all(0.0 <= p["jaw"] <= 1.0 and p["right_d"] == 0.0 for p in seen)
    try:
        vt.SearchPlanner(lambda m: 0.0, ma.rest(), free=("brow",))
    except ValueError:
        pass
    else:
        raise AssertionError("unknown free value accepted")


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def test_sequencer_waits_for_settled_targets_and_rest_approach() -> None:
    clock = Clock()
    measured = []
    poses = [("a", vt.search_vector({"jaw": 1.0})), ("b", vt.search_vector({"d": 1.0}))]
    sequencer = vt.PoseSequencer(vt.ListPlanner(poses), vt.SimulatedMeasurer(),
                                 lambda label, pose, m, info: measured.append((label, info)),
                                 approach="rest", clock=clock)
    moving = {i: 0 for i in fs.COMMAND_LOG_SERVOS}
    t = 0.0
    for step in range(2000):
        actions = sequencer(t)
        if actions is None:
            break
        targets = dict(moving)
        if step % 40 < 20:                      # targets still moving for 20 updates per pose
            targets[7] = step
        sequencer.on_command(t, actions[0], actions[1], targets)
        sequencer.on_update(t)
        t += 0.02
        clock.now = t
    assert [label for label, _ in measured] == ["a", "b"]
    assert all(info["settled"] for _l, info in measured)
    # "b" went through rest first: it was reached later than "a" after its own start.
    assert measured[1][1]["held_s"] >= vt.DWELL_S


def test_sequencer_measures_after_timeout() -> None:
    clock = Clock()
    measured = []
    sequencer = vt.PoseSequencer(vt.ListPlanner([("a", vt.search_vector({"jaw": 1.0}))]),
                                 vt.SimulatedMeasurer(), lambda *args: measured.append(args[3]),
                                 clock=clock, timeout_s=1.0)
    t = 0.0
    for step in range(500):
        actions = sequencer(t)
        if actions is None:
            break
        sequencer.on_command(t, actions[0], actions[1], {7: step})       # never settles
        sequencer.on_update(t)
        t += 0.02
        clock.now = t
    assert len(measured) == 1 and measured[0]["settled"] is False and measured[0]["held_s"] > 1.0


def test_library_round_trip() -> None:
    config = Path(tempfile.mkdtemp())
    assert vt.load_library(config)["visemes"] == {}
    vt.save_to_library(config, "m", vt.search_vector({"lip": 0.7}), score=0.01)
    vt.save_to_library(config, "o", vt.search_vector({"jaw": 0.6, "d": 0.0}))
    library = vt.load_library(config)
    assert library["visemes"]["m"]["pose"]["lip_left"] == 0.7 and library["visemes"]["m"]["score"] == 0.01
    assert set(library["visemes"]) == {"m", "o"}


def test_search_on_the_simulated_board_saves_the_best_pose() -> None:
    records = Path(tempfile.mkdtemp())
    synthetic_targets(records, {"o": vt.search_vector({"jaw": 0.5, "d": 0.25})})
    try:
        code = vt.main(["search", "--preview", "--records", str(records), "--viseme", "o",
                        "--max-trials", "40", "--save", "--free", "jaw,d,v,lip"])
        assert code == 0
        library = vt.load_library(records / "preview" / "config")
        best = vt.search_values(library["visemes"]["o"]["pose"])
        assert abs(best["jaw"] - 0.5) <= 0.13 and abs(best["d"] - 0.25) <= 0.13, best
        trials = json.loads(next((records / "preview" / "runs").glob("*/*/" + vt.TRIALS_FILE)).read_text())
        assert sum(1 for t in trials["trials"] if t.get("calibration")) == len(vt.calibration_poses(True))
        assert trials["best"]["score"] < 0.05
        assert vt.main(["try", "--preview", "--records", str(records), "--viseme", "o",
                        "--pose", "jaw=0.5 right_d=0.25 left_d=0.25"]) == 0
        assert vt.main(["try", "--preview", "--records", str(records), "--viseme", "nope",
                        "--pose", "jaw=0.5"]) == 2
    finally:
        wg.clear_upper_lips()


def main() -> None:
    test_targets_from_photos()
    test_search_planner_finds_the_simulated_goal()
    test_search_respects_bounds_and_free_values()
    test_sequencer_waits_for_settled_targets_and_rest_approach()
    test_sequencer_measures_after_timeout()
    test_library_round_trip()
    test_search_on_the_simulated_board_saves_the_best_pose()
    print("viseme tuner: all tests passed")


if __name__ == "__main__":
    main()
