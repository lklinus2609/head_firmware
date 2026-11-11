"""Tests for the mouth action vector and the trajectory file (mouth_actions.py)."""

from __future__ import annotations

from pathlib import Path
import sys
import tempfile

FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))

import mouth_actions as ma  # noqa: E402


def expect_error(function, *args) -> str:
    try:
        function(*args)
    except ValueError as error:
        return str(error)
    raise AssertionError(f"{function.__name__}{args} did not raise")


def test_vector_checks_and_actions_round_trip() -> None:
    v = ma.check_vector({"jaw": 0.4, "right_v": -0.5, "lip_left": 1.0, "upper_lip_right": 0.2})
    assert set(v) == set(ma.MOUTH_COLUMNS) and v["left_d"] == 0.0
    actions, j = ma.to_actions(v)
    assert j == 0.4 and actions["right"] == (0.0, -0.5) and actions["lip"] == (0.0, 1.0)
    assert actions["upper_lip"] == (0.2, 0.0)
    assert ma.from_actions(actions, j) == v
    rest_actions, _ = ma.to_actions(ma.rest())
    assert "lip" not in rest_actions and "upper_lip" not in rest_actions
    assert "outside" in expect_error(ma.check_vector, {"jaw": 1.5})
    assert "outside" in expect_error(ma.check_vector, {"right_d": -0.1})
    assert "unknown" in expect_error(ma.check_vector, {"brow": 0.5})
    assert "outside" in expect_error(ma.check_vector, {"jaw": float("nan")})
    assert ma.clip_vector({"jaw": 2.0, "right_v": -3.0})["jaw"] == 1.0
    assert ma.parse_pose(ma.format_pose(v)) == {k: round(x, 3) for k, x in v.items()}


def test_trajectory_file_round_trip_and_sampling() -> None:
    rows = [(0.0, ma.rest()), (0.5, ma.check_vector({"jaw": 1.0})), (1.0, ma.rest())]
    path = Path(tempfile.mkdtemp()) / "t.csv"
    ma.write_trajectory(path, rows, {"audio_start_s": "0.25"})
    read, meta = ma.read_trajectory(path)
    assert meta["format"] == ma.TRAJECTORY_FORMAT and meta["audio_start_s"] == "0.25"
    assert [t for t, _ in read] == [0.0, 0.5, 1.0]
    assert abs(ma.sample(read, 0.25)["jaw"] - 0.5) < 1e-9
    assert ma.sample(read, 1.01) is None
    script = ma.trajectory_script(read)
    assert script.total_s == 1.0 and script.marks == []
    actions, j = script(0.75)
    assert abs(j - 0.5) < 1e-9 and actions["right"] == (0.0, 0.0)
    assert script(1.5) is None


def test_trajectory_columns_may_be_left_out() -> None:
    path = Path(tempfile.mkdtemp()) / "short.csv"
    path.write_text(f"# format={ma.TRAJECTORY_FORMAT}\nt_s,jaw\n0,0\n0.1,0.3\n", encoding="utf-8")
    rows, _ = ma.read_trajectory(path)
    assert rows[1][1]["jaw"] == 0.3 and rows[1][1]["lip_left"] == 0.0


def test_bad_trajectories_are_refused() -> None:
    folder = Path(tempfile.mkdtemp())
    cases = {
        "no_format.csv": "t_s,jaw\n0,0\n",
        "bad_column.csv": f"# format={ma.TRAJECTORY_FORMAT}\nt_s,brow\n0,0\n",
        "not_increasing.csv": f"# format={ma.TRAJECTORY_FORMAT}\nt_s,jaw\n0,0\n0,0.1\n",
        "negative.csv": f"# format={ma.TRAJECTORY_FORMAT}\nt_s,jaw\n-0.1,0\n",
        "out_of_bounds.csv": f"# format={ma.TRAJECTORY_FORMAT}\nt_s,jaw\n0,1.2\n",
        "short_row.csv": f"# format={ma.TRAJECTORY_FORMAT}\nt_s,jaw,lip_left\n0,0\n",
        "empty.csv": f"# format={ma.TRAJECTORY_FORMAT}\nt_s,jaw\n",
        "too_long.csv": f"# format={ma.TRAJECTORY_FORMAT}\nt_s,jaw\n0,0\n{ma.MAX_DURATION_S + 1},0\n",
    }
    for name, text in cases.items():
        path = folder / name
        path.write_text(text, encoding="utf-8")
        expect_error(ma.read_trajectory, path)


def main() -> None:
    test_vector_checks_and_actions_round_trip()
    test_trajectory_file_round_trip_and_sampling()
    test_trajectory_columns_may_be_left_out()
    test_bad_trajectories_are_refused()
    print("mouth actions: all tests passed")


if __name__ == "__main__":
    main()
