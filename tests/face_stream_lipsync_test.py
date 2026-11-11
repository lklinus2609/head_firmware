"""Tests for the lip-sync additions to the face session (face_stream.py,
workspace_guard.py): upper lips from the panel marks, trajectory input, the
command log, audio start and the camera mode."""

from __future__ import annotations

import csv
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace

sys.modules.setdefault("serial", SimpleNamespace(Serial=object))
FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))

import face_stream as fs  # noqa: E402
import mouth_actions as ma  # noqa: E402
import workspace_guard as wg  # noqa: E402
from face_stream_test import session  # noqa: E402


def marks(neutral: int, *ends: int) -> dict:
    out = {"neutral": {"present_tick": neutral}}
    for number, end in enumerate(ends, start=1):
        out[f"end_{number}"] = {"present_tick": end}
    return out


def test_upper_lips_come_only_from_both_marks() -> None:
    pulls, span, notes = wg.upper_lips_from_marks({13: marks(1000, 1300), 18: marks(3000, 2600)})
    assert pulls == {13: 1, 18: -1} and span == 400 and notes == []
    pulls, span, notes = wg.upper_lips_from_marks({13: marks(1000, 1300)})
    assert pulls == {} and span == 0 and any("18" in n for n in notes)
    pulls, _span, notes = wg.upper_lips_from_marks({13: marks(1000, 1300, 700), 18: marks(3000, 2600)})
    assert pulls == {} and any("direction" in n for n in notes)
    pulls, _span, _notes = wg.upper_lips_from_marks(None)
    assert pulls == {}


def test_configure_and_clear_upper_lips() -> None:
    before = (dict(wg.UPPER_PULL), dict(wg.ALL_PULL), dict(wg.UPPER_RANGES))
    try:
        wg.configure_upper_lips({13: 1, 18: -1}, 400)
        assert wg.upper_lips_configured() and wg.UPPER_PULL[18] == -1 and wg.ALL_PULL[13] == 1
        assert wg.UPPER_SIDES["right"]["upper_lip"] == 13 and wg.UPPER_RANGES["upper_lip"] == 400
        guard = wg.WorkspaceGuard({8: 2300, 11: 2700, 14: 3100, 9: 1750, 15: 450, 19: 2750},
                                  upper_neutral={13: 1000, 18: 3000}, upper_spans={13: 300})
        targets, drivers = guard.upper_command({"upper_lip": (1.0, 0.5)})
        assert targets == {13: 1300, 18: 2800} and set(drivers) == {13, 18}
        for bad in ({13: 1}, {13: 1, 18: 0}):
            try:
                wg.configure_upper_lips(bad, 400)
            except ValueError:
                continue
            raise AssertionError(f"{bad} accepted")
    finally:
        wg.clear_upper_lips()
    assert (dict(wg.UPPER_PULL), dict(wg.ALL_PULL), dict(wg.UPPER_RANGES)) == before
    assert not wg.upper_lips_configured()


def test_upper_lips_stop_at_their_marks_on_the_simulated_board() -> None:
    try:
        wg.configure_upper_lips({13: 1, 18: -1}, 400)
        panel = {13: marks(2481, 2481 + 250), 18: marks(2666, 2666 - 400)}
        clock, board, face = session(panel_marks=panel)
        face.start()
        script = fs.keyframe_script([fs.parse_keyframe("hold=3 ulip=1,1"), fs.parse_keyframe("hold=0.5 ulip=1,1")])
        sent = []
        face.on_command = lambda t, actions, j, targets: sent.append(dict(targets))
        assert face.run(script, sleep=lambda _s: clock.advance(0.005)) == "finished"
        assert max(t[13] for t in sent) == 2481 + 250          # its own end mark, under the 400 range
        assert min(t[18] for t in sent) == 2666 - 400
        assert board.torque_state == fs.headctl.TORQUE_NAMES.index("off_verified")
    finally:
        wg.clear_upper_lips()


def test_trajectory_streams_and_is_logged() -> None:
    clock, board, face = session()
    path = Path(tempfile.mkdtemp()) / "commands.csv"
    log = fs.CommandLog(path)
    face.on_command = log.write
    face.start()
    rows = [(0.0, ma.rest()), (0.5, ma.check_vector({"jaw": 0.5, "lip_right": 0.5, "lip_left": 0.5})),
            (1.0, ma.rest())]
    assert face.run(ma.trajectory_script(rows), sleep=lambda _s: clock.advance(0.005)) == "finished"
    log.close()
    with open(path, encoding="utf-8") as stream:
        logged = list(csv.DictReader(stream))
    assert log.rows == len(logged) >= 45
    times = [float(r["t_s"]) for r in logged]
    assert times == sorted(times)
    peak = max(logged, key=lambda r: float(r["jaw"]))
    assert abs(float(peak["jaw"]) - 0.5) < 0.03 and abs(float(peak["lip_left"]) - 0.5) < 0.03
    assert min(int(r["target_7"]) for r in logged) < 4000
    assert all(r["target_13"] == "" or int(r["target_13"]) == int(logged[0]["target_13"]) for r in logged)


def test_audio_start_time() -> None:
    assert fs.audio_start_s({"audio_start_s": "0.5"}, 100) == 0.4
    assert fs.audio_start_s({}, 0) == 0.0
    for meta, advance in (({"audio_start_s": "0.05"}, 100), ({}, 10), ({"audio_start_s": "x"}, 0)):
        try:
            fs.audio_start_s(meta, advance)
        except ValueError:
            continue
        raise AssertionError(f"{meta} {advance} accepted")


def test_camera_mode() -> None:
    assert fs.parse_camera_mode("1280x720@15") == (1280, 720, 15)
    for bad in ("1280x720", "1280@15", "ax720@15", "0x720@15", "1280x720@-1"):
        try:
            fs.parse_camera_mode(bad)
        except ValueError:
            continue
        raise AssertionError(f"{bad} accepted")


class FakePlayer:
    started = []

    def __init__(self, path) -> None:
        self.path, self.process, self.started_utc = path, None, None

    def start(self) -> None:
        self.process = object()
        self.started_utc = "2026-11-08T00:00:00+00:00"
        FakePlayer.started.append(self.path)

    def stop(self) -> None:
        pass


def write_wav(path: Path, seconds: float) -> None:
    import wave
    with wave.open(str(path), "wb") as stream:
        stream.setnchannels(1); stream.setsampwidth(2); stream.setframerate(8000)
        stream.writeframes(b"\0\0" * int(8000 * seconds))


def test_head_face_plays_the_audio_with_the_trajectory() -> None:
    import json
    folder = Path(tempfile.mkdtemp())
    trajectory = folder / "speech.csv"
    ma.write_trajectory(trajectory, [(0.0, ma.rest()), (0.4, ma.check_vector({"jaw": 0.3})),
                                     (0.8, ma.rest())], {"audio_start_s": "0.3"})
    wav = folder / "speech.wav"
    write_wav(wav, 0.4)
    FakePlayer.started = []
    code = fs.main(["--preview", "--records", str(folder), "--trajectory", str(trajectory),
                    "--audio", str(wav), "--audio-advance-ms", "100"], audio_player_factory=FakePlayer)
    try:
        assert code == 0 and FakePlayer.started == [wav]
        record = json.loads(next((folder / "preview" / "runs").glob("*/*/record.json")).read_text())
        assert abs(record["audio"]["start_s"] - 0.2) < 1e-9
        assert 0.2 <= record["audio"]["started_script_s"] < 0.3
        assert record["trajectory"]["rows"] == 3 and record["upper_lips"]["from"] == "preview"
        assert (Path(record["commands"])).exists()
        # Too little lead for the advance: refused before anything starts.
        assert fs.main(["--preview", "--records", str(folder), "--trajectory", str(trajectory),
                        "--audio", str(wav), "--audio-advance-ms", "400"],
                       audio_player_factory=FakePlayer) == 2
        assert fs.main(["--preview", "--records", str(folder), "--trajectory", str(trajectory),
                        "--frame", "hold=1"]) == 2
    finally:
        wg.clear_upper_lips()


def main() -> None:
    test_upper_lips_come_only_from_both_marks()
    test_configure_and_clear_upper_lips()
    test_upper_lips_stop_at_their_marks_on_the_simulated_board()
    test_trajectory_streams_and_is_logged()
    test_audio_start_time()
    test_camera_mode()
    test_head_face_plays_the_audio_with_the_trajectory()
    print("face stream lip-sync: all tests passed")


if __name__ == "__main__":
    main()
