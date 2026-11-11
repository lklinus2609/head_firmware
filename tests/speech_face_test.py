"""Tests for speech on the natural face (speech_face.py, natural_face.py --say).
Needs libespeak-ng.so.1 (eSpeak NG) for the synthesis tests."""

from __future__ import annotations

import io
import json
from contextlib import redirect_stdout
from pathlib import Path
import sys
import tempfile
import wave

FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))

import natural_face as nf  # noqa: E402
import speech_face as sf  # noqa: E402

TEXT = "Hello! My name is Head. I can blink, and smile while I talk."


def test_viseme_names() -> None:
    assert sf.viseme_names("") == ("rest",)
    assert sf.viseme_names("m") == ("PBM",)
    assert sf.viseme_names("ˈæ") == ("AA",)                 # stress mark stripped
    assert sf.viseme_names("aɪ") == ("AA", "EE")            # diphthong: two halves
    assert sf.viseme_names("əl") == ("AH",)                 # longest known prefix
    assert sf.viseme_names("§") == ("AH",)                  # unknown: neutral vowel
    for names in sf.PHONE_VISEMES.values():
        assert all(n in sf.VISEMES for n in names)


def test_synthesize_and_wav() -> None:
    speech = sf.synthesize(TEXT, wpm=150)
    assert speech.rate > 0 and speech.duration_s > 1.0
    assert speech.phones and all(p.end_s > p.start_s for p in speech.phones)
    assert [p.start_s for p in speech.phones] == sorted(p.start_s for p in speech.phones)
    assert speech.phones[-1].end_s <= speech.duration_s + 1e-9
    assert not sf.summary(speech)["unknown_ipa"]
    with tempfile.TemporaryDirectory() as tmp:
        path = speech.write_wav(Path(tmp) / "s.wav")
        with wave.open(str(path)) as w:
            assert w.getframerate() == speech.rate and w.getnframes() == len(speech.samples)
    for bad in (dict(wpm=20), dict(pitch=120)):
        try:
            sf.synthesize(TEXT, **bad)
        except ValueError:
            pass
        else:
            raise AssertionError(bad)


def test_track_shapes_and_timing() -> None:
    speech = sf.synthesize("map map map", wpm=120)
    track = sf.SpeechTrack.make(speech, start_s=2.0)
    assert track.mouth(0.0) == sf.REST and track.mouth(track.end_s + 1.0) == sf.REST
    m_phones = [p for p in speech.phones if p.ipa == "m"]
    assert m_phones
    for p in m_phones:
        # The seal is reached before the sound ends; the jaw is shut then.
        mid = 2.0 + (p.start_s + p.end_s) / 2
        shape = track.mouth(mid)
        assert shape.seal > 0.5 and shape.jaw < 0.25, (p, shape)
    vowels = [p for p in speech.phones if sf.viseme_names(p.ipa)[0] == "AA"]
    assert vowels and max(track.mouth(2.0 + (p.start_s + p.end_s) / 2).jaw for p in vowels) > 0.5
    # Anticipation: each viseme's move starts ANTICIPATE_S before its sound.
    first = speech.phones[0]
    assert abs(track.segments[0][0] - (2.0 + first.start_s - sf.ANTICIPATE_S)) < 1e-9
    # A vowel holds its shape between its move and the next one.
    for p in vowels:
        a, b = 2.0 + p.start_s - sf.ANTICIPATE_S + sf.BLEND_S, 2.0 + p.end_s - sf.ANTICIPATE_S
        if b > a:
            assert track._viseme((a + b) / 2) == sf.VISEMES["AA"]
    for t in [2.0 + x / 50 for x in range(int(speech.duration_s * 50))]:
        m = track.mouth(t)
        assert 0.0 <= m.jaw <= 1.0 and 0.0 <= m.spread <= 1.0


def test_phrases_split_at_clauses() -> None:
    speech = sf.synthesize("One two three. Four five six.", wpm=150)
    track = sf.SpeechTrack.make(speech, start_s=1.0)
    assert len(track.phrases) >= 2
    assert all(b > a for a, b in track.phrases)
    assert all(track.phrases[k][1] <= track.phrases[k + 1][0] for k in range(len(track.phrases) - 1))


def test_align_audio_by_frames() -> None:
    fps_real, nominal = 14.9, 15.0
    frames = [100.0 + k / fps_real for k in range(300)]
    samples = [1000] * 2205
    # Audio starts exactly at frame 30: on the video's timeline that is 30/15 s.
    out, rate, info = sf.align_audio(samples, 22050, frames, frames[30], nominal)
    assert abs(info["offset_s"] - 2.0) < 1e-6
    assert rate == round(22050 * nominal / info["measured_fps"]) and rate > 22050
    assert out[:len(out) - len(samples)] == [0] * (len(out) - len(samples))
    # Audio that started 0.1 s before the first frame is trimmed at the front.
    out, rate, info = sf.align_audio(samples, 22050, frames, frames[0] - 0.1, nominal)
    assert len(out) < len(samples) and info["offset_s"] < 0
    try:
        sf.align_audio(samples, 22050, frames, frames[-1] + 1.0, nominal)
    except ValueError:
        pass
    else:
        raise AssertionError("audio after the video")


def test_speech_plan_blinks_gaze_and_actions() -> None:
    speech = sf.synthesize(TEXT, wpm=150)
    settings = nf.NaturalSettings(seconds=20.0, seed=2)
    track = sf.SpeechTrack.make(speech, start_s=4.7, seed=2)
    plan = nf.SpeechPlan.make_speech(settings, track, lips=True)
    assert plan.syllables == []
    assert plan.blinks == sorted(plan.blinks) and plan.blinks
    assert [s[0] for s in plan.saccades] == sorted(s[0] for s in plan.saccades)
    ends = [b for _a, b in track.phrases]
    assert any(any(abs(bl - (e + 0.05)) < 1e-9 for e in ends) for bl in plan.blinks)
    for t in [x / 50 for x in range(0, 20 * 50)]:
        actions, j = plan.face(t)
        assert 0.0 <= j <= 1.0
        for side in ("right", "left"):
            d, v = actions[side]
            assert 0.0 <= d <= 1.0 and -1.0 <= v <= 1.0
        assert set(actions) == {"right", "left", "lip", "upper_lip"}
    assert plan.face(20.5) is None
    assert any("open" in label for _t, label in plan.marks())


class FakePlayer:
    started = []

    def __init__(self, path) -> None:
        self.path, self.process, self.started_utc = Path(path), None, None

    def start(self) -> None:
        self.process = object()
        self.started_utc = "2026-11-11T00:00:00+00:00"
        FakePlayer.started.append(self.path)

    def stop(self) -> None:
        pass


def test_natural_face_say_preview() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        out = io.StringIO()
        with redirect_stdout(out):
            code = nf.main(["--preview", "--records", tmp, "--say", "Hello there. How are you?",
                            "--seed", "3"], audio_player_factory=FakePlayer)
        assert code == 0, out.getvalue()
        record = sorted(Path(tmp).rglob("record.json"))[-1]
        log = json.loads(record.read_text())
        speech = log["natural"]["speech"]
        assert speech["played"] and speech["started_script_s"] >= speech["audio_at_s"]
        assert (record.parent / nf.SPEECH_FILE).exists()
        assert FakePlayer.started and FakePlayer.started[-1].name == nf.SPEECH_FILE
        assert log["natural"]["syllables"] == []


def test_say_refusals() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        for extra, message in ((["--explore"], "does not go with"),
                               (["--seconds", "10"], "--seconds must be at least"),
                               (["--audio-advance-ms", "900"], "-500 to 500"),
                               (["--wpm", "10"], "words per minute")):
            out = io.StringIO()
            with redirect_stdout(out):
                code = nf.main(["--preview", "--records", tmp, "--say",
                                "This sentence is long enough to need more than ten seconds of show time. "
                                "It goes on and on."] + extra, audio_player_factory=FakePlayer)
            assert code == 2 and message in out.getvalue(), (extra, out.getvalue())


if __name__ == "__main__":
    for name, test in sorted(globals().items()):
        if name.startswith("test_") and callable(test):
            test()
            print(f"ok {name}")
