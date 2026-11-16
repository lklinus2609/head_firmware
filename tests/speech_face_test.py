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
        assert set(actions) == {"right", "left", "lip"}
        # The lower lip (pulls DOWN) follows the jaw and rests on a seal.
        m = plan.track.mouth(t)
        assert abs(actions["lip"][0] - min(1.0, nf.SPEECH_LIP_WITH_JAW * m.jaw * (1 - m.seal))) < 1e-9
        if m.seal > 0.99:                         # (1 - seal) < 0.01; eSpeak timing varies per call
            assert actions["lip"][0] <= 0.01 * nf.SPEECH_LIP_WITH_JAW + 1e-9
    assert plan.face(20.5) is None
    assert any("open" in label for _t, label in plan.marks())


def test_articulation_gain() -> None:
    speech = sf.synthesize(TEXT, wpm=150)
    settings = nf.NaturalSettings(seconds=20.0, seed=2, smile_up=0.3, smile_diag=0.25)
    plans = {g: nf.SpeechPlan.make_speech(settings, sf.SpeechTrack.make(speech, 4.7, seed=2),
                                          lips=True, articulation=g) for g in (1.0, 2.0)}
    ts = [4.7 + x / 50 for x in range(int(speech.duration_s * 50))]
    # G = 1 is the plain track (the take that worked, run 142831).
    for t in ts:
        assert plans[1.0].face(t)[1] == plans[1.0].track.mouth(t).jaw
    v = {g: [p.face(t)[0]["right"][1] for t in ts] for g, p in plans.items()}
    jaw = {g: [p.face(t)[1] for t in ts] for g, p in plans.items()}
    lip = {g: [p.face(t)[0]["lip"][0] for t in ts] for g, p in plans.items()}
    assert min(v[1.0]) >= 0.0 and min(v[2.0]) < 0.0          # corners drop with the jaw
    assert max(v[2.0]) - min(v[2.0]) > max(v[1.0]) - min(v[1.0])
    assert sum(jaw[2.0]) > sum(jaw[1.0]) and max(jaw[2.0]) <= 1.0
    assert sum(lip[2.0]) > sum(lip[1.0]) and max(lip[2.0]) <= 1.0     # more lip with more articulation
    for t in ts:
        actions, j = plans[2.0].face(t)
        d, vv = actions["right"]
        assert 0.0 <= d <= 1.0 and -1.0 <= vv <= 1.0 and 0.0 <= j <= 1.0
    with tempfile.TemporaryDirectory() as tmp:
        out = io.StringIO()
        with redirect_stdout(out):
            code = nf.main(["--preview", "--records", tmp, "--say", "Hi.", "--articulation", "4"],
                           audio_player_factory=FakePlayer)
        assert code == 2 and "--articulation" in out.getvalue()


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
                            "--seed", "3", "--tts", "espeak"], audio_player_factory=FakePlayer)
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
                               (["--tts", "espeak", "--wpm", "10"], "words per minute"),
                               (["--wpm", "150"], "are for --tts espeak"),
                               (["--speed", "3"], "speed must be"),
                               (["--voice", "nobody"], "no Piper voice"),
                               (["--tts", "elevenlabs", "--voice", "male"], "no ElevenLabs male voice")):
            out = io.StringIO()
            with redirect_stdout(out):
                code = nf.main(["--preview", "--records", tmp, "--say",
                                "This sentence is long enough to need more than ten seconds of show time. "
                                "It goes on and on."] + extra, audio_player_factory=FakePlayer)
            assert code == 2 and message in out.getvalue(), (extra, out.getvalue())


def test_piper_alignment_merge() -> None:
    # "Hi, mˈaɪ ː" as Piper phonemes with sample counts (rate 100 Hz for easy numbers).
    items = [("^", 10), ("h", 10), ("a", 20), ("ɪ", 10), (",", 30), (" ", 5), ("m", 10),
             ("ˈ", 5), ("u", 10), ("ː", 10), (" ", 5), ("z", 10), (".", 20), ("$", 15)]
    total = sum(n for _p, n in items)
    phones, ends, words = sf.phones_from_alignments([(total, items), (30, [("^", 10), ("o", 20)])], 100)
    names = [p.ipa for p in phones]
    assert names == ["", "h", "a", "ɪ", "", "m", "uː", "z", "", "o"], names
    by = {p.ipa: p for p in phones}
    assert abs(by["uː"].start_s - 0.95) < 1e-9 and abs(by["uː"].end_s - 1.25) < 1e-9   # stress + length + space
    assert abs(phones[4].start_s - 0.5) < 1e-9 and abs(phones[4].end_s - 0.85) < 1e-9  # ", " is one pause
    assert abs(phones[8].start_s - 1.35) < 1e-9 and abs(phones[8].end_s - 1.8) < 1e-9  # ". $ ^" merged
    assert abs(phones[-1].end_s - (total + 30) / 100) < 1e-9
    assert ends == [0.5, 1.35] and [round(w, 2) for w in words] == [0.1, 0.85, 1.25, 1.8]
    for a, b in zip(phones, phones[1:]):
        assert abs(a.end_s - b.start_s) < 1e-9


def test_piper_speak() -> None:
    try:
        sf.piper_model_path("female")
        import piper  # noqa: F401
    except (ValueError, ImportError) as error:
        print(f"skip test_piper_speak: {error}")
        return
    speech = sf.speak(TEXT, "piper", "female")
    assert speech.tts == "piper" and speech.rate > 0 and speech.duration_s > 1.0
    assert not sf.summary(speech)["unknown_ipa"]
    assert speech.phones[-1].end_s <= speech.duration_s + 1e-9
    assert len(speech.clause_ends_s) >= 3 and len(speech.words_s) >= 10
    slower = sf.speak(TEXT, "piper", "female", speed=0.7)
    assert slower.duration_s > speech.duration_s * 1.2
    track = sf.SpeechTrack.make(speech, start_s=1.0)
    assert len(track.phrases) >= 3


def fake_elevenlabs(text: str, stretch: float, rate: int = 16000):
    """An ElevenLabs answer: eSpeak's word times scaled by stretch, spread
    over each word's characters; silent WAV of the stretched length."""
    import base64

    run = sf._espeak_run(text, "en-us", 150, 50, 50)
    starts = [0.0] * len(text)
    ends = [0.0] * len(text)
    words = sorted(run.words)
    for n, (at, first, length) in enumerate(words):
        stop = words[n + 1][0] if n + 1 < len(words) else run.speech.duration_s
        for k in range(length):
            starts[first + k] = stretch * (at + (stop - at) * 0.9 * k / length)
            ends[first + k] = stretch * (at + (stop - at) * 0.9 * (k + 1) / length)
    for i in range(1, len(text)):                 # spaces and marks: after the character before
        if starts[i] == 0.0 and ends[i] == 0.0:
            starts[i] = ends[i] = ends[i - 1]
    duration = stretch * run.speech.duration_s
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as out:
        out.setnchannels(1)
        out.setsampwidth(2)
        out.setframerate(rate)
        out.writeframes(b"\0\0" * int(duration * rate))
    answer = {"audio_base64": base64.b64encode(buffer.getvalue()).decode(),
              "alignment": {"characters": list(text), "character_start_times_seconds": starts,
                            "character_end_times_seconds": ends}}
    return run, answer


def test_elevenlabs_timing_warp() -> None:
    text = "Hello there. My name is Head, and I can talk."
    run, answer = fake_elevenlabs(text, 1.5)
    calls = []

    def request(path, body):
        calls.append((path, body))
        return answer

    speech = sf.synthesize_elevenlabs(text, "abc123", speed=1.2, request=request)
    path, body = calls[0]
    assert path.startswith("/v1/text-to-speech/abc123/with-timestamps?output_format=wav_22050")
    assert body["text"] == text and body["voice_settings"] == {"speed": 1.2}
    assert speech.tts == "elevenlabs" and speech.voice == "abc123" and speech.rate == 16000
    assert [p.ipa for p in speech.phones] == [p.ipa for p in run.speech.phones if p.end_s > p.start_s]
    # Word starts land on the stretched word starts.
    for got, (at, _f, _l) in zip(speech.words_s, run.words):
        assert abs(got - 1.5 * at) < 0.02, (got, at)
    assert [p.start_s for p in speech.phones] == sorted(p.start_s for p in speech.phones)
    assert speech.phones[-1].end_s <= speech.duration_s + 1e-9
    track = sf.SpeechTrack.make(speech, start_s=1.0)
    assert len(track.phrases) >= 2
    try:
        sf.synthesize_elevenlabs(text, "abc123", request=lambda p, b: {"audio_base64": "AAAA"})
    except RuntimeError as error:
        assert "not a WAV" in str(error)
    else:
        raise AssertionError("bad audio accepted")


if __name__ == "__main__":
    for name, test in sorted(globals().items()):
        if name.startswith("test_") and callable(test):
            test()
            print(f"ok {name}")
