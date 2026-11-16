"""Speech for the natural face: text -> audio + timed phonemes -> mouth track.

    from speech_face import synthesize, SpeechTrack
    speech = synthesize("Hello, I am Head.", voice="en-us", wpm=150)
    speech.write_wav(path)
    track = SpeechTrack.make(speech, start_s=4.0)
    track.mouth(t)   # -> MouthShape(jaw, spread, round, seal)

Option 1 of the lip-sync plan (2026-11-10): a speech engine gives the audio
and the time of every phoneme (speak(): Piper by default, ElevenLabs, or
eSpeak NG); each phoneme maps to a viseme (a mouth shape:
jaw opening, lip-corner spread, rounding, lip seal), and the shapes blend in
time with anticipation (the mouth moves before the sound). Option 3
(procedural detail) on top: the jaw follows the loudness of the audio as
well as the viseme, and the phrases give times for blinks, brow lifts and
gaze changes (SpeechTrack.phrases).

eSpeak NG is used through its C library (libespeak-ng.so.1, API revision
12, checked against speak_lib.h of libespeak-ng-dev 1.51 on 2026-11-11):
espeak_Initialize(AUDIO_OUTPUT_SYNCHRONOUS, 0, NULL,
espeakINITIALIZE_PHONEME_EVENTS | espeakINITIALIZE_PHONEME_IPA) returns the
sample rate; the synth callback gets 16-bit samples and a list of events
ending with type 0; a PHONEME event (type 7) has the IPA name in id.string
and its start in ms in audio_position. An empty name is a pause.

The viseme table and the blend times are chosen (common viseme groups for
English), not measured on this head; the camera tunes them later
(viseme_tuner.py).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import ctypes
import math
from pathlib import Path
import random
from typing import Dict, List, Optional, Sequence, Tuple
import wave

# eSpeak NG constants (speak_lib.h, API revision 12).
_AUDIO_OUTPUT_SYNCHRONOUS = 2
_INIT_PHONEME_EVENTS = 0x0001
_INIT_PHONEME_IPA = 0x0002
_POS_CHARACTER = 1
_CHARS_UTF8 = 1
_EVENT_LIST_TERMINATED = 0
_EVENT_WORD = 1
_EVENT_END = 5
_EVENT_PHONEME = 7
_RATE, _PITCH, _RANGE = 1, 3, 4
WPM_RANGE = (80, 450)                  # espeakRATE_MINIMUM / MAXIMUM

# eSpeak NG's PHONEME events come later than the sound: a stop's closure
# (silence) starts a median 39-50 ms before its event (103 stops in four
# sentences at 120/150/180 wpm, eSpeak NG 1.51, 2026-11-11). Phone times are
# moved this much earlier.
EVENT_LAG_S = 0.045
# Mouth blending (chosen): each viseme's move starts this much before its
# sound (people shape the lips 80-300 ms early; Hu et al. 2026), takes the
# blend time (at most the viseme's own length), then the shape holds until
# the next viseme's move starts.
ANTICIPATE_S = 0.07
BLEND_S = 0.08
# The jaw follows the viseme times this mix of the loudness envelope.
LOUDNESS_MIX = 0.35
LOUDNESS_HOP_S = 0.02
# A pause this long (or a clause end) splits phrases.
PHRASE_GAP_S = 0.15


@dataclass(frozen=True)
class MouthShape:
    jaw: float = 0.0       # 0 closed .. 1 the session's open goal
    spread: float = 0.0    # lip corners out/up (adds to the smile), 0..1
    round: float = 0.0     # corners relax inward (takes away the smile), 0..1
    seal: float = 0.0      # lips pressed (p b m), 0..1

    def mix(self, other: "MouthShape", u: float) -> "MouthShape":
        return MouthShape(*(a + (b - a) * u for a, b in zip(self.astuple(), other.astuple())))

    def astuple(self) -> Tuple[float, float, float, float]:
        return (self.jaw, self.spread, self.round, self.seal)


REST = MouthShape()
VISEMES: Dict[str, MouthShape] = {
    "rest": REST,
    "PBM": MouthShape(jaw=0.0, seal=1.0),                     # p b m: lips shut
    "FV": MouthShape(jaw=0.12, seal=0.4),                     # f v: lower lip to teeth
    "TH": MouthShape(jaw=0.2),                                # th
    "TD": MouthShape(jaw=0.25, spread=0.1),                   # t d n l s z, k g ng, h
    "SH": MouthShape(jaw=0.2, round=0.5),                     # sh zh ch j: lips forward
    "R": MouthShape(jaw=0.2, round=0.4),
    "W": MouthShape(jaw=0.15, round=0.9),                     # w, oo
    "EE": MouthShape(jaw=0.3, spread=0.8),                    # ee, i
    "EH": MouthShape(jaw=0.6, spread=0.4),                    # e, ae-ish mid
    "AA": MouthShape(jaw=1.0, spread=0.15),                   # a, ah: widest
    "AH": MouthShape(jaw=0.55),                               # schwa, uh
    "OH": MouthShape(jaw=0.7, round=0.6),                     # aw, o
    "OO": MouthShape(jaw=0.3, round=1.0),                     # u, oo
}

# IPA (as eSpeak NG en-us gives it) -> viseme; diphthongs -> two visemes over
# the two halves of the phoneme.
PHONE_VISEMES: Dict[str, Tuple[str, ...]] = {
    "p": ("PBM",), "b": ("PBM",), "m": ("PBM",),
    "f": ("FV",), "v": ("FV",),
    "θ": ("TH",), "ð": ("TH",),
    "t": ("TD",), "d": ("TD",), "n": ("TD",), "l": ("TD",), "s": ("TD",), "z": ("TD",),
    "k": ("TD",), "ɡ": ("TD",), "g": ("TD",), "ŋ": ("TD",), "h": ("TD",), "j": ("EE",),
    "ɾ": ("TD",), "ʔ": ("TD",), "x": ("TD",), "n̩": ("TD",), "l̩": ("TD",),
    "ʃ": ("SH",), "ʒ": ("SH",), "tʃ": ("SH",), "dʒ": ("SH",),
    "ɹ": ("R",), "r": ("R",), "ɚ": ("R",), "ɝ": ("R",), "ɜ": ("R",), "ɜː": ("R",),
    "w": ("W",),
    "i": ("EE",), "iː": ("EE",), "ɪ": ("EE",), "i̞": ("EE",), "ᵻ": ("EE",),
    "e": ("EH",), "ɛ": ("EH",), "æ": ("AA",), "a": ("AA",), "ɑ": ("AA",), "ɑː": ("AA",),
    "ʌ": ("AH",), "ə": ("AH",), "ɐ": ("AH",),
    "ɔ": ("OH",), "ɔː": ("OH",), "o": ("OH",), "ɒ": ("OH",),
    "u": ("OO",), "uː": ("OO",), "ʊ": ("OO",),
    "eɪ": ("EH", "EE"), "aɪ": ("AA", "EE"), "aʊ": ("AA", "OO"), "oʊ": ("OH", "OO"),
    "ɔɪ": ("OH", "EE"), "ɪə": ("EE", "AH"), "eə": ("EH", "AH"), "ʊə": ("OO", "AH"),
    "ɑːɹ": ("AA", "R"), "ɔːɹ": ("OH", "R"), "ɪɹ": ("EE", "R"), "ɛɹ": ("EH", "R"),
    "ʊɹ": ("OO", "R"), "aɪɚ": ("AA", "R"), "aɪə": ("AA", "AH"), "aʊɚ": ("AA", "R"),
}
# Marks eSpeak may put in a name (stress, length, syllabic): stripped before lookup.
_STRIP = "ˈˌ"


def viseme_names(ipa: str) -> Tuple[str, ...]:
    """The visemes of one phoneme ('rest' for a pause; unknown names fall
    back to the first character that is known, else 'AH')."""
    name = ipa.strip().strip(_STRIP)
    if not name:
        return ("rest",)
    if name in PHONE_VISEMES:
        return PHONE_VISEMES[name]
    for length in range(len(name) - 1, 0, -1):          # longest known prefix
        if name[:length] in PHONE_VISEMES:
            return PHONE_VISEMES[name[:length]]
    return ("AH",)


@dataclass
class Phone:
    start_s: float
    end_s: float
    ipa: str


@dataclass
class Speech:
    text: str
    rate: int
    samples: List[int]
    phones: List[Phone]
    clause_ends_s: List[float] = field(default_factory=list)     # audio timeline
    words_s: List[float] = field(default_factory=list)           # audio timeline
    tts: str = "espeak"
    voice: str = ""

    @property
    def duration_s(self) -> float:
        return len(self.samples) / float(self.rate)

    def write_wav(self, path: Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        import array

        data = array.array("h", self.samples)
        with wave.open(str(path), "wb") as out:
            out.setnchannels(1)
            out.setsampwidth(2)
            out.setframerate(self.rate)
            out.writeframes(data.tobytes())
        return path

    def loudness(self, hop_s: float = LOUDNESS_HOP_S) -> List[float]:
        """RMS every hop_s, divided by its 95th percentile, clipped to 0..1."""
        hop = max(1, int(self.rate * hop_s))
        values = []
        for i in range(0, len(self.samples), hop):
            chunk = self.samples[i:i + hop]
            values.append(math.sqrt(sum(x * x for x in chunk) / len(chunk)) if chunk else 0.0)
        if not values:
            return []
        ordered = sorted(values)
        top = ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))] or 1.0
        return [min(1.0, v / top) for v in values]


class _Union(ctypes.Union):
    _fields_ = [("number", ctypes.c_int), ("name", ctypes.c_char_p), ("string", ctypes.c_char * 8)]


class _Event(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("unique_identifier", ctypes.c_uint),
                ("text_position", ctypes.c_int), ("length", ctypes.c_int),
                ("audio_position", ctypes.c_int), ("sample", ctypes.c_int),
                ("user_data", ctypes.c_void_p), ("id", _Union)]


_CALLBACK = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_short), ctypes.c_int,
                             ctypes.POINTER(_Event))
_LIB = None
_RATE_HZ = 0


def _library():
    global _LIB, _RATE_HZ
    if _LIB is None:
        lib = ctypes.CDLL("libespeak-ng.so.1")
        rate = lib.espeak_Initialize(_AUDIO_OUTPUT_SYNCHRONOUS, 0, None,
                                     _INIT_PHONEME_EVENTS | _INIT_PHONEME_IPA)
        if rate <= 0:
            raise RuntimeError("espeak_Initialize failed")
        _LIB, _RATE_HZ = lib, rate
    return _LIB, _RATE_HZ


@dataclass
class _EspeakRun:
    speech: Speech
    # (start on the audio timeline, first character index (0-based), character
    # count) for each eSpeak WORD event; numbers can give overlapping words.
    words: List[Tuple[float, int, int]]


def _espeak_run(text: str, voice: str, wpm: int, pitch: int, pitch_range: int) -> _EspeakRun:
    if not text.strip():
        raise ValueError("no text")
    if not WPM_RANGE[0] <= wpm <= WPM_RANGE[1]:
        raise ValueError(f"words per minute must be {WPM_RANGE[0]}-{WPM_RANGE[1]}")
    if not (0 <= pitch <= 100 and 0 <= pitch_range <= 100):
        raise ValueError("pitch and pitch range must be 0-100")
    lib, rate = _library()
    if lib.espeak_SetVoiceByName(voice.encode()) != 0:
        raise ValueError(f"no eSpeak voice {voice!r}")
    lib.espeak_SetParameter(_RATE, int(wpm), 0)
    lib.espeak_SetParameter(_PITCH, int(pitch), 0)
    lib.espeak_SetParameter(_RANGE, int(pitch_range), 0)
    samples: List[int] = []
    raw: List[Tuple[int, int, str, int, int]] = []   # (type, ms, name, text position, length)

    def callback(wav, count, events):
        if wav and count > 0:
            samples.extend(wav[:count])
        i = 0
        while events[i].type != _EVENT_LIST_TERMINATED:
            event = events[i]
            name = ""
            if event.type == _EVENT_PHONEME:
                name = bytes(event.id.string).split(b"\0")[0].decode("utf-8", "replace")
            raw.append((event.type, event.audio_position, name, event.text_position, event.length))
            i += 1
        return 0

    keep = _CALLBACK(callback)                    # referenced until synthesis ends
    lib.espeak_SetSynthCallback(keep)
    data = text.encode("utf-8")
    result = lib.espeak_Synth(data, len(data) + 1, 0, _POS_CHARACTER, 0, _CHARS_UTF8, None, None)
    lib.espeak_Synchronize()
    if result != 0:
        raise RuntimeError(f"espeak_Synth returned {result}")
    duration = len(samples) / float(rate)
    # All event times move EVENT_LAG_S earlier, onto the audio timeline.
    events = [(t, max(0.0, ms / 1000.0 - EVENT_LAG_S), name, position, length)
              for t, ms, name, position, length in raw]
    starts = [(s, name) for t, s, name, _p, _l in events if t == _EVENT_PHONEME]
    phones = []
    for n, (start, name) in enumerate(starts):
        end = starts[n + 1][0] if n + 1 < len(starts) else duration
        if end > start:
            phones.append(Phone(start, end, name))
    # text_position counts characters from 1 (checked 2026-11-11: "Hé said"
    # gives 'said' at 4, so characters, not UTF-8 bytes).
    words = [(s, position - 1, length) for t, s, _n, position, length in events
             if t == _EVENT_WORD and length > 0]
    speech = Speech(text=text, rate=rate, samples=samples, phones=phones,
                    clause_ends_s=[s for t, s, *_ in events if t == _EVENT_END],
                    words_s=[s for s, _p, _l in words], tts="espeak", voice=voice)
    return _EspeakRun(speech, words)


def synthesize(text: str, voice: str = "en-us", wpm: int = 150, pitch: int = 50,
               pitch_range: int = 50) -> Speech:
    """Speak text with eSpeak NG into memory: samples, sample rate and the
    timed phonemes. ValueError for bad settings, RuntimeError if eSpeak fails."""
    return _espeak_run(text, voice, wpm, pitch, pitch_range).speech


# Piper (neural, local) ----------------------------------------------------------
# piper-tts 1.8.0 (bench/requirements.txt). A voice loaded with
# include_alignments=True gives, per sentence chunk, phoneme_alignments: one
# (phoneme, num_samples) per Piper phoneme, summing to the chunk's audio
# length (checked 2026-11-11, en_US-ljspeech-high, 256-sample steps). Piper's
# phonemes are single code points from eSpeak NG: stress marks and the length
# mark carry time of their own, diphthongs come as two phonemes, a space
# separates words, punctuation is a pause, ^ and $ start and end the chunk.
# The times come from the same durations that make the audio, so there is no
# event lag to correct.

PIPER_VOICE_DIR = Path(__file__).resolve().parents[5] / "bench" / "piper_voices"
PIPER_PRESETS: Dict[str, str] = {
    "female": "en_US-ljspeech-high",              # model card: "US English female voice"
    "male": "en_US-norman-medium",                # model card: "US English male voice"
}
PIPER_STRESS = "ˈˌ"
PIPER_LENGTH = "ː"
PIPER_BOUNDS = "^$"
SPEED_RANGE = (0.5, 2.0)


def piper_model_path(voice: str) -> Path:
    """A preset (female/male), a voice name in PIPER_VOICE_DIR, or a path to
    an .onnx model (its .onnx.json next to it). ValueError if missing."""
    name = PIPER_PRESETS.get(voice, voice)
    path = Path(name).expanduser()
    if path.suffix != ".onnx":
        path = PIPER_VOICE_DIR / f"{name}.onnx"
    if not path.is_file() or not Path(f"{path}.json").is_file():
        raise ValueError(f"no Piper voice {voice!r}: need {path} and {path}.json "
                         f"(python3 -m piper.download_voices --data-dir {PIPER_VOICE_DIR} {name})")
    return path


def _is_pause(phoneme: str) -> bool:
    import unicodedata

    return phoneme in PIPER_BOUNDS or all(unicodedata.category(c)[0] in "PS" for c in phoneme)


def phones_from_alignments(chunks: Sequence[Tuple[int, Sequence[Tuple[str, int]]]], rate: int
                           ) -> Tuple[List[Phone], List[float], List[float]]:
    """(phones, clause ends, word starts) from Piper chunks of (audio sample
    count, [(phoneme, sample count)]). Stress marks go to the next phone,
    length marks and spaces to the one before; punctuation and chunk bounds
    become pauses ('' phones, adjacent ones merged); a punctuation mark is a
    clause end."""
    phones: List[Phone] = []
    clause_ends: List[float] = []
    words: List[float] = []
    offset = 0
    for chunk_samples, items in chunks:
        at = offset
        carry: Optional[int] = None               # start of stress marks waiting for a phone
        new_word = True
        for phoneme, count in items:
            start, at = at, at + int(count)
            if phoneme and all(c in PIPER_STRESS for c in phoneme):
                carry = start if carry is None else carry
                continue
            last = phones[-1] if phones else None
            if phoneme == PIPER_LENGTH or phoneme.isspace():
                if phoneme.isspace():
                    new_word = True
                if last is not None and last.ipa:
                    last.end_s = at / rate
                    if phoneme == PIPER_LENGTH:
                        last.ipa += PIPER_LENGTH
                    continue
                phoneme = ""                      # nothing to join: a pause
            begin = carry if carry is not None else start
            carry = None
            if not phoneme or _is_pause(phoneme):
                if phoneme and phoneme not in PIPER_BOUNDS:
                    clause_ends.append(begin / rate)
                new_word = True
                if last is not None and not last.ipa:
                    last.end_s = at / rate
                else:
                    phones.append(Phone(begin / rate, at / rate, ""))
                continue
            if new_word:
                words.append(begin / rate)
                new_word = False
            phones.append(Phone(begin / rate, at / rate, phoneme))
        offset += chunk_samples
    return [p for p in phones if p.end_s > p.start_s], clause_ends, words


_PIPER_VOICES: Dict[Path, object] = {}


def synthesize_piper(text: str, voice: str = "female", speed: float = 1.0) -> Speech:
    """Speak text with a Piper voice (see piper_model_path). speed 1 = the
    voice's own pace. ValueError for bad settings, RuntimeError if the model
    gives no alignments."""
    if not text.strip():
        raise ValueError("no text")
    if not SPEED_RANGE[0] <= speed <= SPEED_RANGE[1]:
        raise ValueError(f"speed must be {SPEED_RANGE[0]}-{SPEED_RANGE[1]}")
    path = piper_model_path(voice)
    try:
        from piper import PiperVoice, SynthesisConfig
    except ImportError as error:
        raise RuntimeError("piper-tts is not installed (bench/requirements.txt)") from error
    model = _PIPER_VOICES.get(path)
    if model is None:
        model = _PIPER_VOICES[path] = PiperVoice.load(path, include_alignments=True)
    config = SynthesisConfig(length_scale=model.config.length_scale / speed)
    import array

    samples = array.array("h")
    chunks = []
    rate = 0
    for chunk in model.synthesize(text, syn_config=config, include_alignments=True):
        if chunk.sample_width != 2 or chunk.sample_channels != 1:
            raise RuntimeError("Piper gave audio that is not 16-bit mono")
        if rate and chunk.sample_rate != rate:
            raise RuntimeError("Piper changed sample rate between sentences")
        rate = chunk.sample_rate
        if not chunk.phoneme_alignments:
            raise RuntimeError(f"Piper voice {path.name} gives no phoneme alignments "
                               "(needs the onnx package: piper-tts[alignment])")
        count = len(chunk.audio_int16_bytes) // 2
        aligned = sum(int(a.num_samples) for a in chunk.phoneme_alignments)
        if aligned != count:
            raise RuntimeError(f"Piper alignment covers {aligned} samples of {count}")
        samples.frombytes(chunk.audio_int16_bytes)
        chunks.append((count, [(a.phoneme, int(a.num_samples)) for a in chunk.phoneme_alignments]))
    if not chunks:
        raise ValueError("Piper made no audio for this text")
    phones, clause_ends, words = phones_from_alignments(chunks, rate)
    return Speech(text=text, rate=rate, samples=samples.tolist(), phones=phones,
                  clause_ends_s=clause_ends, words_s=words, tts="piper", voice=path.stem)


# ElevenLabs (neural, cloud) ----------------------------------------------------
# POST /v1/text-to-speech/{voice_id}/with-timestamps, key in the xi-api-key
# header (ElevenLabs API docs, read 2026-11-11). The answer has audio_base64
# and alignment: characters with character_start_times_seconds and
# character_end_times_seconds, for the text as sent. Times are per character,
# not per phoneme, so eSpeak NG gives the phonemes and their relative times
# inside each word, and each word is stretched onto the time ElevenLabs spoke
# its characters. The docs do not say that audio_base64 is a WAV file for
# output_format wav_22050, so the code checks for the RIFF header.

ELEVENLABS_URL = "https://api.elevenlabs.io"
ELEVENLABS_MODEL = "eleven_multilingual_v2"       # the API's documented default model_id
ELEVENLABS_FORMAT = "wav_22050"
ELEVENLABS_KEY_ENV = "ELEVENLABS_API_KEY"
# voice_id for --voice female/male; empty until chosen (--list-voices).
ELEVENLABS_PRESETS: Dict[str, str] = {}


def _elevenlabs_request(path: str, body: Optional[dict] = None, timeout_s: float = 60.0) -> dict:
    import json as _json
    import os
    import urllib.error
    import urllib.request

    key = os.environ.get(ELEVENLABS_KEY_ENV, "").strip()
    if not key:
        raise RuntimeError(f"set {ELEVENLABS_KEY_ENV} to an ElevenLabs API key")
    data = None if body is None else _json.dumps(body).encode("utf-8")
    request = urllib.request.Request(ELEVENLABS_URL + path, data=data,
                                     method="GET" if body is None else "POST",
                                     headers={"xi-api-key": key, "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            return _json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        detail = error.read().decode("utf-8", "replace")[:300]
        raise RuntimeError(f"ElevenLabs HTTP {error.code}: {detail}") from error
    except urllib.error.URLError as error:
        raise RuntimeError(f"ElevenLabs not reachable: {error.reason}") from error


def elevenlabs_voices() -> List[Dict[str, str]]:
    """The account's default (premade) voices: voice_id, name, gender, accent."""
    out = []
    token = None
    while True:
        query = "/v2/voices?voice_type=default&page_size=100"
        if token:
            import urllib.parse

            query += "&next_page_token=" + urllib.parse.quote(token)
        answer = _elevenlabs_request(query)
        for v in answer.get("voices", []):
            labels = v.get("labels") or {}
            out.append({"voice_id": v.get("voice_id", ""), "name": v.get("name", ""),
                        "gender": labels.get("gender", ""), "accent": labels.get("accent", "")})
        token = answer.get("next_page_token")
        if not answer.get("has_more") or not token:
            return out


def _warp(t: float, anchors: Sequence[Tuple[float, float]]) -> float:
    """Piecewise-linear map through (from, to) anchors (both increasing)."""
    if t <= anchors[0][0]:
        return anchors[0][1]
    for (a0, b0), (a1, b1) in zip(anchors, anchors[1:]):
        if t <= a1:
            return b0 + (b1 - b0) * (t - a0) / (a1 - a0)
    return anchors[-1][1]


def phones_onto_characters(run: _EspeakRun, characters: Sequence[str], starts_s: Sequence[float],
                           ends_s: Sequence[float], duration_s: float
                           ) -> Tuple[List[Phone], List[float], List[float]]:
    """(phones, clause ends, word starts): eSpeak's timing stretched, word by
    word, onto per-character times of another voice speaking run's text."""
    import difflib

    text = run.speech.text
    spoken = "".join(characters)
    index: Dict[int, int] = {}                    # text character -> alignment character
    for block in difflib.SequenceMatcher(None, text, spoken, autojunk=False).get_matching_blocks():
        for k in range(block.size):
            index[block.a + k] = block.b + k
    espeak = run.speech
    anchors: List[Tuple[float, float]] = [(0.0, 0.0)]
    for n, (word_at, first, length) in enumerate(run.words):
        chars = [index[i] for i in range(first, first + length)
                 if i in index and not text[i].isspace()]
        if not chars:
            continue
        next_at = run.words[n + 1][0] if n + 1 < len(run.words) else espeak.duration_s
        sounding = [p for p in espeak.phones
                    if p.ipa.strip(_STRIP) and word_at - 1e-6 <= p.start_s < next_at - 1e-6]
        word_end = sounding[-1].end_s if sounding else next_at
        pairs = [(word_at, starts_s[min(chars)])]
        if word_end < next_at - 1e-3:             # a word end on the next start: keep the start
            pairs.append((word_end, ends_s[max(chars)]))
        for a, b in pairs:
            if a > anchors[-1][0] + 1e-6 and b > anchors[-1][1] + 1e-6:
                anchors.append((a, b))
    end = (espeak.duration_s, duration_s)
    if end[0] > anchors[-1][0] and end[1] > anchors[-1][1]:
        anchors.append(end)
    if len(anchors) < 3:
        raise RuntimeError("the ElevenLabs timing does not match the text")
    phones = []
    for p in espeak.phones:
        a, b = _warp(p.start_s, anchors), _warp(p.end_s, anchors)
        if b > a:
            phones.append(Phone(a, b, p.ipa))
    return (phones, [_warp(t, anchors) for t in espeak.clause_ends_s],
            [_warp(t, anchors) for t in espeak.words_s])


def synthesize_elevenlabs(text: str, voice: str, speed: float = 1.0, model_id: str = ELEVENLABS_MODEL,
                          request=None) -> Speech:
    """Speak text with an ElevenLabs voice (preset or voice_id). The text goes
    to ElevenLabs. request(path, body) -> answer dict stands in for the HTTP
    call in tests. ValueError for bad settings, RuntimeError for API errors."""
    import base64
    import io
    import urllib.parse

    if not text.strip():
        raise ValueError("no text")
    if not SPEED_RANGE[0] <= speed <= SPEED_RANGE[1]:
        raise ValueError(f"speed must be {SPEED_RANGE[0]}-{SPEED_RANGE[1]}")
    voice_id = ELEVENLABS_PRESETS.get(voice, voice)
    if voice in ("female", "male") and voice not in ELEVENLABS_PRESETS:
        raise ValueError(f"no ElevenLabs {voice} voice chosen yet: run --list-voices, then --voice VOICE_ID")
    body: Dict[str, object] = {"text": text, "model_id": model_id}
    if speed != 1.0:
        body["voice_settings"] = {"speed": speed}
    path = (f"/v1/text-to-speech/{urllib.parse.quote(voice_id)}/with-timestamps"
            f"?output_format={ELEVENLABS_FORMAT}")
    answer = (request or _elevenlabs_request)(path, body)
    audio = base64.b64decode(answer.get("audio_base64") or "")
    if audio[:4] != b"RIFF":
        raise RuntimeError(f"ElevenLabs audio is not a WAV file ({ELEVENLABS_FORMAT}); "
                           f"first bytes {audio[:4]!r}")
    with wave.open(io.BytesIO(audio)) as stream:
        if stream.getsampwidth() != 2 or stream.getnchannels() != 1:
            raise RuntimeError("ElevenLabs WAV is not 16-bit mono")
        rate = stream.getframerate()
        import array

        samples = array.array("h")
        samples.frombytes(stream.readframes(stream.getnframes()))
    alignment = answer.get("alignment") or {}
    characters = alignment.get("characters") or []
    starts = alignment.get("character_start_times_seconds") or []
    ends = alignment.get("character_end_times_seconds") or []
    if not characters or not len(characters) == len(starts) == len(ends):
        raise RuntimeError("ElevenLabs gave no character timing")
    run = _espeak_run(text, "en-us", 150, 50, 50)
    phones, clause_ends, words = phones_onto_characters(run, characters, starts, ends,
                                                        len(samples) / float(rate))
    return Speech(text=text, rate=rate, samples=samples.tolist(), phones=phones,
                  clause_ends_s=clause_ends, words_s=words, tts="elevenlabs", voice=voice_id)


# One entry point ---------------------------------------------------------------
TTS_ENGINES = ("piper", "elevenlabs", "espeak")
DEFAULT_VOICE = {"piper": "female", "elevenlabs": "female", "espeak": "en-us"}


def speak(text: str, tts: str = "piper", voice: Optional[str] = None, speed: float = 1.0,
          wpm: int = 150, pitch: int = 50) -> Speech:
    """Speech from any engine. speed is for piper/elevenlabs, wpm and pitch
    for espeak."""
    voice = voice or DEFAULT_VOICE.get(tts, "")
    if tts == "piper":
        return synthesize_piper(text, voice, speed)
    if tts == "elevenlabs":
        return synthesize_elevenlabs(text, voice, speed)
    if tts == "espeak":
        return synthesize(text, voice, wpm, pitch)
    raise ValueError(f"no speech engine {tts!r} ({', '.join(TTS_ENGINES)})")


def _ease(u: float) -> float:
    u = min(1.0, max(0.0, u))
    return 0.5 - 0.5 * math.cos(math.pi * u)


@dataclass
class SpeechTrack:
    """The mouth over time for one utterance placed at start_s (script time).
    segments: (move start, move length, shape); the mouth moves from where it
    is at the move start to the shape, then holds it."""
    start_s: float
    duration_s: float
    segments: List[Tuple[float, float, MouthShape]]
    loud: List[float]
    phrases: List[Tuple[float, float]]            # script times
    jitter: List[Tuple[float, float]] = field(default_factory=list)   # (t, spread offset)
    _from: List[MouthShape] = field(default_factory=list)             # shape at each move start

    @classmethod
    def make(cls, speech: Speech, start_s: float, anticipate_s: float = ANTICIPATE_S,
             blend_s: float = BLEND_S, seed: int = 1) -> "SpeechTrack":
        segments: List[Tuple[float, float, MouthShape]] = []
        for phone in speech.phones:
            names = viseme_names(phone.ipa)
            part = (phone.end_s - phone.start_s) / len(names)
            for k, name in enumerate(names):
                at = start_s + phone.start_s + k * part - anticipate_s
                segments.append((at, max(0.02, min(blend_s, part)), VISEMES[name]))
        segments.append((start_s + speech.duration_s - anticipate_s, blend_s, REST))
        segments.sort(key=lambda item: item[0])
        # Phrases: split at clause ends and at pauses of PHRASE_GAP_S or more.
        phrases: List[Tuple[float, float]] = []
        begin: Optional[float] = None
        last_end = 0.0
        ends = sorted(speech.clause_ends_s)
        for phone in speech.phones:
            silent = viseme_names(phone.ipa) == ("rest",)
            if not silent:
                if begin is None:
                    begin = phone.start_s
                last_end = phone.end_s
                continue
            if begin is not None and (phone.end_s - phone.start_s >= PHRASE_GAP_S or
                                      any(abs(e - phone.start_s) < 0.02 or abs(e - phone.end_s) < 0.02
                                          for e in ends)):
                phrases.append((start_s + begin, start_s + last_end))
                begin = None
        if begin is not None:
            phrases.append((start_s + begin, start_s + last_end))
        rng = random.Random(seed)
        jitter = []
        t = start_s
        while t < start_s + speech.duration_s:
            jitter.append((t, rng.uniform(-0.06, 0.06)))
            t += rng.uniform(0.25, 0.45)
        track = cls(start_s, speech.duration_s, segments, speech.loudness(), phrases, jitter)
        # Where each move starts from: the mouth at that time under the
        # previous moves (a short viseme may not have been reached).
        current = REST
        for k, (at, length, shape) in enumerate(segments):
            if k:
                p_at, p_len, p_shape = segments[k - 1]
                u = _ease((at - p_at) / p_len) if p_len > 0 else 1.0
                current = track._from[k - 1].mix(p_shape, u)
            track._from.append(current)
        return track

    @property
    def keys(self) -> List[Tuple[float, MouthShape]]:
        """(time the shape is reached, shape) for each viseme."""
        return [(at + length, shape) for at, length, shape in self.segments]

    @property
    def end_s(self) -> float:
        return self.start_s + self.duration_s

    def _viseme(self, t: float) -> MouthShape:
        segments = self.segments
        if not segments or t < segments[0][0]:
            return REST
        lo, hi = 0, len(segments) - 1                 # last move that started by t
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if segments[mid][0] <= t:
                lo = mid
            else:
                hi = mid - 1
        at, length, shape = segments[lo]
        u = _ease((t - at) / length) if length > 0 else 1.0
        return self._from[lo].mix(shape, u)

    def loudness(self, t: float) -> float:
        i = int((t - self.start_s) / LOUDNESS_HOP_S)
        return self.loud[i] if 0 <= i < len(self.loud) else 0.0

    def _jitter(self, t: float) -> float:
        value = 0.0
        for start, offset in self.jitter:
            if start > t:
                break
            value = offset
        return value

    def mouth(self, t: float) -> MouthShape:
        """The mouth shape at script time t (REST outside the speech)."""
        shape = self._viseme(t)
        if shape == REST:
            return REST
        # The jaw follows the viseme, raised or lowered by the loudness (a
        # loud vowel opens more); a sealed mouth stays shut.
        loud = self.loudness(t + ANTICIPATE_S)
        jaw = shape.jaw * ((1.0 - LOUDNESS_MIX) + LOUDNESS_MIX * 2.0 * loud)
        jaw *= (1.0 - shape.seal)
        spread = shape.spread + self._jitter(t)
        return MouthShape(jaw=min(1.0, max(0.0, jaw)), spread=min(1.0, max(0.0, spread)),
                          round=shape.round, seal=shape.seal)

    def speaking(self, t: float) -> bool:
        return any(a <= t <= b for a, b in self.phrases)


def summary(speech: Speech) -> Dict[str, object]:
    names = [p.ipa for p in speech.phones]
    unknown = sorted({n for n in names if n.strip(_STRIP) and n.strip(_STRIP) not in PHONE_VISEMES})
    return {"text": speech.text, "tts": speech.tts, "voice_used": speech.voice,
            "rate_hz": speech.rate, "duration_s": round(speech.duration_s, 3),
            "phones": len(names), "unknown_ipa": unknown}


# Audio + video ----------------------------------------------------------------
# The camera runs a little slower than the nominal rate written in the video
# (14.90 fps for 15 in runs 121358/122720, 2026-11-11), so the video plays
# about 0.7 % fast. The audio is placed by frame count (where the audio start
# falls between two frames) and sped up by the same factor (written with a
# higher sample rate), so sound and picture stay together over the whole run.

def read_frame_times(path: Path) -> List[float]:
    """Host UTC of each video frame (video.frames.csv) as POSIX seconds."""
    import csv
    from datetime import datetime

    with open(path, encoding="utf-8") as stream:
        return [datetime.fromisoformat(row["host_utc"]).timestamp() for row in csv.DictReader(stream)]


def align_audio(samples: Sequence[int], rate: int, frame_s: Sequence[float], audio_start_s: float,
                nominal_fps: float) -> Tuple[List[int], int, Dict[str, float]]:
    """(samples with leading silence or trimmed, sample rate to write, info)
    so that the audio plays against the video's nominal timeline as it
    played against the real one."""
    if len(frame_s) < 2:
        raise ValueError("need at least two video frames")
    measured = (len(frame_s) - 1) / (frame_s[-1] - frame_s[0])
    factor = nominal_fps / measured
    if audio_start_s <= frame_s[0]:
        position = (audio_start_s - frame_s[0]) * measured
    elif audio_start_s >= frame_s[-1]:
        raise ValueError("the audio started after the last video frame")
    else:
        i = max(k for k in range(len(frame_s) - 1) if frame_s[k] <= audio_start_s)
        position = i + (audio_start_s - frame_s[i]) / (frame_s[i + 1] - frame_s[i])
    offset_s = position / nominal_fps                    # on the video's timeline
    out_rate = int(round(rate * factor))
    pad = int(round(offset_s * out_rate))
    out = [0] * pad + list(samples) if pad >= 0 else list(samples)[-pad:]
    return out, out_rate, {"measured_fps": round(measured, 4), "speed_factor": round(factor, 5),
                           "offset_s": round(offset_s, 4), "rate_hz": out_rate}


def write_samples(path: Path, samples: Sequence[int], rate: int) -> Path:
    import array

    path = Path(path)
    with wave.open(str(path), "wb") as out:
        out.setnchannels(1)
        out.setsampwidth(2)
        out.setframerate(rate)
        out.writeframes(array.array("h", samples).tobytes())
    return path


def mux(video: Path, audio_wav: Path, out: Path, timeout_s: float = 300.0) -> Path:
    """VP8 WebM video + WAV -> WebM with Opus audio (GStreamer; the video is
    not re-encoded). RuntimeError if gst-launch-1.0 fails."""
    import subprocess

    command = ["gst-launch-1.0", "-q", "-e",
               "webmmux", "name=mux", "!", "filesink", f"location={out}",
               "filesrc", f"location={video}", "!", "matroskademux", "!", "queue", "!", "mux.",
               "filesrc", f"location={audio_wav}", "!", "wavparse", "!", "audioconvert", "!",
               "audioresample", "!", "audio/x-raw,rate=48000", "!", "opusenc", "!", "queue", "!", "mux."]
    result = subprocess.run(command, capture_output=True, text=True, timeout=timeout_s)
    if result.returncode != 0 or not Path(out).exists() or Path(out).stat().st_size == 0:
        raise RuntimeError(f"gst-launch-1.0 failed ({result.returncode}): {result.stderr.strip()[-400:]}")
    return Path(out)


def mux_run(run: Path, speech_wav: Path, audio_started_utc: str, nominal_fps: float,
            video_name: str = "video.webm", out_name: str = "video_with_audio.webm") -> Dict[str, object]:
    """Align the run's speech audio to its video and write out_name next to it."""
    from datetime import datetime

    run = Path(run)
    video = run / video_name
    frames = read_frame_times(run / "video.frames.csv")
    with wave.open(str(speech_wav), "rb") as stream:
        rate = stream.getframerate()
        import array

        data = array.array("h")
        data.frombytes(stream.readframes(stream.getnframes()))
    start = datetime.fromisoformat(audio_started_utc).timestamp()
    samples, out_rate, info = align_audio(list(data), rate, frames, start, nominal_fps)
    aligned = write_samples(run / "speech_aligned.wav", samples, out_rate)
    out = mux(video, aligned, run / out_name)
    return {**info, "video": str(out), "aligned_wav": str(aligned)}
