"""Tests for the jaw probe's pulse mode (jaw_probe.py --pulse-ma), on the
simulated board."""

from __future__ import annotations

import io
import json
from contextlib import redirect_stdout
from pathlib import Path
import sys
import tempfile

FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "host_ros" / "src" / "head_ros" / "head_ros"))

import jaw_probe  # noqa: E402

BASE = ["--preview", "--limit", "650", "--abort", "280", "--jaw-abort-ma", "615",
        "--jaw-push", "400", "--open", "-300", "--jaw-peak-s", "1.0", "--rest-s", "1"]


def run(extra, records: Path):
    out = io.StringIO()
    with redirect_stdout(out):
        code = jaw_probe.main(BASE + ["--records", str(records)] + extra)
    return code, out.getvalue()


def probe_log(records: Path) -> dict:
    paths = sorted((records / "preview").rglob("jawprobe.json"))
    assert paths, "no jawprobe.json written"
    return json.loads(paths[-1].read_text())


def test_pulse_ladder_opens_closes_and_logs() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        records = Path(tmp)
        code, text = run(["--payout", "200", "--pulse-ma", "450", "--pulse-ma", "580",
                          "--pulse-s", "0.4"], records)
        assert code == 0, text
        log = probe_log(records)
        pulses = log["pulse"]["pulses"]
        assert [p["push_ma"] for p in pulses] == [450, 580]
        start = log["start"]["7"]["goal"]
        for p in pulses:
            assert p["opened_ticks"] > 0
            assert p["widest"] >= start - 300            # never past --open
            assert abs(p["closed"]["present"] - start) <= 5
            assert p["samples"][-1][0] < 0.4 + 0.05       # pulse length kept
        assert log["stopped"] is None
        # Lines paid out by 200 and reeled back to their start goals.
        for index, pull in log["pull"].items():
            assert log["paid_out"][index]["present"] != log["start"][index]["present"]


def test_payout_zero_allowed_only_with_pulses() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        code, _ = run(["--payout", "0", "--pulse-ma", "300", "--pulse-s", "0.2"], Path(tmp))
        assert code == 0
        code, text = run(["--payout", "0"], Path(tmp))
        assert code == 4 and "Payout" in text


def test_refusals() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        records = Path(tmp)
        cases = [
            (["--payout", "0", "--pulse-ma", "450", "--pulse-s", "1.2"], "--pulse-s must be"),
            (["--payout", "0", "--pulse-ma", "450", "--pulse-s", "1.0"], "shorter than the jaw peak"),
            (["--payout", "0", "--pulse-ma", "615"], "below the jaw abort"),
            (["--payout", "0", "--pulse-ma", "450", "--rest-s", "0.5"], "--rest-s must be"),
        ]
        for extra, message in cases:
            code, text = run(extra, records)
            assert code == 4 and message in text, (extra, text)
        # Push at or over 95 % of the jaw's 650 mA cap (with a higher abort).
        out = io.StringIO()
        with redirect_stdout(out):
            code = jaw_probe.main([a if a != "615" else "640" for a in BASE]
                                  + ["--records", str(records), "--payout", "0", "--pulse-ma", "620"])
        assert code == 4 and "95 %" in out.getvalue()


def test_open_must_be_negative_in_pulse_mode() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        args = [a if a != "-300" else "100" for a in BASE]
        out = io.StringIO()
        with redirect_stdout(out):
            code = jaw_probe.main(args + ["--records", tmp, "--payout", "0", "--pulse-ma", "300"])
        assert code == 4 and "negative" in out.getvalue()


if __name__ == "__main__":
    for name, test in sorted(globals().items()):
        if name.startswith("test_") and callable(test):
            test()
            print(f"ok {name}")
