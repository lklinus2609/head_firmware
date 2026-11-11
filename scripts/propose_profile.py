#!/usr/bin/env python3
"""Propose calibration profile values from tuning records.

Reads one or more records written by the tuning panel and a base profile (the
one currently uploaded), and writes a proposed profile for review:

- home_tick              = the marked start (or the pose the latest session
                           started at, which is reported)
- min_tick / max_tick    = with two ends marked: both ends, each moved inward
                           by --end-margin-ticks. With one end marked (the
                           usual case: home plus the farthest the servo goes):
                           exactly home on one side, and the end moved inward
                           by --end-margin-ticks on the other
- operating_current_ma   = the peak load seen while tuning, plus
                           --current-margin-ma (capped at the servo's 910 mA)

Both margins are required: they are a safety decision, not a default. Servos
without a start and at least one end keep their base values and are listed.
Later records override earlier marks for the same servo; the peak load is the
highest across all records. The result is validated with the same checks the
firmware applies (generate_profile.validate) and is never uploaded by this
tool.

    scripts/propose_profile.py --base profiles/probe_all_twenty.json \\
        --record tuning_runs/panel/2026-10-04_070000_tuning.json \\
        --end-margin-ticks N --current-margin-ma M --output proposed.json
"""
import argparse
import copy
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_profile import DXL_OPERATING_CURRENT_MAX_MA, validate  # noqa: E402

RECORD_FORMAT = "head-tuning-record-1"


def merge_records(records: list[dict]) -> dict[int, dict]:
    servos: dict[int, dict] = {}
    for record in records:
        if record.get("format") != RECORD_FORMAT:
            raise ValueError(f"not a tuning record (format {record.get('format')!r})")
        for key, data in record["servos"].items():
            index = int(key)
            merged = servos.setdefault(index, {"marks": {}, "peak_current_ma": 0,
                                               "neutral_tick": None})
            if data.get("neutral_tick") is not None:
                merged["neutral_tick"] = int(data["neutral_tick"])
            merged["marks"].update(data.get("marks", {}))
            merged["peak_current_ma"] = max(merged["peak_current_ma"],
                                            int(data.get("peak_current_ma", 0)))
    return servos


def propose(base: dict, servos: dict[int, dict], end_margin_ticks: int,
            current_margin_ma: int) -> tuple[dict, list[str], list[str]]:
    profile = copy.deepcopy(base)
    slots = {int(slot["index"]): slot for slot in profile["joints"]}
    active_mask = int(profile["active_servo_mask"])
    notes: list[str] = []
    problems: list[str] = []
    for index in range(len(slots)):
        if not active_mask & (1 << index):
            continue
        tuned = servos.get(index)
        marks = tuned["marks"] if tuned else {}
        ends = [int(marks[name]["present_tick"]) for name in ("end_1", "end_2")
                if name in marks]
        if tuned is None or not ends:
            notes.append(f"servo {index}: not tuned (needs an end marked); kept base values")
            continue
        if "neutral" in marks:
            neutral = int(marks["neutral"]["present_tick"])
        elif tuned["neutral_tick"] is not None:
            neutral = int(tuned["neutral_tick"])
            notes.append(f"servo {index}: start never clicked; home is where the latest "
                         f"session started ({neutral})")
        else:
            problems.append(f"servo {index}: no start recorded")
            continue
        if len(ends) == 2:
            low, high = sorted(ends)
            min_tick, max_tick = low + end_margin_ticks, high - end_margin_ticks
            if not min_tick < neutral < max_tick:
                problems.append(f"servo {index}: start {neutral} is not inside the ends "
                                f"{low}..{high} after a {end_margin_ticks}-tick margin")
                continue
        else:
            # Home plus one far end: the range stops exactly at home (the
            # operator's choice, 2026-10-12) and the margin is taken only at
            # the far end.
            end = ends[0]
            if end > neutral:
                min_tick, max_tick = neutral, end - end_margin_ticks
            elif end < neutral:
                min_tick, max_tick = end + end_margin_ticks, neutral
            else:
                problems.append(f"servo {index}: its end {end} is the same as its start")
                continue
            if not min_tick < max_tick:
                problems.append(f"servo {index}: a {end_margin_ticks}-tick margin leaves no "
                                f"range between start {neutral} and end {end}")
                continue
        slot = slots[index]
        slot["home_tick"] = int(neutral)
        slot["min_tick"] = min_tick
        slot["max_tick"] = max_tick
        operating_ma = min(DXL_OPERATING_CURRENT_MAX_MA,
                           tuned["peak_current_ma"] + current_margin_ma)
        slot["operating_current_ma"] = operating_ma
        # Homing currents may not exceed the operating current. Lower them to
        # it rather than leave a profile the firmware would reject.
        for name in ("homing_current_limit_ma", "homing_current_ma"):
            if int(slot[name]) > operating_ma:
                notes.append(f"servo {index}: {name} lowered {slot[name]} -> {operating_ma}")
                slot[name] = operating_ma
        notes.append(f"servo {index}: home {neutral}, range {min_tick}..{max_tick}, "
                     f"current {operating_ma} mA (peak {tuned['peak_current_ma']} mA)")
    return profile, notes, problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base", required=True, help="profile currently uploaded")
    parser.add_argument("--record", required=True, action="append",
                        help="tuning record; repeat for several sessions")
    parser.add_argument("--end-margin-ticks", type=int, required=True,
                        help="how far inside each marked end the limit sits")
    parser.add_argument("--current-margin-ma", type=int, required=True,
                        help="added to the peak tuning load for operating_current_ma")
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args()
    if arguments.end_margin_ticks < 0 or arguments.current_margin_ma < 0:
        parser.error("margins must not be negative")

    with open(arguments.base, encoding="utf-8") as source:
        base = json.load(source)
    records = []
    for path in arguments.record:
        with open(path, encoding="utf-8") as source:
            records.append(json.load(source))
    try:
        servos = merge_records(records)
    except (KeyError, ValueError) as error:
        print(f"record REJECTED: {error}", file=sys.stderr)
        return 1

    profile, notes, problems = propose(base, servos, arguments.end_margin_ticks,
                                       arguments.current_margin_ma)
    for note in notes:
        print(note, file=sys.stderr)
    errors = problems + validate(profile)
    if errors:
        print("proposal REJECTED:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1
    with open(arguments.output, "w", encoding="utf-8") as output:
        output.write(json.dumps(profile, indent=2, sort_keys=True) + "\n")
    print(f"wrote {arguments.output}; review it before upload-profile", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
