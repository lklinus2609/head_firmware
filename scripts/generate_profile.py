#!/usr/bin/env python3
"""Generate and validate a calibration v4 profile for headctl upload-profile.

The default target is the ten-servo J3+J4 inventory (indices 10-19) used while
J1 and J2 are out of service. Validation mirrors head_config_validate() in
firmware/src/config.c so a profile can be rejected here instead of on the
device.

Any servo ID may sit on any branch, up to five per branch. With --probe-output
the active servos and their branches are taken from saved `headctl probe`
output, so re-cabling is: probe every branch, regenerate, upload:

    for branch in 0 1 2 3; do headctl probe --branch $branch --baud 1000000; done > probe.txt
    scripts/generate_profile.py --probe-output probe.txt --output profile.json

The homing fields are conservative bring-up placeholders, not commissioned
mechanical values. See the WARNING printed on generation.
"""
import argparse
import json
import math
import re
import sys

HEAD_SERVO_COUNT = 20
HEAD_BRANCH_COUNT = 4
HEAD_SERVOS_PER_BRANCH = 5
# Measured branch current above this faults the head (firmware live check,
# HEAD_BRANCH_CURRENT_BUDGET_MA). Configured totals are not limited.
HEAD_BRANCH_CURRENT_BUDGET_MA = 2500
HEAD_DXL_POSITION_MIN_TICK = -1048575
HEAD_DXL_POSITION_MAX_TICK = 1048575
DXL_OPERATING_CURRENT_MAX_MA = 910

# Indices 10-14 are J3, 15-19 are J4.
DEFAULT_ACTIVE_SERVO_MASK = 0xFFC00

# Measured unloaded draw is about 400 mA per servo.
OPERATING_CURRENT_MA = 400
HOMING_CURRENT_MA = 120
HOMING_CURRENT_LIMIT_MA = 200


# One line of `headctl probe` output, e.g. "J2: id=15 model=1210 firmware=53".
PROBE_LINE = re.compile(r"^J([1-4]): id=(\d+)\b")


def joint_slot(servo_index: int, is_active: bool, branch_index: int | None = None) -> dict:
    if branch_index is None:
        branch_index = servo_index // HEAD_SERVOS_PER_BRANCH
    slot = {
        "index": servo_index,
        "branch": branch_index,
        "id": servo_index,
        "reserved": 0,
        "min_tick": 0,
        "max_tick": 4095,
        "home_tick": 2048,
        "max_position_fraction_per_control_cycle": 0.005,
        "max_acceleration_fraction_per_control_cycle": 0.001,
        "homing_direction": 1.0,
        "reserved_homing_start_tick": 0,
        # Zero homing drives to absolute tick 0, not the nearest encoder
        # zero, and set_homing_goal() faults when the datum is farther than
        # this from where the servo started. Post-power-up Present Position
        # is the single-turn reading, so the budget must span a revolution.
        "homing_max_travel_ticks": 4096,
        "homing_timeout_ms": 5000,
        "operating_current_ma": OPERATING_CURRENT_MA,
        "homing_current_ma": HOMING_CURRENT_MA,
        "homing_following_error_ticks": 200,
        "homing_persistence_ms": 50,
        "homing_current_limit_ma": HOMING_CURRENT_LIMIT_MA,
        "homing_speed_ticks_per_second": 200,
        "homing_backoff_ticks": 40,
    }
    if not is_active:
        # Inactive slots are skipped by validation and never staged.
        slot["homing_timeout_ms"] = 0
    return slot


def parse_probe_output(text: str) -> dict[int, int]:
    """Map servo ID to branch index from saved `headctl probe` output."""
    branch_by_id: dict[int, int] = {}
    for line in text.splitlines():
        match = PROBE_LINE.match(line.strip())
        if match is None:
            continue
        branch_index = int(match.group(1)) - 1
        servo_id = int(match.group(2))
        if servo_id >= HEAD_SERVO_COUNT:
            raise ValueError(f"J{branch_index + 1}: id={servo_id} is outside 0-"
                             f"{HEAD_SERVO_COUNT - 1}; reassign it before profiling")
        if servo_id in branch_by_id and branch_by_id[servo_id] != branch_index:
            raise ValueError(f"id={servo_id} answered on both J{branch_by_id[servo_id] + 1} "
                             f"and J{branch_index + 1}; IDs must be unique across the head")
        branch_by_id[servo_id] = branch_index
    if not branch_by_id:
        raise ValueError("no `Jn: id=N` lines found in the probe output")
    return branch_by_id


def build_profile(active_servo_mask: int,
                  branch_by_index: dict[int, int] | None = None) -> dict:
    branch_by_index = branch_by_index or {}
    active_count = bin(active_servo_mask).count("1")
    return {
        "version": 4,
        "expected_servo_count": active_count,
        "allow_partial_inventory": 1 if active_count < HEAD_SERVO_COUNT else 0,
        "thermal_start_c": 45,
        "thermal_full_c": 60,
        "active_servo_mask": active_servo_mask,
        "joints": [joint_slot(index, bool(active_servo_mask & (1 << index)),
                              branch_by_index.get(index))
                   for index in range(HEAD_SERVO_COUNT)],
    }


def validate(profile: dict) -> list[str]:
    """Mirror of head_config_validate(); returns a list of rejection reasons."""
    errors: list[str] = []
    active_servo_mask = int(profile["active_servo_mask"])
    expected_servo_count = int(profile["expected_servo_count"])

    if int(profile["version"]) != 4:
        errors.append("version must be 4")
    if not 0 < expected_servo_count <= HEAD_SERVO_COUNT:
        errors.append("expected_servo_count out of range")
    if int(profile["thermal_start_c"]) >= int(profile["thermal_full_c"]):
        errors.append("thermal_start_c must be below thermal_full_c")
    if active_servo_mask == 0 or (active_servo_mask & ~0x000FFFFF) != 0:
        errors.append("active_servo_mask empty or out of range")

    slots = {int(slot["index"]): slot for slot in profile["joints"]}
    branch_servo_count = [0] * HEAD_BRANCH_COUNT
    active_count = 0

    for servo_index in range(HEAD_SERVO_COUNT):
        if not (active_servo_mask & (1 << servo_index)):
            continue
        active_count += 1
        if servo_index not in slots:
            errors.append(f"slot {servo_index} active but missing")
            continue
        slot = slots[servo_index]
        label = f"slot {servo_index}"
        branch_index = int(slot["branch"])
        if not 0 <= branch_index < HEAD_BRANCH_COUNT:
            errors.append(f"{label}: branch must be 0-{HEAD_BRANCH_COUNT - 1}")
            branch_index = None
        else:
            branch_servo_count[branch_index] += 1
        if int(slot["id"]) != servo_index:
            errors.append(f"{label}: id must equal index ({servo_index})")

        min_tick = int(slot["min_tick"])
        max_tick = int(slot["max_tick"])
        home_tick = int(slot["home_tick"])
        if min_tick < HEAD_DXL_POSITION_MIN_TICK:
            errors.append(f"{label}: min_tick below encoder range")
        if max_tick > HEAD_DXL_POSITION_MAX_TICK:
            errors.append(f"{label}: max_tick above encoder range")
        if min_tick >= max_tick:
            errors.append(f"{label}: min_tick must be below max_tick")
        if not min_tick <= home_tick <= max_tick:
            errors.append(f"{label}: home_tick outside [min_tick, max_tick]")

        position_fraction = float(slot["max_position_fraction_per_control_cycle"])
        acceleration_fraction = float(slot["max_acceleration_fraction_per_control_cycle"])
        homing_direction = float(slot["homing_direction"])
        if not math.isfinite(position_fraction) or not 0.0 < position_fraction <= 1.0:
            errors.append(f"{label}: max_position_fraction_per_control_cycle out of range")
        if not math.isfinite(acceleration_fraction) or acceleration_fraction <= 0.0:
            errors.append(f"{label}: max_acceleration_fraction_per_control_cycle out of range")
        elif acceleration_fraction > position_fraction:
            errors.append(f"{label}: acceleration fraction exceeds position fraction")
        if not math.isfinite(homing_direction) or homing_direction == 0.0:
            errors.append(f"{label}: homing_direction must be non-zero and finite")

        if int(slot["reserved_homing_start_tick"]) != 0:
            errors.append(f"{label}: reserved_homing_start_tick must be zero")

        homing_max_travel_ticks = int(slot["homing_max_travel_ticks"])
        if not 0 < homing_max_travel_ticks <= (HEAD_DXL_POSITION_MAX_TICK -
                                               HEAD_DXL_POSITION_MIN_TICK):
            errors.append(f"{label}: homing_max_travel_ticks out of range")
        if int(slot["homing_timeout_ms"]) == 0:
            errors.append(f"{label}: homing_timeout_ms must be non-zero")

        operating_current_ma = int(slot["operating_current_ma"])
        homing_current_ma = int(slot["homing_current_ma"])
        homing_current_limit_ma = int(slot["homing_current_limit_ma"])
        if not 0 < operating_current_ma <= DXL_OPERATING_CURRENT_MAX_MA:
            errors.append(f"{label}: operating_current_ma out of range")
        if homing_current_ma <= 0 or homing_current_ma > operating_current_ma:
            errors.append(f"{label}: homing_current_ma out of range")
        if homing_current_limit_ma < homing_current_ma:
            errors.append(f"{label}: homing_current_limit_ma below homing_current_ma")
        if homing_current_limit_ma > operating_current_ma:
            errors.append(f"{label}: homing_current_limit_ma above operating_current_ma")
        if int(slot["homing_following_error_ticks"]) < 0:
            errors.append(f"{label}: homing_following_error_ticks must not be negative")
        if int(slot["homing_persistence_ms"]) == 0:
            errors.append(f"{label}: homing_persistence_ms must be non-zero")

        homing_speed = int(slot["homing_speed_ticks_per_second"])
        if not 0 < homing_speed <= 50000:
            errors.append(f"{label}: homing_speed_ticks_per_second out of range")
        homing_backoff_ticks = int(slot["homing_backoff_ticks"])
        if not 0 < homing_backoff_ticks <= homing_max_travel_ticks:
            errors.append(f"{label}: homing_backoff_ticks out of range")

    for branch_index, servo_count in enumerate(branch_servo_count):
        if servo_count > HEAD_SERVOS_PER_BRANCH:
            errors.append(f"branch {branch_index} (J{branch_index + 1}): {servo_count} active "
                          f"servos exceeds {HEAD_SERVOS_PER_BRANCH}")

    if active_count != expected_servo_count:
        errors.append(f"expected_servo_count {expected_servo_count} does not match "
                      f"{active_count} active servos")
    if not int(profile["allow_partial_inventory"]) and active_count != HEAD_SERVO_COUNT:
        errors.append("partial inventory requires allow_partial_inventory = 1")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mask", type=lambda value: int(value, 0),
                        help="active_servo_mask (default: 0xFFC00, indices 10-19)")
    parser.add_argument("--probe-output",
                        help="saved `headctl probe` output ('-' for stdin); activates "
                             "every servo found, on the branch it answered on")
    parser.add_argument("--output", help="write the profile here instead of stdout")
    parser.add_argument("--check", help="validate an existing profile and exit")
    arguments = parser.parse_args()

    if arguments.probe_output and (arguments.mask is not None or arguments.check):
        parser.error("--probe-output cannot be combined with --mask or --check")
    if arguments.check:
        with open(arguments.check, encoding="utf-8") as source:
            profile = json.load(source)
    elif arguments.probe_output:
        if arguments.probe_output == "-":
            probe_text = sys.stdin.read()
        else:
            with open(arguments.probe_output, encoding="utf-8") as source:
                probe_text = source.read()
        try:
            branch_by_id = parse_probe_output(probe_text)
        except ValueError as error:
            print(f"probe output REJECTED: {error}", file=sys.stderr)
            return 1
        active_servo_mask = sum(1 << servo_id for servo_id in branch_by_id)
        profile = build_profile(active_servo_mask, branch_by_id)
    else:
        mask = DEFAULT_ACTIVE_SERVO_MASK if arguments.mask is None else arguments.mask
        profile = build_profile(mask)

    errors = validate(profile)
    if errors:
        print("profile REJECTED:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    active_count = bin(int(profile["active_servo_mask"])).count("1")
    print(f"profile valid: {active_count} active servos, "
          f"mask 0x{int(profile['active_servo_mask']):05X}", file=sys.stderr)
    for branch_index in range(HEAD_BRANCH_COUNT):
        slots = [slot for slot in profile["joints"]
                 if int(profile["active_servo_mask"]) & (1 << int(slot["index"]))
                 and int(slot["branch"]) == branch_index]
        if slots:
            total_ma = sum(int(slot["operating_current_ma"]) for slot in slots)
            servo_ids = ",".join(str(int(slot["id"])) for slot in slots)
            print(f"  branch {branch_index} (J{branch_index + 1}): ids {servo_ids}, "
                  f"{total_ma} mA configured (measured above "
                  f"{HEAD_BRANCH_CURRENT_BUDGET_MA} mA faults the head)", file=sys.stderr)
    if arguments.check:
        return 0

    print("WARNING: homing fields are bring-up placeholders, not commissioned "
          "mechanical values. Do not run `headctl home` on tendons with this "
          "profile.", file=sys.stderr)

    text = json.dumps(profile, indent=2, sort_keys=True)
    if arguments.output:
        with open(arguments.output, "w", encoding="utf-8") as output:
            output.write(text + "\n")
        print(f"wrote {arguments.output}", file=sys.stderr)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
