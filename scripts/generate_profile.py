#!/usr/bin/env python3
"""Generate and validate a calibration v4 profile for headctl upload-profile.

The default target is the ten-servo J3+J4 inventory (indices 10-19) used while
J1 and J2 are out of service. Validation mirrors head_config_validate() in
firmware/src/config.c so a profile can be rejected here instead of on the
device.

The homing fields are conservative bring-up placeholders, not commissioned
mechanical values. See the WARNING printed on generation.
"""
import argparse
import json
import math
import sys

HEAD_SERVO_COUNT = 20
HEAD_BRANCH_COUNT = 4
HEAD_SERVOS_PER_BRANCH = 5
HEAD_BRANCH_CURRENT_BUDGET_MA = 2500
HEAD_DXL_POSITION_MIN_TICK = -1048575
HEAD_DXL_POSITION_MAX_TICK = 1048575
DXL_OPERATING_CURRENT_MAX_MA = 910

# Indices 10-14 are J3, 15-19 are J4.
DEFAULT_ACTIVE_SERVO_MASK = 0xFFC00

# Measured unloaded draw is about 400 mA per servo. Five per branch sums to
# 2000 mA, inside the 2500 mA budget and the 3 A branch fuse.
OPERATING_CURRENT_MA = 400
HOMING_CURRENT_MA = 120
HOMING_CURRENT_LIMIT_MA = 200


def joint_slot(servo_index: int, is_active: bool) -> dict:
    slot = {
        "index": servo_index,
        "branch": servo_index // HEAD_SERVOS_PER_BRANCH,
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


def build_profile(active_servo_mask: int) -> dict:
    active_count = bin(active_servo_mask).count("1")
    return {
        "version": 4,
        "expected_servo_count": active_count,
        "allow_partial_inventory": 1 if active_count < HEAD_SERVO_COUNT else 0,
        "thermal_start_c": 45,
        "thermal_full_c": 60,
        "active_servo_mask": active_servo_mask,
        "joints": [joint_slot(index, bool(active_servo_mask & (1 << index)))
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
    branch_current_ma = [0] * HEAD_BRANCH_COUNT
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
        expected_branch = servo_index // HEAD_SERVOS_PER_BRANCH
        if int(slot["branch"]) != expected_branch:
            errors.append(f"{label}: branch must be {expected_branch}")
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

        branch_current_ma[expected_branch] += max(operating_current_ma, 0)

    for branch_index, total_ma in enumerate(branch_current_ma):
        if total_ma > HEAD_BRANCH_CURRENT_BUDGET_MA:
            errors.append(f"branch {branch_index}: operating current {total_ma} mA "
                          f"exceeds the {HEAD_BRANCH_CURRENT_BUDGET_MA} mA budget")

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
                        default=DEFAULT_ACTIVE_SERVO_MASK,
                        help="active_servo_mask (default: 0xFFC00, indices 10-19)")
    parser.add_argument("--output", help="write the profile here instead of stdout")
    parser.add_argument("--check", help="validate an existing profile and exit")
    arguments = parser.parse_args()

    if arguments.check:
        with open(arguments.check, encoding="utf-8") as source:
            profile = json.load(source)
    else:
        profile = build_profile(arguments.mask)

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
        total_ma = sum(int(slot["operating_current_ma"])
                       for slot in profile["joints"]
                       if int(profile["active_servo_mask"]) & (1 << int(slot["index"]))
                       and int(slot["index"]) // HEAD_SERVOS_PER_BRANCH == branch_index)
        if total_ma:
            print(f"  branch {branch_index} (J{branch_index + 1}): {total_ma} mA "
                  f"of {HEAD_BRANCH_CURRENT_BUDGET_MA} mA", file=sys.stderr)
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
