# J3 + J4 ten-servo bring-up

Brings indices 10–19 online on branches 2 (J3) and 3 (J4) while J1 and J2 are
out of service. Servo IDs were assigned externally with a DYNAMIXEL U2D2.

**Read `../../hardware/docs/KNOWN_HARDWARE_ISSUES.md` first.** HW-1 (bus
pull-ups on the 3.3 V rail) is open and unmitigated on these branches. It is the
leading explanation for the 74.2% ping success already recorded in
`DEFAULT_DYNAMIXEL_BAUD_CHANGE.md`, and it degrades as a branch is populated.
Expect retries. Treat intermittent failures as suspected bus faults, not servo
faults, until the branch idle level has been measured.

## Expected inventory

| Branch | Connector | Indices | Servo IDs | Operating budget |
|---|---|---|---|---|
| 2 | J3 | 10–14 | 10, 11, 12, 13, 14 | 2000 mA of 2500 mA |
| 3 | J4 | 15–19 | 15, 16, 17, 18, 19 | 2000 mA of 2500 mA |

Firmware requires `servo_id == servo_index` and
`branch_index == servo_index / 5` (`head_config_validate()` in
`../src/config.c`). These IDs are not free choices.

Profile: `../profiles/j3_j4_ten_servo.json` — mask `0xFFC00`, 10 active servos,
`allow_partial_inventory = 1`, `expected_servo_count = 10`. Regenerate or
re-check with:

```sh
python3 firmware/scripts/generate_profile.py --output firmware/profiles/j3_j4_ten_servo.json
python3 firmware/scripts/generate_profile.py --check  firmware/profiles/j3_j4_ten_servo.json
```

## Tooling

Commands below use `firmware/scripts/headctl`, a launcher that resolves paths
from its own location and uses `firmware/.venv/bin/python3` (which carries
pyserial). It works from any directory. Symlink it onto PATH to shorten the
commands:

```sh
ln -sf ~/project_HEAD/firmware/scripts/headctl ~/.local/bin/headctl
```

Do not call `headctl.py` through a working-directory-relative path; it breaks
as soon as you are not standing in the repository root.

## Step 0 — preconditions

- Production image flashed. `/dev/ttyACM0` is the Zephyr console,
  `/dev/ttyACM1` is the head protocol port (the `--port` default).
- **Teensy USB connected.** Per HW-2 the logic rail is USB-powered; 12 V alone
  leaves the bus undefined.
- 12 V supply on, current limit set, servos connected to J3 and J4.
- Torque off. `probe` is rejected by firmware if torque may be on.
- A `FAULT CONFIGURATION` status before the profile upload is expected and is
  not a defect.

## Step 1 — confirm the servos answer at the production baud

`probe` sweeps every ID 0–252 with sequential unicast pings, so it reports what
is actually on the wire rather than what is expected.

```sh
firmware/scripts/headctl --port /dev/ttyACM1 probe --branch 2 --baud 1000000
firmware/scripts/headctl --port /dev/ttyACM1 probe --branch 3 --baud 1000000
```

Expected:

```
J3: id=10 model=1210 firmware=<v>
... through id=14
J4: id=15 model=1210 firmware=<v>
... through id=19
```

**Run each probe two or three times.** Given HW-1, a single sweep can miss a
servo that is present. A servo that appears on some sweeps and not others is a
bus-quality symptom, not a faulty servo.

### If a branch reports nothing at 1 Mbps

The U2D2 session may have set IDs without setting the baud rate. Check:

**Do not use `probe --baud 57600`.** The 254-ID sweep exceeds the one-second
hardware watchdog at that baud and resets the MCU — see
`PROBE_WATCHDOG_LIMIT.md`. Ping the specific IDs instead, which is bounded and
safe at any baud:

```sh
firmware/scripts/headctl --port /dev/ttyACM1 ping-debug --branch 3 --id 15 --baud 57600
```

If the servos appear at 57,600, their Baud Rate register is still at the
factory value. Discovery hard-requires 1 Mbps — `dxl.c` skips any servo whose
Baud Rate(8) is not `DXL_BAUD_1M` (3). Fix it on the U2D2, not in the head:

| Register | Address | Factory | Required |
|---|---|---|---|
| ID | 7 | 1 | 10–19 |
| Baud Rate | 8 | 1 (57,600) | 3 (1 Mbps) |

There is no control-table write path in this firmware — the protocol exposes
`HEAD_MSG_DEBUG_READ` with no write counterpart — so all EEPROM changes must be
made externally.

### If a probe returns `-ENOSPC`

More than five servos replied on one branch. `head_dxl_probe_branch()` caps at
`HEAD_SERVOS_PER_BRANCH`. Most likely a duplicate ID or a servo on the wrong
branch. Unplug and isolate; do not proceed.

## Step 2 — spot-check a register before committing

Confirm the read path is healthy on each branch, torque-off, with one servo:

```sh
firmware/scripts/headctl --port /dev/ttyACM1 read-register --branch 2 --id 10 --address 64 --length 1   # Torque Enable, expect 0
firmware/scripts/headctl --port /dev/ttyACM1 read-register --branch 3 --id 15 --address 64 --length 1
```

## Step 3 — upload the profile

`upload-profile` acquires its own lease, stages the header and every active
slot, commits, waits for persistence, and re-reads to confirm the stored
profile matches.

```sh
firmware/scripts/headctl --port /dev/ttyACM1 upload-profile --file firmware/profiles/j3_j4_ten_servo.json
```

Success prints `profile accepted and queued for persistence` then
`profile committed at generation <n>`. Any of `slot <i> rejected`,
`profile commit rejected`, or `readback did not match` means nothing durable
was stored — fix the cause and repeat rather than retrying blind.

## Step 4 — verify

```sh
firmware/scripts/headctl --port /dev/ttyACM1 export-profile --file /tmp/readback.json
python3 firmware/scripts/generate_profile.py --check /tmp/readback.json
firmware/scripts/headctl --port /dev/ttyACM1 diagnostics
```

`--check` should report `10 active servos, mask 0xFFC00` with 2000 mA on
branches 2 and 3.

## Stop here

**Do not run `headctl home`.** The homing fields in this profile are bring-up
placeholders, not commissioned mechanical values — `generate_profile.py` prints
this warning on every generation. Homing against placeholder travel limits,
currents, and following-error thresholds on strung tendons can drive against a
mechanical end.

Also still open from `../../hardware/docs/HARDWARE_FIRMWARE_REVIEW.md`:
per-servo mechanical and current calibration, physical timing verification, the
3S LiPo overvoltage path, and the absence of an E-stop. Commissioning homing
values is a separate, per-servo, one-at-a-time exercise.
