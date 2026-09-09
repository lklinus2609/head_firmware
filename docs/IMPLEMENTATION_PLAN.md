# Implementation and validation plan

This plan follows the 2026-08-20 scope decisions and the fabricated PCB. Work
stops at an acceptance gate when its evidence is missing; later phases must not
quietly assume an earlier safety property.

## Rate policy

The implementation target is a 500 Hz MCU control scheduler and at least
100 Hz complete feedback. Performance is selected from measured tiers:

| Tier | Control | Feedback | Use |
|---|---:|---:|---|
| A | 500 Hz | 100 Hz or higher | Preferred production target |
| B | 250 Hz | 100 Hz or higher | Accepted fallback if Tier A misses deadlines |
| C | 100 Hz | 100 Hz | Minimum acceptable natural-expression baseline |

The firmware advertises the selected tier. It must never claim 500 Hz merely
because a configuration constant says 500.

Implementation snapshot: the 500 Hz absolute scheduler, concurrent DMA-backed
branch writes, exclusive telemetry slots, v2 position protocol, measured
velocity feedback, fail-safe shutdown, servo-mode verification, actuator bus
watchdog, and fan PWM/tach paths are now in code and pass the production build.
Their phase gates remain open until fabricated-board measurements and fault
injection tests provide the listed evidence.

The 2026-08-21 official-API repair also waits for the i.MX RT LPUART hardware
transmission-complete flag before releasing a half-duplex driver, stages a
complete five-servo telemetry train, verifies torque/watchdog writes by
readback, and removes the ROS action callback's dependency on an asyncio loop.

## Phase 0 — freeze facts and keep torque off

- [x] Record the authoritative PCB revision and ERC/DRC status.
- [x] Record current hardware and project scope.
- [x] Cross-check connector nets, UARTs, direction pins, buffers, fan, and power
  paths against firmware.
- [x] Select an external regulated supply instead of direct 3S LiPo power for
  initial bring-up.
- [x] Record the sequential branch/ID map and co-actuated groups `{9, 12, 14}`
  and `{8, 17, 19}`.
- [x] Record the external supply rating: nominal 12 V, approximately 9 A
  maximum, with an adjustable current limit.
- [ ] Measure its static output and turn-on behavior; the servo rail must not
  exceed 12.0 V. Prefer a slightly lower setting when adjustment permits.
- [ ] Record the exact fuse part and initial conservative mechanical limits.

**Gate 0:** No servo is connected to the battery rail until its maximum voltage
is compliant. No tendon is attached during initial communication testing.

## Phase 1 — fix the no-motion safety foundation

1. Correct direction control to GPIO4.6, GPIO2.1, GPIO1.17, and GPIO2.18 and add
   explicit pinmux for Teensy pins 4, 12, 18, and 36.
2. Assert every direction pin low before UART activity and after every send,
   timeout, fault, and reset path.
3. Implement Protocol 2.0 transmit byte stuffing, length update, and CRC over
   the stuffed packet. Add official packet-vector tests.
4. Replace transition-dependent torque-off with an idempotent global shutdown
   operation. It broadcasts torque-disable on all four branches, releases every
   driver, sets fan full, latches the root fault, and records any failed branch.
5. Make Disable, lease release while enabled, watchdog expiration, homing
   abort, and every fault use that operation.
6. Add unit tests for lifecycle transitions, malformed commands, calibration
   validation, packet stuffing/CRC, and global shutdown requests.

Code status: items 1–5 are implemented. Shutdown now verifies Torque Enable on
every active servo instead of treating broadcast transmission as an ACK. The
host test covers malformed and maximum-size v2 frames; official Dynamixel
stuffing vectors and lifecycle/shutdown tests remain.

**Gate 1 evidence:** production and no-12 V builds pass; GPIO line-mode tests
match all four PCB direction pins with a multimeter; no code path can leave a
direction output high or a fault with software-only torque disable.

## Phase 2 — commission actuator configuration safely

1. Add a torque-off readback command for model, firmware, ID, baud, operating
   mode, drive mode, Return Delay Time, Status Return Level, Current Limit,
   velocity/profile limits, Bus Watchdog, Shutdown, and position limits.
2. Keep arbitrary EEPROM changes in an explicit commissioning flow. The two
   project-wide settings already confirmed by the user—mode 5 and zero Return
   Delay Time—are corrected torque-off only when mismatched and then read back.
3. Configure current-based position control and a conservative initial Current
   Limit/Goal Current. Exact values come from mechanical measurements.
4. Give return delays deterministic values and enable the actuator Bus Watchdog
   with a timeout longer than the intended bus update interval but short enough
   to stop after controller/bus loss.
5. Provision the eight newly received servos to unique IDs 12–19 and 1 Mbps
   one at a time before placing them on multidrop branches. Record model and
   firmware version during this isolated-servo step.

Code status: discovery requires firmware v38 or later and verifies XC330-T181
model 1210, baud, drive safety bit, Protocol 2.0, torque-off, Status Return
Level 2, mode 5, zero return delay/profile values, required Shutdown bits,
sufficient Current Limit, and a cleared watchdog. On firmware v46 or later it
conditionally clears Startup Torque On and reads it back. Calibration v4 adds
separate operating/homing current limits, configured homing speed and contact
backoff. Firmware arms and verifies the 200 ms RAM Bus Watchdog before torque
enable. Full incoming-servo ID/baud provisioning remains.

**Gate 2 evidence:** each disconnected-mechanics servo has an exported
configuration record; IDs are unique; torque remains off; settings survive a
power cycle and match the reviewed profile.

## Phase 3 — deterministic branch scheduler

1. Introduce one transaction owner/state machine per physical UART. A branch is
   either released, transmitting control, awaiting telemetry, or recovering.
2. Replace poll-plus-busy-wait output with interrupt/DMA-backed transmission or
   a verified hardware transmission-complete event.
3. Start the four branch control writes together or from a tightly bounded
   dispatch window rather than waiting serially for each branch.
4. Use absolute periodic deadlines (`next += period`) instead of sleeping after
   work. Record start jitter, execution time, overrun, and consecutive misses.
5. Schedule telemetry in reserved branch slots. A branch may never receive a
   control packet while replies are outstanding.
6. Parse Present Velocity and correct feedback timestamp/age semantics.
7. Measure Tier A first, then deliberately select Tier B or C if Tier A cannot
   sustain the acceptance run.

Code status: items 1–6 are implemented, including absolute deadlines, four
parallel UART DMA transmissions, hardware-complete OE release, a single bus
owner, a 640-byte per-branch telemetry stage, Present Velocity, and true
feedback age. Timing acceptance and deliberate tier fallback remain.

**Gate 3 evidence:** with representative five-servo branches, control and
telemetry show no collisions; complete feedback is at least 100 Hz; selected
control tier runs for 30 minutes with no unexplained deadline miss or stale
servo. Record period/jitter histograms and bus-error counters.

## Phase 4 — safe current-threshold homing

1. Extend the calibration profile with a homing batch/conflict identifier.
   Encode `{9, 12, 14}` and `{8, 17, 19}` as two conflict groups. Default first
   commissioning to one servo per batch.
2. Read present position before enabling torque and seed each goal from that
   measurement.
3. Enable torque only for the active independent batch. Co-actuated servos are
   never in the same batch.
4. Advance at a low configured rate until current exceeds threshold for a
   persistence interval. Use following error as optional confirmation, not as
   an unbounded force request.
5. Enforce maximum travel, time, current, temperature, and communication limits
   throughout the search.
6. On any failure, execute whole-head shutdown. Never continue with the next
   tendon after a failed home.
7. Expand concurrency only after individual tendons pass and the action-point
   map proves the batch independent.

Code status: present-position seeding, configured low-current/speed search,
contact backoff, sequential single-servo torque selection, torque-off READY,
and whole-head fault shutdown are implemented. HIL validation remains.

**Gate 4 evidence:** each servo homes individually at low force with repeatable
current/reference measurements; co-actuated servos are demonstrably
sequential; power removal stops force; every injected timeout/error disables
the whole head.

## Phase 5 — command and feedback protocol v2

1. Add protocol/capability negotiation and retain explicit rejection of
   incompatible `v1` frames.
2. Implement `POSITION`, then `POSITION_VELOCITY`, then bounded `VELOCITY`.
3. Keep the control loop on the Teensy. Host messages update the latest
   trajectory input and do not directly clock Dynamixel transactions.
4. Add command source time, applied sequence, selected control tier, measured
   velocity, and scheduling statistics to state/diagnostics.
5. Update ROS messages, bridge validation, `headctl`, example publisher, and
   rosbag-oriented metadata together.
6. Add replay tests for missing, duplicated, stale, non-finite, out-of-range,
   and wrong-mode commands.

Code status: wire/ROS v2, fixed mode field, velocity array, applied sequence,
control rate, measured velocity, and period diagnostics are implemented.
`POSITION` rejects malformed/non-finite/out-of-range input. Jazzy actions now
advance through a ROS timer and `rclpy` Future, and the bridge synchronizes its
active mask from the controller profile. An additive capability and confirmed
command enter a post-homing static proprioception hold: after a two-second
minimum settling state, fixed backoff goals continue to be refreshed while
joint targets are rejected and full state feedback remains active. The two
velocity modes remain explicitly rejected pending the control law and replay
tests. Additive routing mode 10 can enter before homing, seeds torque-on at the
measured positions, ramps all active goals to raw tick 0 through the normal
trajectory limiter, rejects streamed targets, and holds until verified Disable
or a watchdog/fault shutdown.

**Gate 5 evidence:** desktop model/fixture can command position-only and
position-plus-velocity without changing firmware; velocity-only cannot leave
the calibrated workspace; stale or malformed input reaches global shutdown by
the documented watchdog policy.

## Phase 6 — cooling and power behavior

1. Implement 25 kHz fan PWM accounting for the PCB's inverting transistor.
2. Capture tachometer edges and convert the fan's pulse count to RPM.
3. Define startup grace, minimum command, stall persistence, and whole-head
   fault behavior. Missing thermal feedback commands maximum cooling.
4. Measure the Jetson 9 V rail at maximum selected power mode across the usable
   battery range. Set an external cutoff/alert above the point where regulator
   dropout or wiring transients violate the carrier input range.
5. Record fan RPM, 9 V minimum/maximum, regulator temperature, and reset events.

Code status: 25 kHz inverted PWM, two-pulse/revolution tach conversion,
two-second stall persistence, maximum cooling on stale telemetry, and a
whole-head fan fault are implemented. Electrical and HIL measurements remain.

**Gate 6 evidence:** fan duty and RPM agree at several setpoints; unplugged or
stalled fan produces the documented fault; Jetson does not reset during the
worst intended servo motion and inference load.

## Minimal-equipment bring-up sequence

The available equipment is an assembled head, multimeter, and occasional
current-limited supply. Use the current-limited supply only at the high-value
first-power stages, but do not omit it there.

1. USB only: build, enumerate both CDC ports, exercise framing and lifecycle
   tests with the no-12 V image.
2. PCB, no servos: verify battery polarity, each fused connector voltage, 9 V,
   5 V, and 3.3 V. Verify all four direction nets idle low and respond to the
   line-mode test.
3. Current-limited supply, one mechanically disconnected servo: start at no
   more than 1 A and verify ping, readback, torque-off, low-current position
   hold, watchdog, and physical power removal.
4. Repeat one servo per branch, then populate one complete branch. Raise the
   supply limit only in measured stages; the 9 A rating is capacity, not the
   default commissioning limit.
5. Borrow an oscilloscope or suitable logic analyzer for one session to capture
   all four UART turnarounds, the longest telemetry train, scheduler timing, and
   fan PWM. A multimeter cannot close those acceptance items.
6. Assembled head: begin at the lowest useful current, home one servo at a time,
   record thresholds, then validate only proven-independent parallel batches.
7. Run the 30-minute selected-rate acceptance test while logging state,
   diagnostics, battery voltage, and Jetson resets.

## Remaining user decisions

- Measured external-supply tolerance/overshoot, staged current-limit settings,
  and later battery-safe power solution and cutoff.
- Initial current and motion limits from low-force measurements.
- Per-servo calibration v4 operating/homing current, speed, and backoff values;
  `operating_current_ma` remains 1–910 mA and not below the homing threshold.
  Do not copy generic values across the head without low-force measurements.
- Quantitative end-to-end latency and jitter limits after baseline data exists.
- A project-owned USB VID/PID before firmware distribution.
