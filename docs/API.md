# Head controller interface contract

The USB wire and ROS interfaces are protocol `v2`. It was updated on
2026-08-26 from the fabricated PCB, firmware review, and current project
decisions. `POSITION` is implemented; the other declared modes remain
fail-closed until their trajectory semantics and tests are approved.

## Coordinate and mode decisions

The interface is actuator-space: one `q`/`qdot` element addresses one tendon
servo. Kinematics that turn facial/joint intent into tendon-actuator requests
belong outside the Teensy firmware.

The safe calibrated coordinate is used at the researcher boundary:

- `q`: normalized actuator position, where 0.0 and 1.0 are the reviewed
  per-servo workspace endpoints.
- `qdot`: normalized actuator velocity in calibrated-range units per second.
- Firmware converts `q` to Dynamixel ticks using the active calibration
  profile. The reserved velocity modes will convert `qdot` to ticks/second once
  their control law is enabled. Raw ticks, velocity, and current remain
  available in feedback for analysis and debugging.

This is the confirmed model-facing convention. It preserves the existing
position API and prevents model code from depending on a particular pulley
ratio or encoder zero.

The command modes are:

| Mode | Required input | Semantics | Priority |
|---|---|---|---|
| `POSITION` | `q` | Bounded position request; firmware supplies its normal rate/acceleration limits. | First |
| `POSITION_VELOCITY` | `q`, `qdot` | Reserved in v2; currently rejected. | Second |
| `VELOCITY` | `qdot` | Reserved in v2; currently rejected. | Later |

All three host modes drive servos configured in Dynamixel current-based
position control. “Velocity-only” here does not mean changing the actuator to
Dynamixel velocity operating mode.

## Timing contract

- The Teensy owns a fixed-period control scheduler. The target is 500 Hz.
- A measured 250 Hz or 100 Hz control configuration may be selected if 500 Hz
  cannot meet its deadlines with 100 Hz feedback. The selected rate is exposed
  as a capability and recorded in diagnostics.
- The host does not have to publish at exactly the MCU control frequency. Each
  accepted command becomes the latest trajectory input; the Teensy holds or
  interpolates it at its fixed rate.
- The host command watchdog remains independent of the loop period. Exact
  timeout and latency acceptance values are not yet frozen.
- Complete active-servo state is published at a minimum of 100 Hz.
- Control and telemetry must use exclusive, scheduled transactions on each
  half-duplex UART. A control write may not begin while that branch is awaiting
  telemetry responses.

## Safety and ownership

- Only one control lease is accepted. It expires after 1,000 ms unless a valid
  target or `RENEW_LEASE` request refreshes it. Expired tokens are invalid for
  motion, lifecycle, fan, fault-clear, and configuration operations.
- Lease expiry while actuator torque is enabled latches a watchdog fault and
  starts the verified all-branch torque-disable sequence. This applies during
  homing, maintenance calibration, routing hold, proprioception hold, and
  streamed motion.
- Normal commands are accepted only in ENABLED, after successful discovery
  and either homing (`HOME`, then `ENABLE` from READY) or
  `ENABLE_AT_PRESENT_POSE` (no homing, see below).
- On entry to ENABLED, by either path, the firmware sets every active servo's
  requested `q` to its freshly read position, so Enable itself commands no
  motion. If any active servo is outside its position range at that moment,
  it latches `CONFIGURATION` and removes torque instead of clamping.
- Commands contain a monotonic sequence number and the complete active mask.
- Firmware currently validates the complete mask, sequence, mode, finiteness,
  and normalized position workspace before applying a position request.
- Any confirmed serious controller, servo, bus, telemetry, watchdog, homing,
  or cooling fault requests torque-disable on all active branches and latches
  `FAULT`.
- The sum of `operating_current_ma` for the active servos on a branch is not
  limited (decided 2026-10-06; it was 2,500 mA, checked at profile upload and
  before torque-on). While torque may be on, each
  branch is checked as soon as all of its active servos have fresh current
  telemetry; exceeding 2,500 mA latches `BRANCH_CURRENT_BUDGET` and requests
  verified whole-head torque shutdown. The nominal threshold is 500 mA below
  each branch's 3 A fuse rating. However, this is a sum of actuator-reported
  motor current rather than a direct branch-supply measurement, so wiring
  faults and fast input-current transients still rely on the fuse and physical
  supply disconnect. There is no whole-head servo-current admission or trip
  limit; four branches may therefore have up to 10,000 mA of configured servo
  current in aggregate.
- `START_PROPRIOCEPTION` is accepted only from torque-off `READY` with a valid
  lease and fresh actuator feedback. It seeds and verifies fresh measured positions before torque-on, rejects joint targets, and keeps refreshing those fixed goals while
  feedback continues. `DISABLE`, lease loss, or any fault exits through the
  normal verified whole-head torque shutdown.
- `START_ROUTING` is accepted from torque-off `HOMING_REQUIRED` or `READY`
  with a valid lease and fresh feedback. It seeds every active goal to the
  measured position before torque-on, then applies the normal acceleration and
  rate limiter until every goal reaches raw tick 0, the Dynamixel 0-degree
  encoder position. It rejects joint targets and continuously refreshes tick
  0. Entry is rejected if tick 0 is outside any active servo's calibrated
  limits. `DISABLE`, lease loss, or any fault uses the same verified shutdown.
- `START_TUNING` is accepted only from torque-off `HOMING_REQUIRED`, with a
  valid lease, fresh feedback, a valid profile, and a Goal Current limit no
  higher than the highest active `operating_current_ma`. No homing is needed.
  Preparation sets each active servo's Goal Current(102) to that limit or its
  own `operating_current_ma`, whichever is lower (since 2026-10-20, so one
  joint such as the jaw can be allowed more than the others; before, the
  limit had to be within every servo's and applied to all), and the step
  refusal below uses that same per-servo cap. It seeds every goal to the
  measured position, so entering `TUNING=11` commands
  no motion. Those goals are the tuning neutral. `TUNING_STEP` then moves one
  servo by 1 to 50 ticks (`HEAD_TUNING_MAX_STEP_TICKS`). A step toward neutral is always accepted. A step away from
  neutral is refused with `-ERANGE` beyond one revolution of travel, and with
  `-EBUSY` while the servo's current has reached the limit or it trails its
  goal by the limit divided by `HEAD_TUNING_LAG_CURRENT_MA_PER_TICK` (3 mA per
  tick; never less than `HEAD_TUNING_MIN_LAG_TICKS`, 20). With Position I = 0
  the servo's current is proportional to that lag, so the lag bound is reached
  no earlier than the current limit; until 2026-10-10 it was a fixed 20 ticks,
  which stopped steps at about 70 mA of load whatever limit was set. Other refusals: `-EAGAIN` (feedback older
  than 30 ms), `-EINVAL` (inactive servo, or a step of 0 or over 50 ticks), `-EPERM` (not
  in tuning). `DISABLE`, lease loss, or any fault uses the verified shutdown.
- `ENABLE_AT_PRESENT_POSE` is accepted only from torque-off `HOMING_REQUIRED`
  with a valid lease, fresh feedback and a valid profile. No homing happens:
  any homing reference from earlier in the boot is dropped, and `q` 0..1 maps
  to the profile's raw `min_tick..max_tick`. Every active servo's present
  tick must already be inside that range, or the request is refused with
  `-ERANGE`. Preparation then seeds goals and `q` from the measured
  positions, so the head holds where it is. The profile range is the only
  limit on where targets can go, so it must come from tuning. This depends on
  raw multi-turn ticks being the same as when the profile was made.
- Software torque-disable is not an emergency stop. The operator must be able
  to remove the actuator supply.
- The actuator-side Dynamixel Bus Watchdog stops motion after communication
  loss; it does not guarantee Torque Enable becomes zero. Independent
  de-energization requires an external actuator-power disconnect.
- Home/Enable/hold ACKs accept a queued preparation request. Preparation verifies
  torque-off, watchdog/current settings, and a fresh goal in bounded steps;
  state/torque readback establishes completion. Each step and scheduling gap is
  limited to 20 ms, the operation to 1.5 seconds, and lease expiry remains active.
  Ordinary energized control retains its 5 ms fatal scheduling-delay limit.
- Pending/writing storage and electrical diagnostics exclude motion preparation.
  Disable preserves a latched fault and does not bypass explicit rediscovery.
- `COMMIT_CONFIGURATION` ACK means queued acceptance. Successful storage status,
  an advanced generation, and profile readback establish completed persistence.

## Host-facing ROS 2 contract

ROS 2 Jazzy on the desktop is the initial host. The same bridge may later run
on the Jetson. Continuous commands and state use bounded, best-effort topics so
old samples do not queue; lifecycle changes use confirmed services/actions.

ROS action clients must renew their lease during homing and maintenance, too.
Successful homing actions require fresh READY with verified torque-off;
maintenance additionally requires successful storage with an advanced calibration
generation. Disable and timeout responses describe a shutdown request, not proof
of physical torque removal.

The configured branch baud is 1 Mbps. The telemetry response budget is 8 ms
per branch; a five-servo response already exceeds 4 ms before byte stuffing.
The nominal control and feedback rates still require hardware measurement.

Current topics:

| Name | Type | Rate / QoS | Status |
|---|---|---|---|
| `/head/joint_targets` | `head_msgs/JointTargets` | Latest sample, best effort | v2; `POSITION` enabled |
| `/head/state` | `head_msgs/HeadState` | 100 Hz, best effort | v2 |
| `/head/diagnostics` | `head_msgs/HeadDiagnostics` | 1 Hz host publication | v2 |

The confirmed `/head/start_proprioception` service accepts preparation for a
two-second minimum settling state and then `PROPRIOCEPTION_HOLD`; observe the
state stream to confirm activation. The requesting collector owns renewal through `/head/renew_control`; the
bridge does not renew independently of that owner. The settling interval is only a minimum quieting period;
data collection may discard a longer warm-up while silicone creep decays.

The `/head/start_routing` service enters firmware state `ROUTING=10`; the GUI
renews its lease through `/head/renew_control` every 400 ms. Closing or losing
the GUI therefore stops renewal, and the firmware's one-second lease watchdog
faults and disables torque. The GUI's Disable control remains the normal exit.

The `/head/enable_at_present_pose` service (type `head_msgs/Enable`) sends
`ENABLE_AT_PRESENT_POSE`. Its reason names an out-of-range servo refusal
separately from other refusals. As with `/head/enable`, the caller must start
streaming targets as soon as ENABLED is observed.

The `JointTargets.msg` shape is:

```text
std_msgs/Header header
uint32 lease_token
uint32 sequence
uint8 mode
string[] names
float32[] position
float32[] velocity
```

For the implemented `POSITION` mode, velocity is empty and position contains
every active actuator exactly once. The ROS bridge rejects partial, duplicated,
unknown, or wrong-mode requests. It serializes a fixed-size wire payload with
zero velocity values. `POSITION_VELOCITY` and `VELOCITY` are declared for API
stability but rejected by both bridge and firmware.

## Feedback contract

Every 100 Hz v2 state sample includes the MCU timestamp, last command sequence
successfully transmitted to every active branch, configured control rate, the
position range generation (offset 19, a reserved zero byte before 2026-10-08),
and the entropy-derived boot/session identifier, plus
for each servo: goal/present tick,
Present Velocity raw register value, current, voltage, temperature, motion and
hardware-error status, online flag, and computed feedback age. Diagnostics add
the maximum observed control period, deadline misses, and per-branch counters.

The ROS `HeadState` also publishes the bridge's controller-synchronized
`active_servo_mask`. The wire payload retains all 20 fixed slots, so desktop
recorders and models must use this mask rather than interpreting inactive slots
as stale or failed servos.

Desktop experiment recording does not add a firmware lifecycle state. The
collector remains in `PROPRIOCEPTION_HOLD` while it publishes recording/contact
markers on `/proprioception/events`; stop or abort uses the existing confirmed
`/head/disable` service. `/proprioception/session_state` reports the host-only
acquisition lifecycle and is recorded alongside controller telemetry.

The ROS v2 field remains named `applied_sequence` for compatibility. Because
DYNAMIXEL broadcast Sync Write has no Status Packet, its precise meaning is
“transmitted to every active branch,” not physical actuator application. A
future versioned ROS interface will use `transmitted_sequence`.

Still planned for a later compatible extension or v3 are:

- normalized requested/measured velocity and per-servo branch/ID identity;
- source timestamps and measured end-to-end latency;
- full period/jitter histograms and explicit recovery counters.

`feedback_age_ms` is an age computed when the state packet is produced, not the
servo's absolute `last_feedback_ms` timestamp. A separately named MCU timestamp
field should carry absolute controller time. Host receipt time and source time
must not be conflated.

The Sync Read covers addresses 70 through 146. Present Velocity at address 128
is transported as its signed raw register value; its unit is 0.229 rpm.
Present PWM at address 124 is in the separate `PRESENT_PWM` message.

## USB CDC wire transport

USB CDC is the virtual serial device carried by the Teensy's USB cable. The ML
model normally talks to ROS; the bridge owns this lower-level transport.

The implemented `v2` frame is:

```text
offset  size  field
0       1     SOF = 0xa5
1       1     protocol version = 2
2       1     message type
3       2     payload length, little-endian
5       n     payload
5+n     4     CRC-32/ISO-HDLC of bytes 1 through 4+n
```

The implemented fixed command payload is:

```text
u32 lease_token
u32 sequence
u32 active_mask
u8 mode
u8 reserved[3] = 0
20 x f32 normalized_position
20 x f32 normalized_velocity
```

The payload is 176 bytes. Version-1 frames, nonzero reserved bytes, wrong
lengths, and unsupported modes are rejected rather than reinterpreted.

`RENEW_LEASE` is message type 32 and carries one little-endian `u32
lease_token`, followed by the required `u32 transaction_id`. Every confirmed
request ends with a transaction ID; ACK/NACK echoes the request type, result,
current lease token, transaction ID, and boot/session ID. It succeeds only
before the current lease expires and extends
the deadline by another 1,000 ms.

`START_PROPRIOCEPTION` is additive message type 33 and carries the same
`lease_token` plus `transaction_id` layout. Capability bit 8 advertises it.
There is no separate stop message: `DISABLE` is the common safe exit.

`START_ROUTING` is additive message type 34 with the same payload layout.
Capability bit 9 advertises routing hold. It also uses `DISABLE` as its only
normal exit.

`START_TUNING` is additive message type 38: `lease_token` (u32), Goal Current
limit in mA (u16), `transaction_id`. `TUNING_STEP` is type 39: `lease_token`
(u32), servo index (u8), signed step in ticks (i8, 1 to 50 either way),
`transaction_id`; its ACK
or NACK result byte is the step outcome, and an accepted step renews the
lease. Capability bit 10 advertises tuning. The result values are picolibc
errno numbers (EPERM 1, EAGAIN 11, EBUSY 16, EINVAL 22, ERANGE 34).

`ENABLE_AT_PRESENT_POSE` is type 44: `lease_token` (u32), `transaction_id`.
Its ACK/NACK result is 0, `-EPERM` (1) or `-ERANGE` (34, an active servo is
outside its range). Capability bit 13 advertises it. Observe the state stream
for ENABLED; the first target must then arrive within the normal command
timeout.

`GET_POSITION_RANGE` is type 45 and carries only `transaction_id`. The reply
`POSITION_RANGE` (type 46, 176 bytes) is: `transaction_id` (u32), range
generation (u8), three reserved zero bytes, valid mask (u32, active servos with
a usable range), homed mask (u32, servos mapped through a homing reference
rather than the profile range), then for each servo slot 0-19 the ticks that
`q` = 0 and `q` = 1 map to (i32 each, zero when not valid). Capability bit 14
advertises it. The STATE byte at offset 19 carries the same range generation;
it changes whenever a range may have changed (profile applied, homing
reference set or dropped), so a host uses a cached reply only while the two
match. The bridge computes `present_normalized` and `goal_normalized` this
way, unclamped, and publishes NaN while it has no matching range.

`PRESENT_PWM` is additive message type 47, sent right after every STATE
(STATE is already 512 bytes, the frame maximum). The payload is 44 bytes: the
same MCU time as that STATE (u32 ms), then for each servo slot 0-19 the signed
Present PWM(124) register value (i16, unit 0.113 %; the datasheet gives no
range, so it is read as signed like Goal PWM(100)). Like STATE it is low
priority: the USB queue keeps only the newest one, so a host pairs it with the
STATE that has the same MCU time and must not assume every STATE gets one.
Capability bit 15 advertises it. Hosts that do not know type 47 ignore it.

`GET_BLACK_BOX` is type 40 and carries only `transaction_id`. The reply
`BLACK_BOX` (type 41) is: `transaction_id` (u32), kept (u8), boot count (u32),
reset cause (u32, Zephyr `RESET_*` bits), then from the previous boot the
`main.c` line holding `head_lock` (u32, 0 = free), when it was taken (u32 ms),
the last watchdog feed (u32 ms), the control-thread phase and when it began
(u32 each), and the last eight lock takers, oldest first (line u32, ms u32).
The record lives in uncached no-init DTCM; kept = 0 means it did not survive the reset
(power-on or flashing). Capability bit 11 advertises it. `headctl blackbox`
prints it.

`GET_BUS_STATS` is type 42 and carries only `transaction_id`. The reply
`BUS_STATS` (type 43, 508 bytes) is `transaction_id` (u32), then for each branch
J1-J4 twenty-six u32 counts since boot, then one u32 missed-reply count per
servo index 0-19, then the longest control-thread period since the previous
`GET_BUS_STATS` (u32 us) and the total control deadline misses (u32). The branch counts, in order: UART overrun, framing and noise
(overruns reported by the patched driver, plus flag samples), RX ring
overflows, DMA buffer starvations; Sync Read requests sent, requests answered
by every member, timeouts with no byte received, timeouts with some bytes,
bytes received, stale bytes drained before a request, bytes skipped while
searching for a header, LEN fields outside the telemetry reply's range,
packets failing CRC or structure checks, valid packets from an ID not on the
branch, packets with a nonzero servo error field, request transmit failures,
receiver restart failures; `UART_RX_RDY` ranges that skipped bytes, ranges
that repeated bytes, eDMA errors on the branch's receive and transmit channels
(`firmware/patches/`); bad packets followed by a header exactly where they end
(bytes changed in place), bad packets followed by anything else (bytes lost or
added), samples that found the whole eDMA halted, and the eDMA ES register at
the last error on the branch's receive channel. Capability bit 12 advertises it. `headctl bus-stats --seconds N` reads
it twice, N seconds apart, and prints the differences.

`GET_CONTROL_TIMING` is type 48 and carries only `transaction_id`. The reply
`CONTROL_TIMING` (type 49, 56 bytes) is `transaction_id` (u32), the control
cycle count since boot (u32, wraps), the start time of the last control cycle
(u64 us from the MCU cycle counter, low word first), schedule restarts since
boot (u32: cycles that ended after the next deadline, so that slot was lost;
homing reconfiguration is not counted), five period bins since boot (u32 each:
time between cycle starts below 1900 us, 1900-2099, 2100-2499, 2500-3999, and
4000 us or more), cycles whose locked section took half the period or more
(u32), then the longest period, the longest locked section ("busy time") and
the longest wait for `head_lock` since the previous `GET_CONTROL_TIMING` (u32
us each). Capability bit 16 advertises it. The total control deadline misses
in `BUS_STATS` and `DIAGNOSTICS` count every period over 2000 us, so wake
jitter reads as a miss there; use this message to measure the rate.

`headctl control-timing --seconds N` reads it twice, N seconds apart, and
prints the mean rate (cycles over MCU start-time difference), the period bins,
lost cycles, busy time and lock wait, and the controller states seen in the
window. It applies the validation rule and exits 1 on FAIL: mean rate within
0.1 % of the advertised control rate, no schedule restart and no period of
4000 us or more, and a longest busy time under half the period. Run it in one
steady state; it warns when the state changed during the window.

`HELLO` carries only a transaction ID. `HELLO_REPLY` reports the HELLO schema,
wire version, semantic firmware/build identity, hardware revision/identity,
protocol schema hash, capability bits, boot/session ID, reset cause,
calibration generation, selected control/telemetry rates, and USB VID/PID.

## Dynamixel contract

- Protocol 2.0, 1 Mbps, 8-N-1, TTL half duplex.
- Four independent physical UARTs, up to five configured IDs per branch. Each
  profile slot's `branch` byte assigns that servo to a branch; any ID 0–19
  may use any branch. Per-branch diagnostics `expected`/`received` masks
  number that branch's active servos in ascending ID order (bit 0 is the
  lowest active ID on the branch), not by ID.
- Broadcast Sync Write for time-aligned goals within a branch.
- Scheduled Sync/Fast Read for measured state.
- CRC validation and required Protocol 2.0 byte stuffing in both directions.
- Model 1210, firmware v38 or later, 1 Mbps, safe drive-mode bits, Protocol 2.0,
  Torque Enable 0, Status Return Level 2, required Shutdown bits, zero profile
  values, and Current Limit are checked at discovery. Because they are
  confirmed project-wide choices, Return Delay Time is set to zero and
  Operating Mode is set to mode 5 only when readback differs and while torque
  is off; both are then verified. Firmware v46 and later also has Startup
  Torque On conditionally cleared and verified.
- Required commissioned settings include ID, baud, current-based position
  operating mode, current limit, velocity/profile limits, return delay, status
  return level, Bus Watchdog, shutdown mask, and position limits.

Calibration profile v4 requires per-servo `operating_current_ma`, homing current
limit, homing speed, and contact backoff; firmware
writes Goal Current before torque enable and verifies it does not exceed the
servo's persistent Current Limit. It then arms the RAM Bus Watchdog for 200 ms
and reads the watchdog and Torque Enable registers back from every active
servo. Broadcast transport success alone is never considered a safety ACK.
Exact current, homing, and motion limits remain mechanical calibration data.
Calibration v4 names travel-rate fields
`max_position_fraction_per_control_cycle` and
`max_acceleration_fraction_per_control_cycle`. The former
`homing_start_tick` slot is retained as `reserved_homing_start_tick` for binary
compatibility and must be zero.

## Homing contract

Homing is a bounded tension search, not a hard-stop calibration:

1. Start from a measured present position with a low commissioned current
   limit; never command every actuator to a stored home tick at torque-on.
2. Enable torque only for the active homing batch.
3. Advance slowly in the configured direction.
4. Accept tension only after current remains over threshold for a configured
   persistence interval. Following error may be used as supporting evidence.
5. Abort on maximum travel, time, temperature, communication error, or current
   safety limit.
6. Hold the measured tension reference at the qualified position, or disable
   it as required by the validated mechanical sequence.

The calibration profile will carry a homing conflict/batch identifier. The two
confirmed co-actuated groups are the right modiolus `{8, 11, 14}` and the
left modiolus `{9, 16, 19}` (down, diag, up). Members of
each group must be placed in different batches and home separately. Servos with
distinct action points may eventually share a batch, after individual homing
and branch-current behavior have been validated.

## Remaining limitations

- `POSITION_VELOCITY` and `VELOCITY` are explicitly unsupported.
- The 500 Hz scheduler, 100 Hz complete feedback, UART turnaround, and fan
  behavior have not been measured on hardware. `headctl control-timing`
  (capability bit 16) is the measurement for the scheduler.
- Homing is deliberately sequential and lacks a profile-level batch map.
- Full servo configuration export/readback, packet-vector, lifecycle, timing,
  and hardware-in-loop test coverage remain incomplete.
- USB VID/PID values are still Zephyr test identifiers.
- A ROS 2 Jazzy colcon build still requires a host with Jazzy installed; this
  workspace currently provides syntax checks only for the bridge.
