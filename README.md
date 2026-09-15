# Tendon robot-head controller

This repository contains the Teensy 4.1 firmware and ROS 2 bridge for a
tendon-driven expressive robot head. Future learning code and data belong in
`../proprioception/`. A model may provide actuator-space position (`q`) and,
optionally, velocity (`qdot`) requests; the Teensy remains responsible for
timing, limits, homing, bus communication, cooling, watchdogs, and torque-off
behavior.

The project is currently a laboratory prototype. The authoritative PCB is
`../hardware/electronics/controller-pcb/PCB.pdf.kicad_sch` plus
`../hardware/electronics/controller-pcb/PCB.pdf.kicad_pcb`. That revision has
passed KiCad ERC/DRC and is in fabrication as of 2026-08-20. Hardware is the
source of truth; firmware and documentation must conform to it.

See:

- [API.md](docs/API.md) for the current and planned host contracts.
- [HARDWARE_FIRMWARE_REVIEW.md](../hardware/docs/HARDWARE_FIRMWARE_REVIEW.md)
  for the cross-check findings and unresolved commissioning data.
- [IMPLEMENTATION_PLAN.md](docs/IMPLEMENTATION_PLAN.md) for the ordered work and
  acceptance gates.
- [proprioception/README.md](../proprioception/README.md) for the deferred
  current-based elastic-surface touch experiment and cheek-poke demonstration.
- [NAMING_CONVENTIONS.md](docs/NAMING_CONVENTIONS.md) for mandatory project-owned
  identifier, unit-suffix, and compatibility rules.
- [CODE_AUDIT_2026-08-26.md](../docs/reviews/CODE_AUDIT_2026-08-26.md) for
  the current API, firmware-logic, RT/nonblocking-communication, naming, and
  verification audit.

## Confirmed project scope

- Up to 20 ROBOTIS Dynamixel XC330-T181-T servos on four independent 1 Mbps
  TTL half-duplex buses, with five addresses reserved per branch.
- All 20 servos are now on hand. The original twelve use IDs 0 through 11;
  the eight newly received units are reserved for IDs 12 through 19 but remain
  uncommissioned until each is individually assigned, changed to 1 Mbps, and
  verified. The confirmed final assignment is sequential: IDs 0–4 on J1,
  5–9 on J2, 10–14 on J3, and 15–19 on J4.
- Current-based position control is the intended Dynamixel operating mode.
- Position-only commands are the first priority. Position plus velocity
  feed-forward should be supported next; velocity-only operation is useful but
  lower priority.
- The MCU control-loop target is 500 Hz. A measured 250 Hz or 100 Hz rate is an
  acceptable fallback if the hardware and telemetry budget cannot reliably
  sustain 500 Hz. The chosen production rate must be measured, not inferred.
- Complete measured actuator feedback is required at at least 100 Hz.
- A confirmed serious fault disables torque on the entire head.
- Homing pulls a tendon slowly until sustained current indicates mechanical
  resistance. The co-actuated groups are `{9, 12, 14}` and `{8, 17, 19}`;
  members of a group must home separately. Other servos may eventually home
  concurrently, but first commissioning remains one servo at a time.
- The model initially runs on a desktop and may later move to the local Jetson.

## Do not tension the assembled head yet

The September 8 firmware audit has software repairs and regression evidence
recorded in [the repair report](../docs/reviews/FIRMWARE_REPAIR_PLAN_2026-09-09.md).
Independent torque removal remains a hardware requirement. These repairs do
not validate electrical timing, current, homing, or cooling on the fabricated board. Keep tendons unloaded until the
commissioning gates in `docs/IMPLEMENTATION_PLAN.md` pass. A calibration v4 profile
with an explicit `operating_current_ma` for every active servo is required;
old v2 profiles are deliberately rejected.

There is also a power-system constraint: a normal 3-cell LiPo is 12.6 V when fully
charged, while the XC330-T181-T operating range ends at 12.0 V. The PCB routes
the battery input directly to the fused servo branches. Firmware cannot make
12.6 V safe for a 12.0 V-rated actuator. Initial bring-up will therefore use an
external regulated supply rather than the LiPo. The available supply is
nominally 12 V, approximately 9 A maximum, with an adjustable current limit.
Verify its polarity and actual output with a multimeter before connection. Since
12.0 V is the servo's upper operating limit, account for supply tolerance and
turn-on overshoot; set the rail slightly below 12.0 V unless measurements prove
it never exceeds 12.0 V.

The board has no firmware-readable emergency stop. The present physical stop
is removal of the 12 V actuator supply. Fuses protect branch wiring; they are
not an emergency stop.

## Fabricated hardware map

All four servo connectors use the same pin order:

| Connector pin | Net |
|---:|---|
| 1 | GND |
| 2 | Fused battery/servo supply |
| 3 | Dynamixel DATA |

The branch assignments are:

| Branch | Connector | Teensy RX / TX | Teensy direction pin | Intended IDs |
|---:|---|---:|---:|---:|
| 0 | J1 | 0 / 1 | 4 | 0–4 |
| 1 | J2 | 25 / 24 | 12 | 5–9 |
| 2 | J3 | 16 / 17 | 18 | 10–14 |
| 3 | J4 | 34 / 35 | 36 | 15–19 |

The installed logic topology is not a direct Teensy-to-5 V bus connection.
Each branch uses a 3.3 V 74LVC2G241 for receive/direction gating and a 5 V
SN74AHCT1G126 as the final transmit driver. The shared direction signal is low
for receive/released and high for transmit. Each direction signal has a 10 kΩ
pulldown so the driver is released during reset.

Firmware now uses GPIO4.6, GPIO2.1, GPIO1.17, and GPIO2.18. Zephyr's i.MX RT
GPIO driver selects the GPIO mux entry from the SoC pinmux table when each pin
is configured. The assignments still require no-power multimeter verification
on the fabricated board.

## Power and cooling

- Input: 3-cell LiPo through J5/XT60.
- Servo branches: direct battery input through four 3 A mini-blade fuses.
- Jetson: Pololu D42V55F9, nominal 9 V output through J7 to the barrel jack.
- Buffer supply: Pololu D45V5F5, nominal 5 V output.
- Controller: Teensy 4.1 powered by USB.
- Fan: 12 V Noctua NF-A4x10 PWM on J8.

The D42V55F9 is a step-down regulator. Its input must remain above 9 V by its
load-dependent dropout voltage to regulate the Jetson rail. Nine volts is also
the minimum specified DC-jack voltage for the Jetson Orin Nano developer-kit
carrier. Establish a battery cutoff from measured worst-case Jetson load,
wiring drop, and transient behavior; do not use the LiPo's absolute empty
voltage as the operating cutoff.

The fan PWM path is an inverting 2N3904 open-collector stage driven by Teensy
pin 33; tachometer feedback reaches pin 32. Firmware now drives 25 kHz PWM,
counts the two tach pulses per revolution, and latches a whole-head fan fault
after a two-second no-pulse interval while torque is enabled. Validate the J8
waveform and reported RPM before treating cooling diagnostics as accepted.

## What “USB CDC” means

USB CDC is simply a virtual serial connection carried over the Teensy's USB
cable. On Linux it normally appears as `/dev/ttyACM0` and `/dev/ttyACM1`:

- CDC0 is the Zephyr diagnostic console.
- CDC1 is the framed head-control and telemetry endpoint.

The desktop ROS bridge opens CDC1 and translates ROS messages to the compact
firmware protocol. The ML model does not need to know the USB framing; it can
publish through the ROS-facing interface. A future Jetson-hosted model can use
the same arrangement.

## Current implementation status

| Capability | Current status |
|---|---|
| Zephyr 3.7.2 LTS production build | Builds successfully |
| Four UART selection | Implemented |
| Direction GPIO map/turnaround | Corrected to PCB; OE release waits the NXP LPUART transmission-complete flag; hardware test pending |
| Dynamixel CRC | Implemented and cross-checked |
| Dynamixel transmit byte stuffing | Implemented; official-vector coverage still needed |
| Position command | Protocol v2 normalized `POSITION` implemented |
| Position + velocity / velocity-only | Represented in v2 but explicitly rejected pending control-law validation |
| Servo safety commissioning | Requires firmware v38+ (v46+ for Startup Configuration), mode 5, Protocol 2.0, 1 Mbps, Secondary ID disabled, conservative thermal/voltage/PWM/current/shutdown limits, normalized gains, safe startup torque, and watchdog readback |
| Servo current | Calibration v4 separates operating and homing limits; each physical branch's active operating limits must fit a 2,500 mA budget, checked first before torque-on and supervised independently from fresh live current telemetry |
| 500 Hz fixed-period scheduling | Absolute scheduler and parallel DMA TX implemented; target measurement pending |
| 100 Hz telemetry | Full five-servo branch response is staged/drained; single-owner schedule implemented; hardware acceptance pending |
| Present velocity / feedback age | Implemented in v2 state |
| Whole-head torque-off | Immediate boot broadcast plus idempotent two-sweep shutdown with per-servo Torque Enable readback |
| Current-threshold homing | Configured low-speed/current search, one energized servo at a time, contact backoff, and torque-off READY implemented; HIL validation pending |
| Static proprioception hold | Explicit post-homing settling/hold states seed fresh measured targets, reject motion commands, refresh the actuator watchdog, and retain 100 Hz state feedback; HIL validation pending |
| Tendon-routing hold | Explicit pre-homing-capable state ramps every active servo to raw tick 0 (0°), rejects motion commands, and holds under lease/fault shutdown protection; HIL validation pending |
| Fan PWM and tachometer | Implemented; waveform/RPM/stall tests pending |
| ROS 2 bridge | Jazzy actions use ROS timers/`rclpy` futures; controller profile supplies the active mask; non-finite targets rejected |
| Proprioception desktop collection | Headless ROS session controller, MCAP/JSONL capture, atomic finalization, and thin Qt operator GUI implemented; replay/HIL validation pending |
| Automated/HIL tests | Protocol, lifecycle, DXL, shutdown/preparation and host transport regressions pass; live ROS and HIL validation remain |

After homing reaches torque-off `READY`, a valid controller may enter the
static collection mode through `/head/start_proprioception` or
`headctl proprioception`. The requesting ROS collector/client or `headctl` owns lease renewal because
this mode intentionally sends no joint-target stream. The bridge never renews
a lease independently of its owner; action clients must also renew during homing. Use `/head/disable` or
Ctrl-C in `headctl` to request the standard verified torque shutdown.

For tendon installation, the operator GUI may enter `ROUTING` directly from
torque-off `HOMING_REQUIRED` or `READY`. The firmware ramps all active servos
to raw tick 0 and holds them there; use the GUI Disable control before removing
the control connection. Loss of the GUI heartbeat expires the lease and
triggers verified torque shutdown.

For labeled dataset collection, use the desktop session controller described in
[`host_ros/README.md`](host_ros/README.md). Recording start/end are host
acquisition states layered over `PROPRIOCEPTION_HOLD`; they are intentionally
not additional actuator states in the Teensy firmware.

## Build

Use the Zephyr 3.7.2 LTS workspace and the production storage configuration:

```bash
cd ~/project_HEAD/firmware
.venv/bin/west build -p always -b teensy41 -d builds/production zephyr -- \
  -DCONF_FILE="prj.conf;prj_production_storage.conf" \
  -DDTC_OVERLAY_FILE=teensy41.overlay
```

The trailing `zephyr` is the application source directory and is required:
`west build` is run from `firmware/`, but the application lives in
`firmware/zephyr/`. Omitting it fails with *source directory "." does not
contain a CMakeLists.txt*. Because `-p always` wipes the build directory before
configuring, that failure also destroys the previous image — always pass the
board, source directory, and CMake flags in full rather than relying on a
cached configuration.


For USB-only work with actuator power physically disconnected, use the no-12 V
bench image:

```bash
.venv/bin/west build -p always -b teensy41 -d builds/bench-no-12v zephyr -- \
  -DHEAD_BENCH_NO_12V=ON \
  -DCONF_FILE="prj.conf;prj_bench_no_12v.conf" \
  -DDTC_OVERLAY_FILE=teensy41_bench_no_12v.overlay
```

The production build currently warns that the USB VID/PID values are Zephyr
test identifiers. A project-owned VID/PID must be assigned before distributing
firmware; the deprecated CDC devicetree `label` properties have been removed.
Generated builds from the former directory are retained in
`builds/legacy-pre-reorg/` for reference only; their CMake caches contain old
absolute paths. `builds/production/` was regenerated after the reorganization.

## Commissioning policy

Commissioning proceeds through the gates in `docs/IMPLEMENTATION_PLAN.md`. In
summary:

1. Verify the nominal 12 V/9 A external supply is at or below 12.0 V. Begin with
   a low current limit and do not substitute the direct 3S LiPo.
2. Correct and test GPIO direction control with no servo power.
3. Verify all branch connectors with a multimeter and the firmware's safe
   line tests.
4. Use a current-limited supply for the first powered single-servo test. This
   is one of the few stages where it should not be skipped.
5. Read and record every servo's model, firmware, ID, baud, operating mode,
   current limit, return delay, watchdog, startup configuration, Status Return
   Level, profiles, and shutdown settings before motion. Firmware below v38 is
   rejected because it lacks the required Bus Watchdog; firmware v46 and later
   additionally has Startup Torque On cleared and read back.
6. Characterize one mechanically unloaded or isolated tendon at low current.
7. Add one branch at a time, then perform low-force assembled-head homing.

A multimeter is sufficient for static rail, polarity, and direction-enable
checks. It cannot validate 1 Mbps signal integrity, UART turnaround, 500 Hz
jitter, or a 25 kHz PWM waveform. Borrowing an oscilloscope or suitable logic
analyzer for one final validation session is therefore an acceptance-plan
item, not an assumption that such equipment is always available.
