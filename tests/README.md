# Host unit tests

The small protocol test is intentionally independent of Zephyr so malformed
host frames and the advertised 512-byte maximum payload can be checked quickly
before a target build:

```sh
cc -std=c99 -Wall -Wextra -Werror -Iinclude \
  tests/protocol_test.c src/protocol.c -o /tmp/head_protocol_test
/tmp/head_protocol_test
```

The logic suite also covers calibration validation, the fixed-width storage
schema, lease rollover, command sequencing, the static-proprioception
lifecycle, the zero-degree routing lifecycle, enable at present pose and
exact `q` seeding at Enable, randomized trajectory limits,
terminal homing for single/sparse/full inventories, latched-fault recovery and
shutdown idempotence, storage/preparation exclusion, and shifted position
range boundaries:

```sh
cc -std=c99 -Wall -Wextra -Wconversion -Wshadow -Werror \
  -Iinclude tests/logic_test.c src/config.c src/control.c src/lease.c \
  src/protocol.c src/state_machine.c -o /tmp/head_logic_test
/tmp/head_logic_test
```

The DXL regression compiles the production `src/dxl.c` with a deterministic
synthetic board transport. It checks the Hardware Alert policy (including the
latched alert mask), incremental preparation ordering, fresh position seeding
before torque enable, and homing preparation of only the selected servo:

```sh
python3 tests/run_dxl_regression.py
```

The main-loop regression extracts production preparation/shutdown functions and
links the real state/control/configuration logic. It checks preparation time and
lease bounds, shutdown retry backoff, repeated fault requests, and two-sweep
verified torque-off:

```sh
python3 tests/run_main_regression.py
```

Host transport tests cover bounded writes, ACK matching, queue priority,
coalescing, reset, and fragmented OS PTY traffic. The integration suite also
executes production bridge/collector lifecycle methods, the bridge's
position-range cache, maintenance completion
and cancellation, and the bridge DIAGNOSTICS decoder against the firmware
payload layout. It uses ROS stubs and a small serial adapter; it does not test
a live ROS executor or the installed pySerial driver:

```sh
python3 tests/host_transport_test.py
python3 tests/host_integration_test.py
```

The ROS-free commissioning regression checks ACK deadlines/session identity,
profile compatibility, completion/readback of persistence, the DIAGNOSTICS
decoder, and `--port auto` port selection:

```sh
python3 tests/headctl_test.py
```

The collection state and atomic-session writer are also ROS-independent:

```sh
python3 tests/collection_core_test.py
```

The jog panel's seeding, streaming, lease and pose rules are ROS-independent
too. They do not cover the Qt window or a live ROS executor:

```sh
python3 tests/jog_core_test.py
```

The tuning panel's link, lease handling, record writing, PWM pairing,
telemetry log and jaw sweep (on the simulator's tendon model, with lines the
horns cannot feel the jaw through: release tendons pay out so the jaw opens,
come back to their start when it closes, a jaw with no room is not forced;
abort, wrong pull direction, travel limit and stage timeout) are tested against the in-process simulated
controller, together with the profile proposal tool:

```sh
python3 tests/tuning_core_test.py
```

The face workspace guard (action mapping, the no-fighting rule, speed limit,
follower steps, live stop checks) is pure Python:

```sh
python3 tests/workspace_guard_test.py
```

The ENABLED-mode face session (enable at the present pose, target frames
and their q rounding, followers, jaw-dependent neutrals, the command
watchdog, the stop checks and the session neutral file) runs on the
simulated board:

```sh
python3 tests/face_stream_test.py
```

The knee rule of the modiolus knee test (baseline, noise, threshold, knee):

```sh
python3 tests/knee_finder_test.py
```

Finding the face in the camera picture (the knee test's corner boxes) needs
OpenCV, so it runs with the bench venv:

```sh
../bench/.venv/bin/python tests/knee_camera_test.py
```

The face marker tracker needs numpy, so it runs with the bench venv:

```sh
../bench/.venv/bin/python tests/face_markers_test.py
```

The lip-sync additions (2026-11-08). The mouth trajectory file, the session's
upper lips from the panel marks, trajectory streaming, the command log, the
audio start and the camera mode, and the babbling generator are pure Python:

```sh
python3 tests/mouth_actions_test.py
```

```sh
python3 tests/face_stream_lipsync_test.py
```

```sh
python3 tests/babble_test.py
```

The mouth measures (drawn faces; the MediaPipe lip landmark numbers against
the installed table) and the viseme tool (targets from photos, the pose
sequencer, the search on the simulated board with a simulated mouth) need
numpy, Pillow and MediaPipe, so they run with the bench venv:

```sh
../bench/.venv/bin/python tests/mouth_shape_test.py
```

```sh
../bench/.venv/bin/python tests/viseme_tuner_test.py
```

The lip-sync training side has its own tests in `../../lipsync/tests`
(`../../lipsync/README.md`).

The model-data window regression lives in `../../proprioception/tests` and
requires the packages in `../../proprioception/requirements.txt`:

```sh
python3 ../../proprioception/tests/dataset_test.py
```

Run these commands from `firmware/`. Add `-fsanitize=undefined` to the C
commands to reproduce the repair validation.

Target-driver changes still require the production Teensy build and the
current-limited hardware commissioning procedure in
`../docs/IMPLEMENTATION_PLAN.md`.
