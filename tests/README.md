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
lifecycle, the zero-degree routing lifecycle, randomized trajectory limits,
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
executes production bridge/collector lifecycle methods and maintenance completion
and cancellation. It uses ROS stubs and a small serial adapter; it does not test
a live ROS executor or the installed pySerial driver:

```sh
python3 tests/host_transport_test.py
python3 tests/host_integration_test.py
```

The ROS-free commissioning regression checks ACK deadlines/session identity,
profile compatibility, and completion/readback of persistence:

```sh
python3 tests/headctl_test.py
```

The collection state and atomic-session writer are also ROS-independent:

```sh
python3 tests/collection_core_test.py
```

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
