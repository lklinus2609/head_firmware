# Build outputs

Each variant is defined by its `CONF_FILE` and devicetree overlay, not by its
name. The name describes the bench rig; the config decides what the image can
do. Full commands are in `../README.md`.

| Directory | CONF_FILE | Overlay | NVS / flash map | Watchdog | CMake `-D` flags |
|---|---|---|---|---|---|
| `production/` | `prj.conf;prj_production_storage.conf` | `teensy41.overlay` | yes | yes | (none) |
| `bench-no-fan/` | `prj.conf;prj_production_storage.conf` | `teensy41.overlay` | yes | yes | `HEAD_BENCH_NO_FAN=ON` |
| `bus-stats/` | `prj.conf;prj_production_storage.conf` | `teensy41.overlay` | yes | yes | `HEAD_BENCH_NO_FAN=ON` (2026-10-06 diagnostics image A3, no receive fixes) |
| `jog-present-pose/` | `prj.conf;prj_production_storage.conf` | `teensy41.overlay` | yes | yes | `HEAD_BENCH_NO_FAN=ON` (2026-10-08: enable at present pose, position range, needs `../patches/` applied) |
| `present-pwm/` | `prj.conf;prj_production_storage.conf` | `teensy41.overlay` | yes | yes | `HEAD_BENCH_NO_FAN=ON` (2026-10-13: `jog-present-pose` sources plus the `PRESENT_PWM` message, needs `../patches/` applied) |
| `rx-fixes/` | `prj.conf;prj_production_storage.conf` | `teensy41.overlay` | yes | yes | `HEAD_BENCH_NO_FAN=ON` (2026-10-06 image B: receive fixes, needs `../patches/` applied) |
| `per-servo-tuning/` | `prj.conf;prj_production_storage.conf` | `teensy41.overlay` | yes | yes | `HEAD_BENCH_NO_FAN=ON` (2026-10-20: `present-pwm` sources plus per-servo tuning caps, min of limit and `operating_current_ma`; patches applied) |
| `baseline-20261020/` | `prj.conf;prj_production_storage.conf` | `teensy41.overlay` | yes | yes | `HEAD_BENCH_NO_FAN=ON` (2026-10-20 rebuild of today's sources before the per-servo change; byte-identical to `present-pwm/`) |
| `prep-diag/` | `prj.conf;prj_production_storage.conf` | `teensy41.overlay` | yes | yes | `HEAD_BENCH_NO_FAN=ON` (2026-11-10: today's sources, patches applied, plus DIAGNOSTICS byte 173 = last failed Enable/homing preparation step, `(phase << 5) \| servo`; built, NOT flashed) |
| `bench-j3-no-fan/` | `prj.conf` only | `teensy41.overlay` | **no** | **no** | `HEAD_BENCH_NO_FAN=ON` |
| `bench-j3-id10-no-fan/` | `prj.conf` only | `teensy41.overlay` | **no** | **no** | `HEAD_BENCH_NO_FAN=ON` |
| `bench-no-12v/` | `prj.conf;prj_bench_no_12v.conf` | `teensy41_bench_no_12v.overlay` | **no** | **no** | `HEAD_BENCH_NO_12V=ON` |

- `production/` is the image to flash for anything involving a calibration
  profile. `head_config_load()` is compiled behind `CONFIG_NVS` and
  `CONFIG_FLASH_MAP`; without them `headctl upload-profile` fails at commit with
  *profile commit rejected (NVS may be unavailable)* and `FAULT CONFIGURATION`
  never clears.
- **CMake `-D` flags do not appear in `zephyr/.config`.** `HEAD_BENCH_NO_FAN`
  and `HEAD_BENCH_NO_12V` reach the app through
  `target_compile_definitions()` in `../zephyr/CMakeLists.txt`, so comparing
  `.config` files makes a bench image look identical to production when it is
  not. Check `CMakeCache.txt` for these flags, not `.config`.
- `bench-no-fan/` has the same Kconfig as `production/` but is compiled with
  `HEAD_BENCH_NO_FAN=1`, which suppresses the fan-stall shutdown. On a bench
  with no fan wired this is essential: `main.c` raises `HEAD_FAULT_FAN` as soon
  as torque may be on if the tach reports no RPM.
- `bench-j3-no-fan/` is the J3 bring-up image: fan suppressed, and no NVS, so
  no previously committed profile can load. That matters when an older stored
  profile names branches that are not populated — discovery fails on them, the
  torque-off sweep parks in `SHUTDOWN_FAILED`, and `shutdown_requested` stays
  set. Without persistence every boot starts clean and the profile is uploaded
  per session.
- `bench-j3-id10-no-fan/` and `bench-no-12v/` omit the production storage
  config. They are useful for electrical work such as `line-test`, where the
  absent watchdog avoids the MCU resets described in
  `../docs/LINE_TEST_WATCHDOG_FIX.md`. They cannot store a profile.
- `legacy-pre-reorg/` preserves generated build variants from the old absolute
  workspace path. They are reference artifacts, not reusable CMake caches.

Create or refresh named variants with `west build -p always` and the full board,
source-directory, and CMake arguments; do not run Ninja directly inside a legacy
cache. Note that `-p always` deletes the build directory before configuring, so
a malformed command destroys the existing image as well as failing.

`bisect-*/` (2026-10-06) are one-off images for the receive-path bisect in
`../../docs/reviews/FIRMWARE_AUDIT_2026-10-05.md` section 8; the driver
switches they used were removed afterwards, so they cannot be rebuilt as-is.
`rx-fresh-buffer/` is image C: image B's fixes without the eDMA handler
change, plus a fresh 1024-byte receive buffer before each Sync Read.
`jog-present-pose/` (2026-10-08) is image D's sources plus
`ENABLE_AT_PRESENT_POSE`, `POSITION_RANGE` and `q` seeding at Enable, built
with the same flags as `rx-fresh-all/` (`HEAD_BENCH_NO_FAN=ON`, patches
applied).
`rx-fresh-all/` is image D: image C, with every request that expects a reply
(not only telemetry) sent through `dxl_send_request()`, which restarts the
receiver first; `headctl bus-stats` also reports control-loop timing.
