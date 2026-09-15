# Build outputs

Each variant is defined by its `CONF_FILE` and devicetree overlay, not by its
name. The name describes the bench rig; the config decides what the image can
do. Full commands are in `../README.md`.

| Directory | CONF_FILE | Overlay | NVS / flash map | Watchdog | CMake `-D` flags |
|---|---|---|---|---|---|
| `production/` | `prj.conf;prj_production_storage.conf` | `teensy41.overlay` | yes | yes | (none) |
| `bench-no-fan/` | `prj.conf;prj_production_storage.conf` | `teensy41.overlay` | yes | yes | `HEAD_BENCH_NO_FAN=ON` |
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
