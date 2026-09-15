cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(head_firmware)

target_include_directories(app PRIVATE ../include)
target_sources(app PRIVATE
  ../src/main.c
  ../src/board.c
  ../src/config.c
  ../src/control.c
  ../src/dxl.c
  ../src/fan.c
  ../src/lease.c
  ../src/protocol.c
  ../src/state_machine.c
)

if(HEAD_BENCH_NO_12V)
  target_compile_definitions(app PRIVATE HEAD_BENCH_NO_12V=1)
endif()

# Bench-only: allow torque with no cooling fan wired to the tach input.
# Never enable this for an assembled head.
if(HEAD_BENCH_NO_FAN)
  target_compile_definitions(app PRIVATE HEAD_BENCH_NO_FAN=1)
  message(WARNING "HEAD_BENCH_NO_FAN: fan-stall shutdown disabled; bench use only")
endif()

# Bench-only: admit line-test, uart-tx-test and uart-rx-test while the board is
# in a discovery fault. HEAD_TORQUE_OFF_VERIFIED needs a successful discovery,
# so a silent branch otherwise locks the diagnostics that would explain it. All
# three are torque-off by construction. Production builds never define it.
if(HEAD_BENCH_LINE_TEST_UNGATED)
  target_compile_definitions(app PRIVATE HEAD_BENCH_LINE_TEST_UNGATED=1)
  message(WARNING "HEAD_BENCH_LINE_TEST_UNGATED: line diagnostics admitted in a faulted state; bench use only")
endif()

# Bench-only recovery profile for isolating the physical J3 UART path. The
# stored NVS profile is left untouched and production builds never define it.
if(HEAD_BENCH_J3_ID10)
  target_compile_definitions(app PRIVATE HEAD_BENCH_J3_ID10=1)
  message(WARNING "HEAD_BENCH_J3_ID10: volatile one-servo J3/ID10 profile; bench use only")
endif()

# Whole-branch variant of the above: all five J3 servos (indices 10-14, IDs
# 10-14). Implies HEAD_BENCH_J3_ID10, which carries the shared profile body.
if(HEAD_BENCH_J3_ALL)
  target_compile_definitions(app PRIVATE HEAD_BENCH_J3_ID10=1 HEAD_BENCH_J3_ALL=1)
  # Optional: narrow the active window inside indices 10-14. COUNT bisects the
  # servo count; START picks where the window begins, so COUNT=1 START=12 runs
  # servo 12 alone.
  if(HEAD_BENCH_J3_COUNT)
    target_compile_definitions(app PRIVATE HEAD_BENCH_J3_COUNT=${HEAD_BENCH_J3_COUNT})
  endif()
  if(HEAD_BENCH_J3_START)
    target_compile_definitions(app PRIVATE HEAD_BENCH_J3_START=${HEAD_BENCH_J3_START})
  endif()
  message(WARNING "HEAD_BENCH_J3_ALL: volatile J3 profile; bench use only")
endif()
