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
