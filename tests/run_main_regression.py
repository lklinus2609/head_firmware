"""Exercise actual main.c supervisor helpers with deterministic hardware doubles.

No Zephyr scheduling is simulated: this checks orchestration, not target timing.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / 'firmware/src/main.c').read_text()

def function(signature):
    start = SOURCE.index(signature)
    return SOURCE[start:SOURCE.index('\n}', start) + 2]

PREAMBLE = r'''
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include "head.h"
#include "state_machine.h"
#include "lease.h"
static struct head_runtime runtime;
static struct head_calibration calibration;
static uint32_t fake_now_ms;
static uint32_t step_duration_ms;
static int step_result;
static unsigned step_count;
static unsigned shutdown_count;
static int shutdown_result;
static uint32_t alert_mask;
static uint32_t k_uptime_get_32(void) { return fake_now_ms; }
static void head_dxl_abort_telemetry(struct head_runtime *value) { (void)value; }
static int head_dxl_prepare_step(struct head_runtime *value, const struct head_calibration *profile)
{ (void)value; (void)profile; ++step_count; fake_now_ms += step_duration_ms; return step_result; }
static int head_board_fan_set_percent(uint8_t percent) { assert(percent == 100u); return 0; }
static int head_dxl_set_torque_all(const struct head_calibration *profile, bool enabled)
{ (void)profile; assert(!enabled); ++shutdown_count; return shutdown_result; }
static int head_board_release_all(void) { return 0; }
static uint32_t head_dxl_take_hardware_alert_mask(void)
{ const uint32_t value = alert_mask; alert_mask = 0u; return value; }
'''
TESTS = r'''
static void reset_fixture(void)
{
  runtime = (struct head_runtime){0};
  runtime.state = HEAD_READY;
  runtime.discovery_verified = true;
  runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
  runtime.active_lease_token = 1u;
  runtime.lease_expires_ms = 2000u;
  fake_now_ms = 100u; step_duration_ms = 1u; step_result = 0;
  step_count = 0u; shutdown_count = 0u; shutdown_result = 0; alert_mask = 0u;
}
int main(void)
{
  reset_fixture();
  begin_preparation_locked(HEAD_ENABLED);
  assert(runtime.preparation_active);
  assert(!head_state_can_prepare_motion(&runtime));
  assert(runtime.torque_state == HEAD_TORQUE_OFF_VERIFIED);
  service_preparation_locked();
  assert(step_count == 1u && runtime.preparation_active && runtime.state == HEAD_READY);
  step_result = 1;
  service_preparation_locked();
  assert(runtime.state == HEAD_ENABLED && runtime.torque_state == HEAD_TORQUE_ON_VERIFIED);
  assert(!runtime.preparation_active && runtime.last_command_ms == fake_now_ms);

  reset_fixture(); begin_preparation_locked(HEAD_ENABLED);
  fake_now_ms = runtime.lease_expires_ms;
  service_preparation_locked();
  assert(step_count == 0u && runtime.fault == HEAD_FAULT_WATCHDOG);
  assert(runtime.shutdown_requested && !runtime.preparation_active);

  reset_fixture(); begin_preparation_locked(HEAD_ENABLED);
  step_duration_ms = 21u;
  service_preparation_locked();
  assert(runtime.fault == HEAD_FAULT_CONTROL_DEADLINE && runtime.shutdown_requested);

  reset_fixture(); begin_preparation_locked(HEAD_ENABLED);
  step_result = -EIO;
  service_preparation_locked();
  assert(runtime.fault == HEAD_FAULT_BUS && runtime.shutdown_requested);

  reset_fixture();
  head_state_fault(&runtime, HEAD_FAULT_SERVO);
  shutdown_result = -EIO;
  assert(service_shutdown_locked(fake_now_ms) == -EIO);
  const uint32_t retry_ms = runtime.shutdown_next_attempt_ms;
  assert(shutdown_count == 1u && retry_ms > fake_now_ms);
  head_state_fault(&runtime, HEAD_FAULT_WATCHDOG);
  assert(runtime.fault == HEAD_FAULT_SERVO && runtime.shutdown_next_attempt_ms == retry_ms);
  assert(service_shutdown_locked(fake_now_ms + 1u) == -EAGAIN && shutdown_count == 1u);
  shutdown_result = 0;
  assert(service_shutdown_locked(retry_ms) == 0);
  assert(runtime.shutdown_requested && runtime.shutdown_confirmations == 1u);
  assert(service_shutdown_locked(retry_ms + 10u) == 0);
  assert(runtime.torque_state == HEAD_TORQUE_OFF_VERIFIED && !runtime.shutdown_requested);
  assert(runtime.state == HEAD_FAULT && runtime.fault == HEAD_FAULT_SERVO);

  reset_fixture(); head_state_disable(&runtime);
  alert_mask = 1u;
  assert(service_shutdown_locked(fake_now_ms) == 0);
  assert(runtime.fault == HEAD_FAULT_SERVO && runtime.shutdown_confirmations == 1u);
  assert(service_shutdown_locked(fake_now_ms + 10u) == 0);
  assert(runtime.torque_state == HEAD_TORQUE_OFF_VERIFIED && runtime.state == HEAD_FAULT);
  puts("main preparation/shutdown integration: PASS");
}
'''

def main():
    with tempfile.TemporaryDirectory(prefix='head-main-regression-') as directory:
        source = Path(directory) / 'main_regression.c'
        executable = Path(directory) / 'main_regression'
        source.write_text(PREAMBLE + function('static void begin_preparation_locked') +
                          function('static void service_preparation_locked') +
                          function('static int service_shutdown_locked') + TESTS)
        subprocess.run(['cc', '-std=c99', '-Wall', '-Wextra', '-Wconversion', '-Wshadow',
                        '-Werror', '-fsanitize=undefined', '-I' + str(ROOT / 'firmware/include'),
                        str(source), *[str(ROOT / 'firmware/src' / name) for name in
                        ('state_machine.c', 'control.c', 'config.c', 'protocol.c', 'lease.c')],
                        '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)

if __name__ == '__main__':
    main()
