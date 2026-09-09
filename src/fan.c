#include "board.h"
#include "fan.h"

static uint16_t last_rpm;
static bool stalled;
static uint32_t possible_stall_since_ms;

int head_fan_init(void)
{
  return head_board_fan_set_percent(100u);
}

int head_fan_tick(const struct head_runtime *runtime,
                  const struct head_calibration *calibration,
                  uint32_t now_ms)
{
  uint8_t hottest = 0u;
  uint8_t percent;
  bool stale = false;
  for (uint8_t servo_index = 0; servo_index < HEAD_SERVO_COUNT; ++servo_index) {
    if ((calibration->active_servo_mask & (1u << servo_index)) == 0u) continue;
    if (!runtime->servos[servo_index].online || now_ms - runtime->servos[servo_index].last_feedback_ms > 30u) {
      stale = true;
      continue;
    }
    if (runtime->servos[servo_index].temperature_c > hottest) hottest = runtime->servos[servo_index].temperature_c;
  }
  if (stale || hottest >= calibration->thermal_full_c) {
    percent = 100u;
  } else if (hottest <= calibration->thermal_start_c) {
    percent = 20u;
  } else {
    percent = 20u + (uint8_t)((80u * (hottest - calibration->thermal_start_c)) /
              (calibration->thermal_full_c - calibration->thermal_start_c));
  }
  /* Host control can request additional cooling during commissioning, but it
   * cannot lower the autonomous thermal-safe duty cycle. */
  if (runtime->fan_override && runtime->fan_override_percent > percent) {
    percent = runtime->fan_override_percent;
  }
  const int pwm_result = head_board_fan_set_percent(percent);
  last_rpm = head_board_fan_rpm();
  if (percent < 20u || last_rpm != 0u) {
    possible_stall_since_ms = 0u;
    stalled = false;
  } else if (possible_stall_since_ms == 0u) {
    possible_stall_since_ms = now_ms;
    stalled = false;
  } else {
    /* Allow startup and one complete 500 ms tach measurement window before
     * treating absent pulses as a cooling failure. */
    stalled = now_ms - possible_stall_since_ms >= 2000u;
  }
  return pwm_result;
}

uint16_t head_fan_rpm(void) { return last_rpm; }
bool head_fan_stalled(void) { return stalled; }
