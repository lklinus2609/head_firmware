#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>

#include "board.h"

#if defined(HEAD_BENCH_NO_12V)

int head_board_init(void) { return -ENOTSUP; }
int head_board_branch_set_baud(uint8_t branch_index, uint32_t baudrate)
{ ARG_UNUSED(branch_index); ARG_UNUSED(baudrate); return -ENOTSUP; }
int head_board_branch_set_tx(uint8_t branch_index, bool tx)
{ ARG_UNUSED(branch_index); ARG_UNUSED(tx); return -ENOTSUP; }
int head_board_branch_rx_restart(uint8_t branch_index)
{ ARG_UNUSED(branch_index); return -ENOTSUP; }
int head_board_release_all(void) { return 0; }
uint32_t head_board_branch_error_flags(uint8_t branch_index)
{ ARG_UNUSED(branch_index); return 0u; }
int head_board_branch_uart_tx_meter_test(uint8_t branch_index, uint32_t duration_ms)
{ ARG_UNUSED(branch_index); ARG_UNUSED(duration_ms); return -ENOTSUP; }
int head_board_branch_uart_rx_line_test(uint8_t branch_index, uint32_t duration_ms,
                                        uint32_t *bytes, uint32_t *errors)
{
  ARG_UNUSED(branch_index); ARG_UNUSED(duration_ms); ARG_UNUSED(bytes); ARG_UNUSED(errors);
  return -ENOTSUP;
}
int head_board_branch_write_async(uint8_t branch_index, const uint8_t *data, size_t length)
{ ARG_UNUSED(branch_index); ARG_UNUSED(data); ARG_UNUSED(length); return -ENOTSUP; }
int head_board_branch_write_wait(uint8_t branch_index, uint32_t timeout_us)
{ ARG_UNUSED(branch_index); ARG_UNUSED(timeout_us); return -ENOTSUP; }
int head_board_branch_write(uint8_t branch_index, const uint8_t *data, size_t length)
{ ARG_UNUSED(branch_index); ARG_UNUSED(data); ARG_UNUSED(length); return -ENOTSUP; }
int head_board_branch_read(uint8_t branch_index, uint8_t *data, size_t length,
                           uint32_t timeout_us)
{
  ARG_UNUSED(branch_index); ARG_UNUSED(data); ARG_UNUSED(length); ARG_UNUSED(timeout_us);
  return -ENOTSUP;
}
size_t head_board_branch_read_available(uint8_t branch_index, uint8_t *data, size_t capacity)
{ ARG_UNUSED(branch_index); ARG_UNUSED(data); ARG_UNUSED(capacity); return 0u; }
int head_board_branch_wait_rx(uint8_t branch_index, uint32_t timeout_us)
{ ARG_UNUSED(branch_index); ARG_UNUSED(timeout_us); return -ENOTSUP; }
void head_board_branch_rx_stats(uint8_t branch_index, uint16_t *disabled,
                                uint16_t *restart_failures, uint16_t *overflows)
{
  ARG_UNUSED(branch_index);
  if (disabled != NULL) *disabled = 0u;
  if (restart_failures != NULL) *restart_failures = 0u;
  if (overflows != NULL) *overflows = 0u;
}
int head_board_fan_set_percent(uint8_t percent)
{ ARG_UNUSED(percent); return -ENOTSUP; }
uint16_t head_board_fan_rpm(void) { return 0u; }

#else

#include <fsl_lpuart.h>

/* Fixed devicetree references: Zephyr 3.7 does not create the legacy
 * string labels used by device_get_binding().  The enable pins are Teensy
 * digital pins 4, 12, 18, 36: GPIO4.6, GPIO2.1, GPIO1.17, GPIO2.18. */
static const struct device *const uart_devices[4] = {
  DEVICE_DT_GET(DT_NODELABEL(lpuart6)),
  DEVICE_DT_GET(DT_NODELABEL(lpuart1)),
  DEVICE_DT_GET(DT_NODELABEL(lpuart3)),
  DEVICE_DT_GET(DT_NODELABEL(lpuart5)),
};
/* Zephyr's MCUX asynchronous UART driver reports UART_TX_DONE when DMA has
 * supplied the final byte to LPUART.  The half-duplex buffer must remain in
 * transmit until the peripheral says that its FIFO and shift register are
 * both empty.  Use the same NXP status flag as the MCUX transactional API. */
static LPUART_Type *const uart_bases[4] = {
  (LPUART_Type *)DT_REG_ADDR(DT_NODELABEL(lpuart6)),
  (LPUART_Type *)DT_REG_ADDR(DT_NODELABEL(lpuart1)),
  (LPUART_Type *)DT_REG_ADDR(DT_NODELABEL(lpuart3)),
  (LPUART_Type *)DT_REG_ADDR(DT_NODELABEL(lpuart5)),
};
static const struct device *const oe_devices[4] = {
  DEVICE_DT_GET(DT_NODELABEL(gpio4)),
  DEVICE_DT_GET(DT_NODELABEL(gpio2)),
  DEVICE_DT_GET(DT_NODELABEL(gpio1)),
  DEVICE_DT_GET(DT_NODELABEL(gpio2)),
};
static const gpio_pin_t oe_pins[4] = { 6u, 1u, 17u, 18u };
static const struct device *branch_uart[4];
static const struct device *branch_oe[4];
static uint32_t branch_baudrate[4] = {
  HEAD_BRANCH_DEFAULT_BAUD, HEAD_BRANCH_DEFAULT_BAUD,
  HEAD_BRANCH_DEFAULT_BAUD, HEAD_BRANCH_DEFAULT_BAUD,
};

#define HEAD_USER_NODE DT_PATH(zephyr_user)
static const struct pwm_dt_spec fan_pwm = PWM_DT_SPEC_GET(HEAD_USER_NODE);
static const struct gpio_dt_spec fan_tach =
    GPIO_DT_SPEC_GET(HEAD_USER_NODE, fan_tach_gpios);
static struct gpio_callback fan_tach_callback;
static atomic_t fan_tach_edges;
static uint32_t fan_rpm_window_ms;
static uint16_t fan_rpm;

static void fan_tach_edge(const struct device *device, struct gpio_callback *callback,
                          gpio_port_pins_t pins)
{
  ARG_UNUSED(device);
  ARG_UNUSED(callback);
  ARG_UNUSED(pins);
  atomic_inc(&fan_tach_edges);
}

BUILD_ASSERT(IS_ENABLED(CONFIG_NOCACHE_MEMORY), "UART DMA requires MPU noncacheable RAM");
#define HEAD_BRANCH_TX_CAPACITY 96u
#define HEAD_BRANCH_RX_DMA_CAPACITY 512u
#define HEAD_BRANCH_RX_RING_CAPACITY 1024u
struct branch_tx_state {
  struct k_sem done;
  struct k_sem rx_disabled;
  struct k_sem rx_ready;
  struct k_spinlock rx_lock;
  struct ring_buf rx_ring;

  uint8_t rx_storage[HEAD_BRANCH_RX_RING_CAPACITY];
  size_t length;
  atomic_t result;
  atomic_t rx_buffers_used;
  atomic_t rx_error_flags;
  atomic_t active;
  atomic_t transmission_complete_pending;
  atomic_t abort_requested;
  uint8_t branch_index;
  atomic_t rx_enabled;
  atomic_t rx_disabled_count;
  atomic_t rx_starvation_count;
  atomic_t rx_restart_failures;
  atomic_t rx_overflow_count;
};
/* DMA payloads alone live in the MPU noncacheable region. Kernel objects and
 * CPU-owned rings remain in ordinary RAM. Each row starts on a cache line. */
static uint8_t branch_tx_payload[4][HEAD_BRANCH_TX_CAPACITY] __nocache __aligned(32);
static uint8_t branch_rx_payload[4][2][HEAD_BRANCH_RX_DMA_CAPACITY] __nocache __aligned(32);
static struct branch_tx_state branch_tx[4];

static int branch_wait_transmission_complete(uint8_t branch_index)
{
  /* DMA completion can leave up to one hardware FIFO of bytes pending.  Size
   * this bound from the selected diagnostic/production baud so factory-default
   * 57,600-baud probes are as safe as the 1-Mbps runtime path. */
  const uint32_t fifo_bytes = FSL_FEATURE_LPUART_FIFO_SIZEn(uart_bases[branch_index]);
  const uint32_t drain_us = (fifo_bytes * 10000000u +
                             branch_baudrate[branch_index] - 1u) /
                            branch_baudrate[branch_index] + 50u;
  const uint32_t deadline = k_cycle_get_32() + k_us_to_cyc_ceil32(drain_us);
  while ((LPUART_GetStatusFlags(uart_bases[branch_index]) &
          (uint32_t)kLPUART_TransmissionCompleteFlag) == 0u) {
    if ((int32_t)(k_cycle_get_32() - deadline) >= 0) return -ETIMEDOUT;
  }
  return 0;
}

/* Callable from the UART completion callback: every step is ISR-safe (a bounded
 * status-register poll, a GPIO write, and atomics). */
static int branch_finalize_transmission(struct branch_tx_state *tx)
{
  int result = branch_wait_transmission_complete(tx->branch_index);
  if (head_board_branch_set_tx(tx->branch_index, false) != 0) result = -EIO;
  if (atomic_get(&tx->abort_requested) != 0) result = -ETIMEDOUT;
  atomic_set(&tx->result, result);
  atomic_clear(&tx->transmission_complete_pending);
  atomic_clear(&tx->abort_requested);
  atomic_clear(&tx->active);
  return result;
}

static void branch_uart_callback(const struct device *device,
                                 struct uart_event *event, void *user_data)
{
  struct branch_tx_state *tx = user_data;
  switch (event->type) {
  case UART_TX_DONE:
    /* DMA completion is not necessarily shift-register completion, so the
     * bounded hardware-complete check still has to happen -- but it happens
     * here, not in the waiting thread. Deferring the release costs a thread
     * wake-up, and any cooperative thread or work item that happens to be
     * running stretches that to hundreds of microseconds. The servo answers
     * about 300 us after the request, so a late release leaves this branch
     * still driving the bus as the reply starts and the leading bytes of the
     * status packet are lost. The check is bounded by one FIFO drain. */
    (void)branch_finalize_transmission(tx);
    k_sem_give(&tx->done);
    break;
  case UART_TX_ABORTED:
    atomic_set(&tx->result,
               atomic_get(&tx->abort_requested) != 0 ? -ETIMEDOUT : -EIO);
    if (head_board_branch_set_tx(tx->branch_index, false) != 0) {
      atomic_set(&tx->result, -EIO);
    }
    atomic_clear(&tx->transmission_complete_pending);
    atomic_clear(&tx->abort_requested);
    atomic_clear(&tx->active);
    k_sem_give(&tx->done);
    break;
  case UART_RX_RDY: {
    const k_spinlock_key_t key = k_spin_lock(&tx->rx_lock);
    const uint32_t stored = ring_buf_put(&tx->rx_ring,
        &event->data.rx.buf[event->data.rx.offset], event->data.rx.len);
    if (stored != event->data.rx.len) {
      ring_buf_reset(&tx->rx_ring);
      atomic_or(&tx->rx_error_flags, BIT(31));
      atomic_inc(&tx->rx_overflow_count);
    }
    k_spin_unlock(&tx->rx_lock, key);
    k_sem_give(&tx->rx_ready);
    break;
  }
  case UART_RX_BUF_REQUEST: {
    bool supplied = false;
    for (uint8_t index = 0u; index < 2u; ++index) {
      if (!atomic_test_and_set_bit(&tx->rx_buffers_used, index)) {
        if (uart_rx_buf_rsp(device, branch_rx_payload[tx->branch_index][index],
                            sizeof(branch_rx_payload[tx->branch_index][index])) != 0) {
          atomic_clear_bit(&tx->rx_buffers_used, index);
          atomic_or(&tx->rx_error_flags, BIT(30));
        } else {
          supplied = true;
        }
        break;
      }
    }
    /* Both buffers already outstanding. The driver is left with nothing to
     * fill, which silently starves the receiver, so record it rather than
     * falling through without a trace. */
    if (!supplied) {
      atomic_inc(&tx->rx_starvation_count);
      /* Surfaced through uart_error_flags so ping-debug reports it without a
       * wire-protocol change: BIT(29) means "no DMA buffer was available". */
      atomic_or(&tx->rx_error_flags, BIT(29));
    }
    break;
  }
  case UART_RX_BUF_RELEASED:
    for (uint8_t index = 0u; index < 2u; ++index) {
      if (event->data.rx_buf.buf == branch_rx_payload[tx->branch_index][index]) {
        atomic_clear_bit(&tx->rx_buffers_used, index);
        break;
      }
    }
    break;
  case UART_RX_STOPPED:
    atomic_or(&tx->rx_error_flags, (atomic_val_t)event->data.rx_stop.reason);
    break;
  case UART_RX_DISABLED:
    atomic_clear(&tx->rx_enabled);
    atomic_inc(&tx->rx_disabled_count);
    k_sem_give(&tx->rx_disabled);
    break;
  default:
    break;
  }
}

static int branch_rx_start(uint8_t branch_index)
{
  struct branch_tx_state *tx = &branch_tx[branch_index];
  atomic_clear(&tx->rx_buffers_used);
  atomic_set_bit(&tx->rx_buffers_used, 0u);
  const int result = uart_rx_enable(branch_uart[branch_index], branch_rx_payload[tx->branch_index][0],
                                    sizeof(branch_rx_payload[tx->branch_index][0]), 100u);
  if (result == 0) {
    atomic_set(&tx->rx_enabled, 1);
  } else {
    atomic_clear(&tx->rx_buffers_used);
    atomic_inc(&tx->rx_restart_failures);
  }
  return result;
}

int head_board_init(void)
{
  for (uint8_t branch_index = 0; branch_index < 4u; ++branch_index) {
    branch_uart[branch_index] = uart_devices[branch_index];
    branch_oe[branch_index] = oe_devices[branch_index];
    if (!device_is_ready(branch_uart[branch_index]) || !device_is_ready(branch_oe[branch_index])) {
      return -ENODEV;
    }
    /* Ux pin 1 (RX enable) is active-low and pin 7 (TX enable) is
     * active-high. Both are tied to this GPIO: low is receive/released,
     * high is transmit. Start in receive so the bus is never driven. */
    if (gpio_pin_configure(branch_oe[branch_index], oe_pins[branch_index],
                           GPIO_OUTPUT_LOW) != 0) {
      return -EIO;
    }
    branch_tx[branch_index].branch_index = branch_index;
    atomic_clear(&branch_tx[branch_index].active);
    atomic_clear(&branch_tx[branch_index].transmission_complete_pending);
    atomic_clear(&branch_tx[branch_index].abort_requested);
    atomic_clear(&branch_tx[branch_index].rx_error_flags);
    k_sem_init(&branch_tx[branch_index].done, 0u, 1u);
    k_sem_init(&branch_tx[branch_index].rx_ready, 0u, 1u);
    k_sem_init(&branch_tx[branch_index].rx_disabled, 0u, 1u);
    ring_buf_init(&branch_tx[branch_index].rx_ring,
                  sizeof(branch_tx[branch_index].rx_storage),
                  branch_tx[branch_index].rx_storage);
    if (uart_callback_set(branch_uart[branch_index], branch_uart_callback,
                          &branch_tx[branch_index]) != 0 || branch_rx_start(branch_index) != 0) {
      return -ENOTSUP;
    }
  }
  if (!pwm_is_ready_dt(&fan_pwm) || !gpio_is_ready_dt(&fan_tach)) return -ENODEV;
  if (gpio_pin_configure_dt(&fan_tach, GPIO_INPUT) != 0) return -EIO;
  gpio_init_callback(&fan_tach_callback, fan_tach_edge, BIT(fan_tach.pin));
  if (gpio_add_callback(fan_tach.port, &fan_tach_callback) != 0 ||
      gpio_pin_interrupt_configure_dt(&fan_tach, GPIO_INT_EDGE_TO_ACTIVE) != 0) {
    return -EIO;
  }
  atomic_clear(&fan_tach_edges);
  fan_rpm_window_ms = k_uptime_get_32();
  return 0;
}

int head_board_branch_set_baud(uint8_t branch_index, uint32_t baudrate)
{
  const struct uart_config config = {
    .baudrate = baudrate,
    .parity = UART_CFG_PARITY_NONE,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .data_bits = UART_CFG_DATA_BITS_8,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
  };

  if (branch_index >= 4u || branch_uart[branch_index] == NULL) return -EINVAL;
  struct branch_tx_state *tx = &branch_tx[branch_index];
  if (atomic_get(&tx->active) != 0) return -EBUSY;
  if (branch_baudrate[branch_index] == baudrate) return 0;
  if (atomic_get(&tx->rx_enabled) != 0) {
    k_sem_reset(&tx->rx_disabled);
    if (uart_rx_disable(branch_uart[branch_index]) != 0 ||
        k_sem_take(&tx->rx_disabled, K_MSEC(5)) != 0) return -EIO;
  }
  const int result = uart_configure(branch_uart[branch_index], &config);
  if (result == 0) branch_baudrate[branch_index] = baudrate;
  const int rx_result = branch_rx_start(branch_index);
  return result != 0 ? result : rx_result;
}

/* Force the receiver back to a known state. head_board_branch_read_available()
 * only restarts a branch whose rx_enabled has been cleared, which relies on
 * UART_RX_DISABLED arriving. A receiver that has run out of DMA buffers stops
 * consuming without ever reporting that event: it stays marked enabled, the
 * hardware FIFO overruns, and no reply is delivered again. Nothing recovers
 * the branch short of a reboot, so the telemetry timeout path calls this to
 * break that wedge. */
int head_board_branch_rx_restart(uint8_t branch_index)
{
  if (branch_index >= 4u || branch_uart[branch_index] == NULL) return -EINVAL;
  struct branch_tx_state *tx = &branch_tx[branch_index];
  if (atomic_get(&tx->active) != 0) return -EBUSY;
  if (atomic_get(&tx->rx_enabled) != 0) {
    k_sem_reset(&tx->rx_disabled);
    /* A starved receiver may never answer, so do not fail on an unconfirmed
     * disable; branch_rx_start() re-arms the buffer accounting regardless. */
    if (uart_rx_disable(branch_uart[branch_index]) == 0) {
      (void)k_sem_take(&tx->rx_disabled, K_MSEC(5));
    }
    atomic_clear(&tx->rx_enabled);
  }
  const k_spinlock_key_t key = k_spin_lock(&tx->rx_lock);
  ring_buf_reset(&tx->rx_ring);
  k_spin_unlock(&tx->rx_lock, key);
  return branch_rx_start(branch_index);
}

int head_board_branch_set_tx(uint8_t branch_index, bool tx)
{
  if (branch_index >= 4u || branch_oe[branch_index] == NULL) return -EINVAL;
  /* The tied complementary enables select TX when high and RX when low. */
  return gpio_pin_set(branch_oe[branch_index], oe_pins[branch_index], tx ? 1 : 0);
}

int head_board_release_all(void)
{
  int result = 0;
  for (uint8_t branch_index = 0u; branch_index < 4u; ++branch_index) {
    if (branch_oe[branch_index] == NULL) continue;
    if (head_board_branch_set_tx(branch_index, false) != 0) result = -EIO;
  }
  return result;
}

uint32_t head_board_branch_error_flags(uint8_t branch_index)
{
  if (branch_index >= 4u || branch_uart[branch_index] == NULL) return 0u;
  return (uint32_t)atomic_set(&branch_tx[branch_index].rx_error_flags, 0) |
         (uint32_t)uart_err_check(branch_uart[branch_index]);
}

int head_board_branch_uart_tx_meter_test(uint8_t branch_index, uint32_t duration_ms)
{
  if (branch_index >= 4u || duration_ms == 0u) return -EINVAL;
  int result = head_board_branch_set_tx(branch_index, true);
  if (result == 0) {
    const int64_t deadline = k_uptime_get() + duration_ms;
    /* 0x00 at 1 Mbps keeps the DATA wire low for nine of every ten bits.
     * It cannot form a Dynamixel Protocol 2.0 header. */
    while (k_uptime_get() < deadline) uart_poll_out(branch_uart[branch_index], 0x00u);
    /* Give the UART FIFO time to drain before releasing the bus. */
    result = branch_wait_transmission_complete(branch_index);
    const int release_result = head_board_branch_set_tx(branch_index, false);
    if (release_result != 0) result = release_result;
  }
  return result;
}

int head_board_branch_uart_rx_line_test(uint8_t branch_index, uint32_t duration_ms,
                                        uint32_t *bytes, uint32_t *errors)
{
  if (branch_index >= 4u || duration_ms == 0u || bytes == NULL || errors == NULL) return -EINVAL;
  *bytes = 0u;
  *errors = 0u;
  int result = head_board_branch_set_tx(branch_index, false);
  if (result != 0) return result;
  const int64_t deadline = k_uptime_get() + duration_ms;
  while (k_uptime_get() < deadline) {
    uint8_t incoming[64];
    *bytes += head_board_branch_read_available(branch_index, incoming, sizeof(incoming));
    *errors |= head_board_branch_error_flags(branch_index);
    k_sleep(K_MSEC(1));
  }
  return 0;
}

int head_board_branch_write(uint8_t branch_index, const uint8_t *data, size_t length)
{
  int result = head_board_branch_write_async(branch_index, data, length);
  if (result != 0) return result;
  const uint32_t wire_us = ((uint32_t)length * 10000000u +
                            branch_baudrate[branch_index] - 1u) /
                           branch_baudrate[branch_index];
  return head_board_branch_write_wait(branch_index, wire_us + 1000u);
}

int head_board_branch_write_async(uint8_t branch_index, const uint8_t *data, size_t length)
{
  struct branch_tx_state *tx;
  int result;
  if (branch_index >= 4u || branch_uart[branch_index] == NULL || data == NULL ||
      length == 0u || length > HEAD_BRANCH_TX_CAPACITY) return -EINVAL;
  tx = &branch_tx[branch_index];
  if (atomic_get(&tx->active) != 0) {
    if (atomic_get(&tx->transmission_complete_pending) == 0) return -EBUSY;
    (void)branch_finalize_transmission(tx);
  }
  if (!atomic_cas(&tx->active, 0, 1)) return -EBUSY;
  memcpy(branch_tx_payload[tx->branch_index], data, length);
  tx->length = length;
  atomic_set(&tx->result, -EINPROGRESS);
  atomic_clear(&tx->transmission_complete_pending);
  atomic_clear(&tx->abort_requested);
  k_sem_reset(&tx->done);
  result = head_board_branch_set_tx(branch_index, true);
  if (result == 0) result = uart_tx(branch_uart[branch_index], branch_tx_payload[tx->branch_index], tx->length,
                                    SYS_FOREVER_US);
  if (result != 0) {
    atomic_clear(&tx->active);
    (void)head_board_branch_set_tx(branch_index, false);
  }
  return result;
}

int head_board_branch_write_wait(uint8_t branch_index, uint32_t timeout_us)
{
  struct branch_tx_state *tx;
  if (branch_index >= 4u || branch_uart[branch_index] == NULL) return -EINVAL;
  tx = &branch_tx[branch_index];
  if (atomic_get(&tx->active) == 0) return (int)atomic_get(&tx->result);
  if (k_sem_take(&tx->done, K_USEC(timeout_us)) != 0) {
    atomic_set(&tx->abort_requested, 1);
    atomic_set(&tx->result, -ETIMEDOUT);
    (void)uart_tx_abort(branch_uart[branch_index]);
    /* A successful abort produces UART_TX_ABORTED. If completion raced with
     * the timeout, UART_TX_DONE may arrive instead and is finalized here. If
     * neither event arrives within this bounded recovery wait, ownership is
     * deliberately retained so the DMA buffer cannot be reused. */
    if (k_sem_take(&tx->done, K_USEC(1000u)) == 0 &&
        atomic_get(&tx->transmission_complete_pending) != 0) {
      (void)branch_finalize_transmission(tx);
    }
    return -ETIMEDOUT;
  }
  /* UART_TX_DONE already released the bus in the callback. This remains only
   * as a fallback for a completion that reached the thread without it. */
  if (atomic_get(&tx->transmission_complete_pending) != 0) {
    return branch_finalize_transmission(tx);
  }
  return (int)atomic_get(&tx->result);
}

int head_board_branch_read(uint8_t branch_index, uint8_t *data, size_t length,
                           uint32_t timeout_us)
{
  uint32_t started = k_cycle_get_32();
  uint32_t limit = started + k_us_to_cyc_ceil32(timeout_us);
  size_t used = 0u;

  if (branch_index >= 4u || branch_uart[branch_index] == NULL || data == NULL) return -EINVAL;
  while (used < length && (int32_t)(k_cycle_get_32() - limit) < 0) {
    used += head_board_branch_read_available(branch_index, &data[used], length - used);
    if (used < length) {
      const int32_t remaining_cycles = (int32_t)(limit - k_cycle_get_32());
      if (remaining_cycles <= 0) break;
      (void)k_sem_take(&branch_tx[branch_index].rx_ready,
          K_USEC(k_cyc_to_us_ceil32((uint32_t)remaining_cycles)));
    }
  }
  return used == length ? 0 : -ETIMEDOUT;
}

size_t head_board_branch_read_available(uint8_t branch_index, uint8_t *data,
                                        size_t capacity)
{
  uint32_t used;
  if (branch_index >= 4u || branch_uart[branch_index] == NULL || data == NULL) return 0u;
  if (atomic_get(&branch_tx[branch_index].rx_enabled) == 0 &&
      branch_rx_start(branch_index) != 0) return 0u;
  const k_spinlock_key_t key = k_spin_lock(&branch_tx[branch_index].rx_lock);
  used = ring_buf_get(&branch_tx[branch_index].rx_ring, data, capacity);
  k_spin_unlock(&branch_tx[branch_index].rx_lock, key);
  return (size_t)used;
}

void head_board_branch_rx_stats(uint8_t branch_index, uint16_t *disabled,
                                uint16_t *restart_failures, uint16_t *overflows)
{
  if (branch_index >= 4u) return;
  const struct branch_tx_state *tx = &branch_tx[branch_index];
  if (disabled != NULL) *disabled = (uint16_t)atomic_get(&tx->rx_disabled_count);
  if (restart_failures != NULL) {
    *restart_failures = (uint16_t)atomic_get(&tx->rx_restart_failures);
  }
  if (overflows != NULL) *overflows = (uint16_t)atomic_get(&tx->rx_overflow_count);
}

int head_board_branch_wait_rx(uint8_t branch_index, uint32_t timeout_us)
{
  if (branch_index >= 4u || branch_uart[branch_index] == NULL) return -EINVAL;
  /* Sleeping here is what lets the driver's cooperative idle-timeout work run
   * and move a short reply out of the DMA buffer into the ring. */
  return k_sem_take(&branch_tx[branch_index].rx_ready, K_USEC(timeout_us)) == 0 ?
         0 : -ETIMEDOUT;
}

int head_board_fan_set_percent(uint8_t percent)
{
  if (percent > 100u || !pwm_is_ready_dt(&fan_pwm)) return -EINVAL;
  return pwm_set_dt(&fan_pwm, fan_pwm.period,
                    (fan_pwm.period * percent) / 100u);
}

uint16_t head_board_fan_rpm(void)
{
  const uint32_t now = k_uptime_get_32();
  const uint32_t elapsed = now - fan_rpm_window_ms;
  if (elapsed >= 500u) {
    /* Noctua tach output is two pulses per revolution. Atomic exchange makes
     * the ISR/window handoff lossless enough for cooling supervision. */
    const uint32_t edges = (uint32_t)atomic_set(&fan_tach_edges, 0);
    const uint32_t calculated = (edges * 30000u) / elapsed;
    fan_rpm = calculated > UINT16_MAX ? UINT16_MAX : (uint16_t)calculated;
    fan_rpm_window_ms = now;
  }
  return fan_rpm;
}

#endif /* HEAD_BENCH_NO_12V */
