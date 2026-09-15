#ifndef HEAD_BOARD_H_
#define HEAD_BOARD_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#define HEAD_BRANCH_DEFAULT_BAUD 1000000u

int head_board_init(void);
int head_board_branch_set_baud(uint8_t branch_index, uint32_t baudrate);
/* Select the half-duplex buffer direction. tx=false releases bus drive. */
int head_board_branch_set_tx(uint8_t branch_index, bool tx);
/* Release every half-duplex driver. Safe to call repeatedly on fault paths. */
int head_board_release_all(void);
uint32_t head_board_branch_error_flags(uint8_t branch_index);
/* Emits invalid zero bytes at the configured baud for a bounded electrical test. */
int head_board_branch_uart_tx_meter_test(uint8_t branch_index, uint32_t duration_ms);
/* Listens only and reports received bytes/UART error flags for a bounded test. */
int head_board_branch_uart_rx_line_test(uint8_t branch_index, uint32_t duration_ms,
                                        uint32_t *bytes, uint32_t *errors);
/* Async start/wait permits all four physical UARTs to transmit concurrently. */
int head_board_branch_write_async(uint8_t branch_index, const uint8_t *data, size_t length);
int head_board_branch_write_wait(uint8_t branch_index, uint32_t timeout_us);
int head_board_branch_write(uint8_t branch_index, const uint8_t *data, size_t length);
int head_board_branch_read(uint8_t branch_index, uint8_t *data, size_t length,
                           uint32_t timeout_us);
size_t head_board_branch_read_available(uint8_t branch_index, uint8_t *data,
                                        size_t capacity);
/* Block until the driver delivers received bytes, or the timeout expires.
 * A short Dynamixel reply never fills a DMA buffer, so it only reaches the
 * ring buffer when the UART idle-timeout work item runs on the cooperative
 * system workqueue. Callers must therefore sleep rather than poll; a busy
 * wait starves that work item and the reply is never delivered. */
int head_board_branch_wait_rx(uint8_t branch_index, uint32_t timeout_us);
/* Recover a receiver that stopped consuming without reporting RX_DISABLED. */
int head_board_branch_rx_restart(uint8_t branch_index);
/* Branch-lifetime RX health counters, for diagnosing dropped replies. */
void head_board_branch_rx_stats(uint8_t branch_index, uint16_t *disabled,
                                uint16_t *restart_failures, uint16_t *overflows);
int head_board_fan_set_percent(uint8_t percent);
uint16_t head_board_fan_rpm(void);

#endif
