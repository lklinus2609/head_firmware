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
/* Receive-error counts since boot. The UART flags are sampled each time
 * head_board_branch_error_flags() runs, so a count is the number of samples
 * that saw the flag: a lower bound on events. Parity is disabled on every
 * branch, so the driver's PARITY report is the LPUART noise flag. */
struct head_board_error_counts {
  uint32_t overrun_count;
  uint32_t framing_error_count;
  uint32_t noise_error_count;
  uint32_t ring_overflow_count;
  uint32_t dma_starvation_count;
  /* UART_RX_RDY ranges that did not continue where the previous one in the
   * same DMA buffer ended: skipped bytes (gap) or bytes delivered twice. */
  uint32_t rx_gap_count;
  uint32_t rx_overlap_count;
  /* eDMA errors on this branch's receive and transmit channels, counted by
   * the patched dma_mcux_edma.c (firmware/patches/). */
  uint32_t dma_rx_error_count;
  uint32_t dma_tx_error_count;
  /* Error-flag checks that found the whole eDMA halted (CR.HALT, set by any
   * error while CR.HOE is on), and the eDMA ES register at the last error on
   * this branch's receive channel. */
  uint32_t dma_halt_sample_count;
  uint32_t dma_rx_last_error_status;
};
void head_board_branch_error_counts(uint8_t branch_index,
                                    struct head_board_error_counts *counts);
/* Raw registers for the watchdog black box; plain reads, safe in an ISR.
 * eDMA ES (error status), ERR and INT (one bit per channel). */
void head_board_dma_error_registers(uint32_t *error_status, uint32_t *error_channels,
                                    uint32_t *interrupt_channels);
/* LPUART STAT register of a branch. */
uint32_t head_board_branch_uart_status(uint8_t branch_index);
int head_board_fan_set_percent(uint8_t percent);
uint16_t head_board_fan_rpm(void);

/* Serial-port event counts per branch, for the watchdog black box in main.c.
 * Index by Zephyr's enum uart_event_type (UART_TX_DONE .. UART_RX_STOPPED).
 * Kept in uncached no-init DTCM by board.c so it can survive a reset. */
#define HEAD_BOARD_UART_EVENT_TYPE_COUNT 7u
struct head_board_trace {
  uint32_t uart_event_count[4][HEAD_BOARD_UART_EVENT_TYPE_COUNT];
  uint32_t uart_last_event_ms[4];
};
extern struct head_board_trace head_board_trace;

#endif
