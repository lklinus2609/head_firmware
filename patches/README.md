# Patches to vendored code

`deps/zephyr` is managed by `west` (`zephyr/west.yml`, Zephyr v3.7.2). A
`west update` resets it and removes these changes. Re-apply them from
`firmware/`:

```sh
git -C deps/zephyr apply ../../patches/zephyr-v3.7.2-rx-fixes.patch
```

Check that they are applied (prints the two files):

```sh
git -C deps/zephyr diff --stat
```

## `zephyr-v3.7.2-rx-fixes.patch`

Receive-path fixes found by the 2026-10-06 bus statistics
(`docs/reviews/FIRMWARE_AUDIT_2026-10-05.md`, R1, R2, W6):

- `drivers/serial/uart_mcux_lpuart.c`
  - The RX-timeout flush and the DMA completion interrupt both updated
    `rx_dma_params` without a lock, which delivered receive data out of order
    (seen as paired `rx_gaps`/`rx_overlaps`). The flush now runs with
    interrupts locked. Backport of upstream commit `d73e311f4a`.
  - The receiver stores nothing while the overrun flag is set, and the async
    path never cleared it. The overrun interrupt is now enabled; the ISR
    clears the flag at once and reports `UART_RX_STOPPED` with
    `UART_ERROR_OVERRUN`, with RX left enabled. The head firmware counts it.
- `drivers/dma/dma_mcux_edma.c`
  - The error interrupt never cleared an error on a channel not marked busy, so
    that interrupt could fire without end and starve every thread (the
    2026-10-06 watchdog reset). Such an error is now cleared. Busy channels are
    handled as before: a 2026-10-06 bisect showed that aborting only the
    channel in error made receive worse (destination bus errors, silent
    timeouts), so the original abort of every busy channel stays.
  - Errors and the ES register at the last error are kept per channel in
    `dma_mcux_edma_channel_error_count` and
    `dma_mcux_edma_channel_last_error_status` for the bus statistics.
