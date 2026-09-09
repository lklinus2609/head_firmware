# Direction line-test watchdog fix

## Symptom observed

With production firmware flashed and no servos connected, this command printed
`Holding J1 in TX mode for 10 seconds; measure now.` and then failed with
`OSError: [Errno 5] Input/output error`. The Teensy reset and USB CDC briefly
disappeared.

## Before

`LINE_MODE_TEST` set the direction GPIO and then slept for ten seconds inside
`dispatch_frame()` while holding `head_lock`. The control thread also needed
`head_lock` before it could feed the one-second hardware watchdog. The watchdog
therefore expired during the diagnostic, resetting the MCU before the generic
completion ACK could be sent.

## After

All three long electrical tests now reserve diagnostic ownership and run in
short slices on a dedicated worker. The worker releases `head_lock` between
slices, so control supervision and USB dispatch continue. Normal telemetry and
motion preparation are excluded until the worker releases the direction line,
restores the configured production baud, and sends the completion ACK.
Overlapping tests are rejected. Disable requests cancel the reservation.
TX-meter output is a sequence of bursts with supervision gaps; its average
meter voltage is not a calibrated logic-level test.

The existing fatal/reset release path still forces all direction lines low.

## Debug checklist if this regresses

1. Keep ACM0 open and run one branch only.
2. Confirm the selected direction line rises and returns low after ten seconds.
3. Confirm the command exits successfully and ACM1 remains present.
4. Check ACM0 for a new Zephyr boot banner; a new banner indicates an MCU reset.
5. A `FAULT CONFIGURATION` status is expected until a valid calibration profile
   exists; it is not by itself evidence of this bug.
6. Do not attach servos or proceed to motion testing until all four branches
   pass.

## Verification

Build and flash the production image, then run `line-test` once for branches 0
through 3 while measuring the matching direction nets. The test remains
torque-safe and requires no connected servo.
