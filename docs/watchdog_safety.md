# WDOG0 supervision and motor-output containment

**Implementation status:** WDOG0 foreground supervision is integrated in
`firmware/tiva/src/main.c`. The complete TivaWare target has not yet been
compiled, and watchdog timing/reset behavior has not been measured on a
TM4C123GH6PM board.

## Firmware behavior

- Device: TM4C123GH6PM, Watchdog Timer 0 (WDOG0).
- Clock source: system clock, configured by the firmware for 80 MHz.
- Reload: 4,000,000 counts, nominally 50 ms for the first timeout.
- Reset: enabled on the second timeout if the first-timeout flag remains
  uncleared.
- Debug behavior: watchdog stall during debugger halt is enabled.
- Service policy: foreground code checks the raw WDOG0 interrupt flag and
  clears it after setup progress, completed gyro-calibration samples, and
  completed control-loop tick waits.

The service call is deliberately not placed in a timer ISR. If the foreground
blocks waiting for Timer0A, stalls in a peripheral operation, or stops
progressing, it cannot keep servicing WDOG0. The intended reset time is
nominally about 100 ms from a persistent stall, but this must be measured on
the actual target and depends on the configured clock and watchdog behavior.

## Safety boundary

WDOG0 is clocked from the system clock. It is not an independent-clock
monitor and does not protect against failure of that clock source. A watchdog
reset restarts the MCU; it does **not** directly disconnect motor-battery
power.

During reset, MCU GPIO pins may be high-impedance. Therefore:

- Verify that the exact TB6612FNG IC/breakout holds STBY low during MCU reset,
  power-up, and MCU-unpowered conditions.
- Fit an external STBY pull-down if the breakout's behavior is not guaranteed.
- Keep a correctly rated fuse and an accessible physical disconnect in the
  motor-battery circuit.
- Keep `MOTOR_OUTPUT_ALLOWED 0` until the electrical, mechanical and motor
  polarity reviews are complete.
- Keep motor power isolated during all initial watchdog and firmware tests.

The software watchdog is additional fault containment; it is not a substitute
for an independent motor-power cutoff or emergency-stop circuit.

## Required target verification

Perform these checks only with motor power physically disconnected:

1. Build the complete project for the exact TM4C123GH6PM using the installed
   CCS and TivaWare versions. Record compiler, SDK and driverlib versions.
2. Flash the image with `MOTOR_OUTPUT_ALLOWED 0`; verify the board starts in
   the safe/off state and PA5/STBY remains low.
3. Probe PA5 and the driver STBY pin through power-up, manual reset, watchdog
   reset and MCU-unpowered states.
4. In a temporary test build, deliberately stall the foreground after
   watchdog initialization without clearing WDOG0. Observe reset on the
   second timeout and measure the elapsed time with a scope or logic analyzer.
5. Remove the deliberate stall and verify normal startup calibration and
   sustained 200 Hz loop operation do not cause watchdog resets.
6. Repeat the reset test with driver logic connected but motor supply isolated.
   Confirm the physical STBY state is safe throughout.

Do not treat the nominal 50 ms/100 ms values as measured results until the
waveforms have been captured and archived with the firmware commit SHA.

## Evidence status

| Evidence | Status |
|---|---|
| WDOG0 API calls present in firmware source | Implemented |
| Foreground servicing in startup/calibration/control paths | Implemented |
| Portable host tests for TivaWare watchdog peripheral behavior | Not applicable; target peripheral |
| CCS/TivaWare target compilation | Pending |
| Measured first/second timeout | Pending |
| Deliberate-stall reset test on board | Pending |
| STBY low during reset and MCU-unpowered states | Pending |
| Independent motor-power cutoff test | Pending |

## References

- [TI EK-TM4C123GXL resources](https://www.ti.com/tool/EK-TM4C123GXL)
- [TI TM4C123GH6PM datasheet](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf)
- [TI TivaWare Peripheral Driver Library User's Guide](https://software-dl.ti.com/tiva-c/SW-TM4C/latest/exports/SW-TM4C-DRL-UG-2.1.4.178.pdf)
- [Toshiba TB6612FNG product documentation](https://toshiba.semicon-storage.com/eu/semiconductor/product/motor-driver-ics/brushed-dc-motor-driver-ics/detail.TB6612FNG.html)
