# Watchdog reset and motor-output containment

**Status:** WDOG0 is integrated in the Tiva-C source. It has **not** been compiled with the target toolchain or verified on a TM4C123GH6PM board.

## Implementation

- Device: TM4C123GH6PM, Watchdog Timer 0 (WDOG0).
- Clock: system clock. At the configured 80 MHz, the load value is 4,000,000 counts for a nominal 50 ms first timeout.
- WatchdogResetEnable() enables reset on the **second** timeout if the first timeout interrupt remains uncleared.
- The `watchdog_service()` function checks the raw WDOG0 interrupt flag after a completed control-loop sample and clears a pending first-timeout flag. Startup calibration samples are also serviced.
- WatchdogStallEnable() pauses the watchdog while the debugger halts the processor.
- If the control loop stops servicing the watchdog, the first timeout occurs after up to approximately 50 ms and the second timeout resets the MCU after another approximately 50 ms. Actual timing must be measured on the target.

The watchdog is serviced by foreground code, not a timer interrupt. A CPU stuck while waiting for a tick, reading the IMU, running the control logic or updating outputs therefore stops servicing it.

## Safety boundary

WDOG0 uses the system clock; it does not protect against failure of that clock source. The watchdog resets the MCU, but it does **not** directly remove motor-battery power. During reset, GPIO PA5 is not guaranteed to actively hold TB6612FNG STBY low.

Before any motor supply is connected:

1. The Toshiba TB6612FNG datasheet specifies an internal 200 kΩ pull-down on STBY. Verify that the exact driver IC and breakout/module preserve this feature. If it is absent or uncertain, add an external pull-down sized for the module and logic voltage.
2. Verify STBY stays below the driver's standby threshold during power-up, manual reset, watchdog reset and MCU-unpowered conditions.
3. Provide a separate, accessible physical disconnect in the motor-battery circuit and appropriate fuse/current limiting.
4. Keep MOTOR_OUTPUT_ALLOWED 0 until the full electrical and mechanical preflight is signed off.

A software watchdog, SW1 and an LED indication are not substitutes for a physical motor-power disconnect.

## Target verification procedure

1. Compile and flash for the exact TM4C123GH6PM using the installed CCS/TivaWare versions. Record the compiler and library versions.
2. Keep the motor battery disconnected. Probe PA5 and TB6612 STBY through power-up and reset; verify the driver's internal pull-down (or a fitted external pull-down if required) holds STBY low.
3. In a temporary test build, insert a deliberate non-serviced infinite loop immediately after watchdog_init(). Keep MOTOR_OUTPUT_ALLOWED 0.
4. Observe that WDOG0 resets the MCU after its second timeout. Measure the interval with a scope or logic analyzer; do not treat the nominal 100 ms as measured until captured.
5. Remove the deliberate hang and verify the normal 200 Hz loop does not cause watchdog resets during startup calibration and extended idle operation.
6. Repeat the reset test with driver logic connected but motor power isolated. Confirm STBY remains low across reset and MCU-unpowered states.

## References

- [Toshiba TB6612FNG datasheet](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf): lists a nominal 200 kΩ internal pull-down on STBY; verify the actual module or clone.
- [TI TM4C123GH6PM datasheet](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf): WDOG0 is clocked by the system clock; the watchdog can interrupt on its first timeout and reset on its second timeout.
- [TI TivaWare Peripheral Driver Library User's Guide, Chapter 33](https://software-dl.ti.com/tiva-c/SW-TM4C/latest/exports/SW-TM4C-DRL-UG-2.1.4.178.pdf): WatchdogReloadSet, WatchdogResetEnable, WatchdogEnable, WatchdogIntClear and WatchdogStallEnable.