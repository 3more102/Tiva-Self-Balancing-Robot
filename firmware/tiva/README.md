# TivaWare firmware bring-up: EK-TM4C123GXL / TM4C123GH6PM

**Build status:** MCU target not compiled or board-tested. Portable control/safety routines are covered separately by GCC host tests; they do not test TivaWare I²C/PWM or electrical behavior.

## 1. Expected environment

- Texas Instruments Code Composer Studio (CCS)
- Device `TM4C123GH6PM` on the EK-TM4C123GXL LaunchPad
- TivaWare Peripheral Driver Library, include paths and compiler-matched `driverlib`
- Correct device startup/vector and linker command files, ideally from a known working official `blinky` example for this exact device
- Physical motor power cutoff, current-limited supply for bench tests, scope/logic analyzer and multimeter

Official platform: https://www.ti.com/tool/EK-TM4C123GXL

## 2. Building the draft

1. Import/create a CCS target project for the exact TM4C123GH6PM. Verify the official example builds and flashes first.
2. Remove the example's competing `main` function. Add `src/main.c`, `src/controller.c`, `src/diagnostics.c`, `src/imu_decode.c`, `src/range_sensing.c`, `src/ai_sensor_adapter.c`, `src/ai_inference.c`, and `src/ai_navigation.c`; do **not** compile helper headers as separate units. They are locally included by `main.c`.
3. Add `include` and the TivaWare root to include directories, and link the correct version of the TivaWare driverlib.
4. Enable appropriate Cortex-M4F FPU ABI/compiler settings and `math` routines (`atan2f`, `fabsf`, `isfinite`, `NAN`).
5. **Do not alter `MOTOR_OUTPUT_ALLOWED 0`** during first sensor/logic tests. Verify STBY remains low.
6. Capture any compilation errors verbatim. Apply corrections only against the actual target version, then document CCS/TivaWare versions and commit SHA.

## 3. Pin and startup assumptions

| Peripheral | Pins | Firmware expectation |
|---|---|---|
| I2C0 | PB2 SCL, PB3 SDA | MPU6050 at `0x68`, AD0 low, properly voltage-limited 3.3 V bus |
| PWM0 GEN0 | PB6 M0PWM0, PB7 M0PWM1 | 20 kHz target, TB6612 PWMA/PWMB |
| Motor direction | PC4, PC5, PC6, PC7 | Driver AIN1/AIN2/BIN1/BIN2 |
| Driver enable | PA5 | **STBY low except authorized output** |
| User switch | PF4 | Onboard SW1, active low, hold-to-enable only after initial release |
| Status | PF1 (red), PF2 (blue), PF3 (green) | Fault, disabled, enabled; unverified target |
| Timer0A | No pin | 200 Hz desired periodic sample wake-up |
| Timer1A | No pin | 32-bit free-running downcounter used to calculate elapsed control dt |
| Timer2A | No pin | One-shot 12 µs trigger pulse when the optional HC-SR04 backend is enabled |
| GPIOE | PE0–PE5 | Optional front/left/right TRIG/ECHO pins; backend disabled by default |

Before using PWM, inspect the board's PB6↔PD0 and PB7↔PD1 hard links, board revision and pin mux for conflicts.

## 4. Range-sensor integration status

The portable capture/scheduling layer is in `include/range_sensing.h` and `src/range_sensing.c`; `ai_sensor_adapter.c` maps a checked `RangeSnapshot` into the TinyML input frame. It uses a round-robin front/left/right schedule, a 60 ms inter-ping gap, a 13 ms echo timeout, pulse-width validation and freshness limits. Host tests cover pulse conversion, timeout, 80 MHz timer ticks, rollover, invalid pulses and freshness.

An **optional HC-SR04-compatible Tiva adapter** is present in `src/range_backend.h`, but `RANGE_HCSR04_BACKEND_ENABLED` defaults to `0`. Its current provisional map uses PE0/PE1 (front TRIG/ECHO), PE2/PE3 (left), PE4/PE5 (right), Timer2A for a 12 µs trigger pulse and Port E GPIO edge interrupts for Echo. Timer1A is read in the GPIO ISR to timestamp edges. **This is not GPTM CCP hardware input capture** and has unmeasured interrupt latency; PE0–PE5 do not expose CCP alternate functions on this MCU.

TI's TM4C123GH6PM datasheet lists WT3CCP0/PD2, WT3CCP1/PD3 and WT5CCP0/PD6 as candidate hardware-capture inputs; the LaunchPad manual maps these to J3.5, J3.6 and J4.8. They are documented as a candidate future Echo map in [hardware/wiring.md](../../hardware/wiring.md), not yet wired or implemented. A future CCP backend must preserve the existing control timebase and trigger timer.

**Do not enable the current adapter until your actual sensor model and wiring are confirmed.** A 5 V Echo output must be translated to a safe logic level before reaching a TM4C GPIO. The adapter has not been compiled with your TivaWare installation or tested on a board. With the default disabled setting, the range readings remain invalid and the AI correctly reports STOP. See [range-sensing design](../../docs/range_sensing.md).

## 5. State machine and control

The pure C safety logic in `controller.c` starts DISARMED, requires a prior switch release, then 40 accepted upright samples (≤8°) while held before ARMED. On release it disarms. On ≥25° tip or invalid sample it faults until release with valid input. The application separately treats fatal IMU communication errors as latched until reset.

Pitch gyro X and vertical/yaw gyro Z offsets are estimated only after 200 **consecutive** valid, stationary, upright samples with SW1 released, subject to the provisional sample-variance limit on both axes and a nominal 10-second startup attempt window. Timer1A measures elapsed per-cycle dt; the software rejects loops outside the provisional 2.5–8 ms window. **Neither real timing nor sensor noise has been measured on a board.** A complementary filter combines `atan2(ay, az)` with bias-corrected gyro X. The PID controller clamps command to ±0.7 and avoids integral windup.

**Polarity flags in `main.c` are provisional.** Verify both motors and sensor signs with wheels raised and power limited before enabling driver output.


The MPU6050 burst decoder is kept in portable `src/imu_decode.c` and is
host-tested independently. The target backend reads gyro X for pitch control
and gyro Z for vertical-axis yaw sensing; both biases are calibrated together.
The main loop now builds a range/yaw input frame and runs TinyML inference at
50 Hz in **shadow mode**. Inspect `g_ai_action_debug`,
`g_ai_confidence_debug`, `g_ai_safety_override_debug`, and
`g_yaw_rate_debug_dps` in CCS. The action is deliberately not connected to
the balance target or motor mixer; `MOTOR_OUTPUT_ALLOWED` remains zero.

## 6. Watchdog supervision

WDOG0 is initialized after the outputs are placed in their safe/off state. At the configured 80 MHz system clock, the reload value is 4,000,000 counts (nominally 50 ms). The foreground checks the raw first-timeout flag and clears it only after setup/calibration progress or a completed control-loop tick; if the foreground stops progressing, the uncleared second timeout resets the MCU. The watchdog is configured to stall while the debugger is halted.

This is **source-level integration only**: CCS/TivaWare compilation, measured reset timing and deliberate-stall testing on the TM4C123 are still outstanding. WDOG0 shares the system clock and does not directly de-energize the motor supply. The external STBY pull-down, fused motor supply and accessible physical disconnect remain mandatory. See [watchdog safety and target test procedure](../../docs/watchdog_safety.md).

## 7. Important limits

- Timer1A elapsed-time measurement improves over a constant 5 ms assumption, but no hardware measurements or worst-case execution timing exist. Timer0A flag coalescing, CPU hangs and counter overflow/aliasing after ~53.7 s remain limitations.
- I2C transaction handling has a software-bounded busy poll; timeouts, bus recovery, transaction sequencing and maximum duration still require real-target tests.
- The current optional ultrasonic backend timestamps Port E GPIO interrupts with Timer1A; it is not GPTM CCP hardware capture. A CCP pin map is documented, but the capture backend is still pending.
- WDOG0 foreground supervision is integrated in `src/main.c`: nominal 50 ms first timeout, reset on the second unserviced timeout, and servicing after completed foreground progress. It has **not** been target-compiled or fault-injection tested. It uses the system clock and cannot remove motor power.
- No wheel encoders, independent-clock watchdog, UART logger, verified hardware emergency-stop circuit, current monitoring, real sensor calibration report or power-stage approval exist.
- Current MATLAB LQR is an illustrative comparison, not deployed code. Control gains are placeholders.
- Software output gating does not substitute for power isolation and mechanical restraint.

Next evidence: capture CCS build log, scope traces (PB6/PB7/PA5), static IMU readings and measured loop timing; then proceed through [verification_plan.md](../../docs/verification_plan.md) and [calibration / timing diagrams](../../docs/calibration_timing.md).
