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
2. Remove the example's competing `main` function. Add `src/main.c`, `src/controller.c`; do **not** compile helper headers as separate units. They are locally included by `main.c`.
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

Before using PWM, inspect the board's PB6↔PD0 and PB7↔PD1 hard links, board revision and pin mux for conflicts.

## 4. State machine and control

The pure C safety logic in `controller.c` starts DISARMED, requires a prior switch release, then 40 accepted upright samples (≤8°) while held before ARMED. On release it disarms. On ≥25° tip or invalid sample it faults until release with valid input. The application separately treats fatal IMU communication errors as latched until reset.

The gyro offset is estimated from 200 samples assuming the board is stationary; no measured variance/stillness verification yet. A complementary filter combines `atan2(ay, az)` with bias-corrected gyro X and a nominal 5 ms interval. The PID controller clamps command to ±0.7 and avoids integral windup.

**Polarity flags in `main.c` are provisional.** Verify both motors and sensor signs with wheels raised and power limited before enabling driver output.

## 5. Important limits

- Timing uses a nominal 5 ms dt, not independently recorded real-time execution times; actual missed ticks may not be detected by the current polling approach.
- I2C transaction handling has a software-bounded busy poll; timeouts, bus recovery, transaction sequencing and maximum duration still require real-target tests.
- No wheel encoders, independent watchdog, UART logger, hardware emergency switch design, current monitoring, real sensor calibration report or power-stage approval exist.
- Current MATLAB LQR is an illustrative comparison, not deployed code. Control gains are placeholders.
- Software output gating does not substitute for power isolation and mechanical restraint.

Next evidence: capture CCS build log, scope traces (PB6/PB7/PA5), static IMU readings and measured loop timing; then proceed through [verification_plan.md](../../docs/verification_plan.md).
