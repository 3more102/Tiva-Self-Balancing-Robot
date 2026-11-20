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
2. Remove the example's competing `main` function. Add `src/main.c`, `src/controller.c`, `src/diagnostics.c`, `src/navigation.c`, `src/range_capture.c`, `src/range_scanner.c`, and `src/range_navigation.c`; do **not** compile helper headers as separate units. The IMU and motor backend headers are locally included by `main.c`. The navigation policy is called by `main.c`; the portable range-capture, three-sensor scanner and scanner-to-navigation adapter modules are host-tested but are not yet called by main.c. They still need a TivaWare GPIO/timer adapter, physical pin mapping and trigger-pulse implementation. Autonomy remains disabled until range acquisition and a separate operator enable are implemented.
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
| Timer1A | No pin | 80 MHz 32-bit free-running downcounter for elapsed control dt; `~TimerValueGet()` supplies an increasing tick timestamp to the portable range-capture core |
| WDOG0 | No pin | System-clock watchdog; 50 ms first timeout, reset on second uncleared timeout; serviced by foreground code |

Before using PWM, inspect the board's PB6↔PD0 and PB7↔PD1 hard links, board revision and pin mux for conflicts.

## 4. State machine and control

The pure C safety logic in `controller.c` starts DISARMED, requires a prior switch release, then 40 accepted upright samples (≤8°) while held before ARMED. On release it disarms. On ≥25° tip or invalid sample it faults until release with valid input. The application separately treats fatal IMU communication errors as latched until reset.

The gyro offset is estimated only after 200 **consecutive** valid, stationary, upright samples with SW1 released, subject to the provisional sample-variance limit and a nominal 10-second startup attempt window. Timer1A measures elapsed per-cycle dt; the software rejects loops outside the provisional 2.5–8 ms window. **Neither real timing nor sensor noise has been measured on a board.** A complementary filter combines `atan2(ay, az)` with bias-corrected gyro X. The PID controller clamps command to ±0.7, limits commanded pitch targets to ±3° and avoids integral windup; all gains/limits remain provisional.

**Polarity flags in `main.c` are provisional.** Verify both motors and sensor signs with wheels raised and power limited before enabling driver output.

## 5. Obstacle navigation status

The portable front/left/right navigation state machine is in `src/navigation.c` and its interface is in `include/navigation.h`. Its host tests are in `../../tests/test_navigation.c`. The reusable echo-pulse capture core is in `src/range_capture.c` with interface `include/range_capture.h`. The three-sensor round-robin scheduler is in `src/range_scanner.c` with interface `include/range_scanner.h`; it enforces one active ping, a configurable quiet interval, timeout publication and timestamped samples. `src/range_navigation.c` maps the latest scanner samples into `NavigationInput` without changing the separate autonomy-enable gate. These portable modules have host tests and use the Timer1A tick domain. They are not connected to TivaWare GPIO interrupts or physical TRIG pins yet. `main.c` routes navigation requests through the balance controller and differential mixer, but autonomy is hard-disabled and range inputs remain invalid. The physical sensor pin map and separate autonomy-enable input are not present yet. See [obstacle avoidance design](../../docs/obstacle_avoidance.md), [range-capture engine](../../docs/range_capture.md), [three-sensor scanner](../../docs/range_scanner.md) and [range-to-navigation adapter](../../docs/range_scanner.md).

## 6. Watchdog supervision

WDOG0 is configured from the 80 MHz system clock with a 50 ms first timeout and reset enabled on the second timeout if the first interrupt flag is not cleared. The foreground calls `watchdog_service()` after setup stages, completed calibration samples and control-loop paths; a stalled wait, IMU transaction or control path therefore stops servicing it. `WatchdogStallEnable()` pauses WDOG0 while the debugger halts the CPU. This is source-level supervision only: the target has not been compiled or tested, and WDOG0 does not protect against failure of the system clock. WDOG1 (PIOSC clock) or an external supervisor remains future work. See [watchdog reset and motor-output containment](../../docs/watchdog_safety.md).

## 7. Important limits

- Timer1A elapsed-time measurement improves over a constant 5 ms assumption, but no hardware measurements or worst-case execution timing exist. Timer0A flag coalescing, CPU hangs and counter overflow/aliasing after ~53.7 s remain limitations.
- I2C transaction handling has a software-bounded busy poll; timeouts, bus recovery, transaction sequencing and maximum duration still require real-target tests.
- No wheel encoders, independent PIOSC-clocked watchdog (WDOG1), UART logger, hardware emergency switch design, current monitoring, real sensor calibration report or power-stage approval exist. WDOG0 is integrated in source but its target behavior remains unverified.
- The main loop calls the portable navigation policy and routes its pitch/yaw requests through the controller and differential mixer, but autonomy is hard-disabled and all range inputs remain invalid until a real sensor backend and operator enable are implemented.
- Current MATLAB LQR is an illustrative comparison, not deployed code. Control gains are placeholders.
- Software output gating does not substitute for power isolation and mechanical restraint.

Next evidence: capture CCS build log, scope traces (PB6/PB7/PA5), static IMU readings and measured loop timing; then proceed through [verification_plan.md](../../docs/verification_plan.md) and [calibration / timing diagrams](../../docs/calibration_timing.md).
