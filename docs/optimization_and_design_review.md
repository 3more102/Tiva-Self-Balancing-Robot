# Engineering optimizations, design review and prioritized next work

**Purpose:** distinguish actual source-code changes from proposed improvements. Performance percentages, sensor sample timings, CPU use and physical balance quality are **unknown** until measured on the TM4C123 board.

## Implemented and covered by portable C tests

| Area | Change | Source / evidence | Limitation |
|---|---|---|---|
| Angle estimation | Reject non-finite samples, excessive `dt`, invalid YZ acceleration projection | `controller.c`, `tests/test_controller.c`, GitHub Actions | Actual IMU failure modes not bench tested |
| Digital controller | Conditional integration under saturation and normalized command cap | `controller.c`, regression tests | No verified motor torque mapping |
| Safe output gating | Startup DISARMED, release-to-arm requirement, 40 upright samples, angle trip, FAULT/release reset | `controller.c`, `main.c`, unit tests | SW1 debounce and input electrical fault analysis missing |
| Firmware operator control | Motor outputs compile-time disabled; STBY LOW when disarmed | `main.c` and `motor_backend.h` | Not verified on oscilloscope or target board |
| Configuration traceability | Documented axes/pins, motor polarity assumptions and default parameters | `docs/architecture.md`, wiring file | Unknown purchased part variants |
| Analysis traceability | PID in firmware explicitly separated from illustrative LQR in MATLAB | `docs/control_theory.md`, MATLAB script | MATLAB only illustrative |

These changes are mostly **safety/reliability improvements**, not benchmarked CPU optimizations. Descriptions such as “optimized balancing,” “production ready,” or “zero hardware defects” would be unsupported.

## Next actions in recommended priority order

### P0 — hardware correctness and safety (before energizing)

1. Obtain instructor approval for using Tiva C instead of the Arduino required by the official PDF.
2. Freeze exact part numbers and vendor datasheets for the motor, driver breakout, MPU breakout, battery, charger and regulator.
3. Draw and independently review a full schematic; check motor stall currents against the TB6612FNG **operating** rating, not just absolute maximum/brief peak.
4. Add hardware pull-down to STBY (per correct driver-interface design), fused motor power, accessible mechanical cutoff and secure battery mounting.
5. Compile the MCU target using the actual CCS/TivaWare version. Check pin mux, I²C completion sequence, expected MPU WHO_AM_I and all GPIO voltage rails.
6. Keep wheels suspended and motor supply isolated while confirming LED/status behavior, switch logic, I²C integrity, PWM output frequency and both motor directions.

### P1 — time determinism and fault containment

7. Replace nominal `Ts=0.005` in estimator with a **measured, validated per-loop interval** from a high-resolution timer; detect missed deadlines, record worst-case execution time and jitter. The current source only rejects invalid dt if supplied; `main.c` still passes nominal dt.
8. Add I²C transaction timeout measurements and errors per type; verify safe recovery on NACK, disconnect and stuck clock/data.
9. Add a true independent watchdog / brownout strategy. The existing software gate cannot recover if the CPU stops executing.
10. Add switch debounce and a dedicated sensor calibration state with stillness and angle validation.
11. Add sampled telemetry (timestamp, angle, gyro, command, safety state, battery/current), using DMA/ring buffers or low-priority output to avoid blocking 5 ms control loop.
12. Set hardware-validated limits for velocity, torque/current and repeated re-arming after faults.

### P2 — stability and system identification

13. Measure wheel geometry, masses/COM, inertia, motor winding resistance, gearbox ratio, friction, battery sag and IMU errors with logged uncertainty.
14. Extend mathematical plant from pitch-only to wheel position/velocity and pitch/rate with motor dynamics, saturation and deadzone.
15. Use train/test split with real logging; compare predicted and measured responses, and cross-validate parameter fitting.
16. Design/tune digital PID, LQR or cascading position/pitch controller **against measured plant**, then verify robustness and reference stability.
17. Measure gain margin/phase margin or equivalent sampled-data robustness and compare to alternatives at identical test conditions.

### P3 — reporting and release quality

18. Finish priced BOM in EGP with vendor/date, mechanical CAD and net-connected schematic; update Stage 1 draft.
19. Add real hardware integration tests and a reproducible CCS build in CI if a licensed/available toolchain permits.
20. Record videos, plots, success/failure criteria and documentation of repeated physical trials; complete Stage 2 and final report.
21. Prepare reproducible release with tested hardware build, wiring revision, calibrated configuration and known issues.

## Quantitative validation targets (proposed; not claims)

| Metric | How measured | What to report |
|---|---|---|
| Worst observed period and control execution time | instrument MCU timing/oscilloscope | max, median, jitter, missed counts |
| Pitch RMS error | logged angles against upright | deg RMS over stated interval |
| Recovery time after small perturbation | repeated tilt tests | seconds, N trials, spread |
| Current peak and battery sag | suitable current/supply instrumentation | amps and volts, thermal notes |
| Sensor error | fixed angle jig and repeated logging | bias, RMSE and drift |
| Controller saturation | duty command traces | percent samples at clamp |
| Fault response | simulated/disconnected sensor with safe rig | time to driver standby low |
| Test reproducibility | committed toolchain versions and data | pass/fail traceable commit and setup |

## High-risk design gaps

- PWM and I²C peripheral APIs need a CCS/TivaWare compile to establish correctness.
- The current motor code can only drive after manually changing a compile-time safety setting, and the physical polarity is unverified.
- No current sensor, wheel encoder, physical e-stop circuit design, schematic, CAD, hardware watchdog or production release has been validated.
- The existing MATLAB `model_and_design.m` has illustrative plant constants, not fitted robot values.
- The official assignment requires Arduino: the microcontroller change is unresolved until instructor confirms it.

## Review sign-off

| Review | Reviewer | Date | Evidence / issue |
|---|---|---|---|
| Schematic / supply | TBD | — | — |
| Pinout / firmware target build | TBD | — | — |
| Motor rating & mechanical risks | TBD | — | — |
| Controller assumptions and test design | TBD | — | — |
| Report scientific integrity | TBD | — | — |
