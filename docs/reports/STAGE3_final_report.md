# NANENG 512 — Stage 3 Final Report Framework
## Digital Controller Design, Tiva Firmware Integration, Experimental Results, Presentation

**Deadline:** 27 December 2026 — Week 15  
**Current status:** report structure and verification criteria, not a completed final report.  
**Critical deviation:** official document requires Arduino — **obtain written approval** for Tiva implementation.

## Executive summary (fill after validation)

*Required final result: state whether the assembled Tiva robot balanced without tether, how many trials were performed, which controller was deployed, its sample interval and measured errors, its highest observed current and whether safety tests passed.* **No experimental outcomes are available at this time.**

## 1. Research question and design decisions

Can a two-wheel robot using IMU pitch estimation, a sampled digital feedback controller and TB6612FNG motor drive remain upright over a defined initial-tilt and disturbance envelope? Why choose the design controller over alternatives? State test condition boundaries and physically verify the direction mapping.

## 2. Final hardware and software configuration

Identify final board revision, MCU clock, sensor configuration, bus speed, motor/gear ratings, wheel radius, battery type and capacity, fuse cutoff, physical kill switch, driver thermal/power ratings, measured sample jitter and code revision SHA. Provide a final net-connected schematic and photographs, replacing initial block-only illustrations.

## 3. Digital controller synthesis

Document continuous model, identified parameters, discrete sample time `Ts`, `Ad/Bd` derivation, stability margins, controller equations, design criteria and MATLAB charts. If using PID, explain the actual deployed gains, units and the target error direction. If using LQR, implement LQR consistently on MCU and state how the needed states are estimated. Do not present the existing MATLAB LQR **as the firmware PID**.

## 4. Embedded implementation and scheduling

Present diagram from [architecture.md](../architecture.md). Document SPI/I2C choices, initialization, gyro calibration, periodic sampling, PWM update, task execution time, error handling and memory/CPU budget. Test all embedded code with CCS/TivaWare using a supported startup file and correct linker configuration. Keep motor outputs disabled by default in repository release unless board-specific safety validation justifies changing that flag in a controlled lab build.

## 5. Required controls and independent safety evidence

- Release-to-arm SW1 switch, upright confirmation and angle trip, witnessed test logs.
- Correct motor polarity and left/right PWM mapping under restrained motion.
- Sensor bus disconnection, corrupt sample handling, MCU reset and missed control deadline.
- Hardware motor cutoff, fuse and appropriately rated motor driver; show schematic and test procedure.
- Comparison of predicted controller output and measured output for selected test vectors.

**Important:** current software alone provides no independent watchdog or fail-safe hardware cutoff beyond holding driver STBY low while running. A physical cutoff is still necessary.

## 6. Experiment design and plots

Test several controlled initial angles, multiple battery charge levels and documented disturbances. Include timestamps and at least three repeated runs per selected condition if feasible; explain exclusions and any stopped unsafe run. Plots should show measured pitch, reference pitch, gyro rate, signed command, battery and current where available. Compare simulation and real traces distinctly, never replacing failed or absent measurements with model output.

### Unfilled results register

| Test ID | Initial tilt | Recovery success | RMS pitch | Peak pitch | Saturation fraction | Worst execution time | File |
|---|---|---|---|---|---|---|---|
| Pending | — | Not tested | — | — | — | — | — |

## 7. Critical analysis

Interpret failures (sensor motion artifacts, motor deadzone, battery droop, time jitter, control polarity, gearbox friction) with evidence. Compare competing controllers **under the same mechanical setup**. Quantify uncertainty and limitations.

## 8. Conclusions, learning outcomes and future work

Summarize only measured outcomes. Potential upgrades: wheel encoders, cascaded speed/pitch control, tested independent watchdog, current sensing, robust scheduling, parameter fitting and extended full-state models. State all unresolved safety and scope deviations.

## 9. Deliverables and presentation skeleton

- [ ] Approved MCU scope change (Arduino → Tiva)
- [ ] MATLAB controller-design files and documented calculations
- [ ] MCU firmware and TivaWare/CCS reproducible build evidence
- [ ] Full final hardware photos/schematic/CAD/BOM
- [ ] Repeatable time-stamped experimental data
- [ ] Results, statistical analysis and references
- [ ] Final report reviewed and signed
- [ ] Presentation: problem → scope → architecture → model → controller → code → experimental results → safety/limitations → conclusions

**Primary sources:** [TI platform](https://www.ti.com/tool/EK-TM4C123GXL), [TDK IMU document](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf), [Toshiba motor driver](https://toshiba.semicon-storage.com/eu/semiconductor/product/motor-driver-ics/brushed-dc-motor-driver-ics/detail.TB6612FNG.html), plus full academic references to be added by the team.
