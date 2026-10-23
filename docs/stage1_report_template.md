# NANENG 512 — Stage 1 Report (Draft Template)

**Project:** Tiva C Self-Balancing Robot

**Team:** [names / IDs]

**Advisor / section:** [fill in]

**Submission:** November 4, 2026

## 1. Problem statement
A two-wheel self-balancing robot is an unstable inverted-pendulum system. The system will estimate pitch using an accelerometer and gyroscope and use feedback to command wheel rotation.

## 2. Background and comparative survey
- [ ] Survey at least 3 credible inverted-pendulum robot references with full bibliographic details.
- [ ] Compare control algorithms (PID/PD, state feedback, LQR) and sensor fusion methods.
- [ ] Explain the embedded-platform change from Arduino to TM4C123, subject to written instructor approval.

## 3. System block diagram
See root `README.md` Mermaid diagram. Export as figure for final report.

## 4. Hardware architecture and schematic
See `hardware/wiring.md` pin mapping. **Attach real schematic after it has been drawn and verified**. Include power, motor driver, pull-ups, motor stall currents, battery and safety switch.

## 5. Bill of materials
See `hardware/bom.csv`. Get supplier prices with the date, shipping/tax assumptions, and final aggregate cost. TBDs are not acceptable in submitted report.

## 6. Mechanical design
Provide measured chassis dimensions, center of mass, wheel size, motor brackets and CAD exports. Label CAD pending until a real model is created.

## 7. Work plan / milestones
Stage 1: 4 Nov 2026 — architecture, schematic, BOM, Stage 1 report.
Stage 2: 6 Dec 2026 — build, system model, MATLAB, Stage 2 report.
Stage 3: 27 Dec 2026 — MATLAB digital controller, approved MCU implementation, final report and presentation.

## 8. References
Use manufacturer's datasheets, vendor quotations and cited research publications; cite each specific assertion.
