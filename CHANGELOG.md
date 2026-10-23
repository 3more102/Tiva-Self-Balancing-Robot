# Changelog

## 0.2.0 — 2026-10-23 — Engineering documentation and safety review

- Improved startup safety: new release-to-arm state machine with a 40-sample upright qualification period, 25-degree trip and safe reset/fault handling; integration in Tiva main loop.
- Estimator and controller reject non-finite data and invalid update periods; regression suite expanded for arming, faults, angular limits, dropout inputs and PID anti-windup.
- Added multi-level, GitHub-rendered Mermaid system, control-flow, power, wiring and safety state diagrams.
- Added full control model derivation and clear MATLAB versus embedded PID/LQR distinctions.
- Added in-depth Stage 1, Stage 2 and Stage 3 academic drafts, component rating/budget worksheet and prioritized optimization/engineering review log.
- Overhauled README with safety limitations, quickstart, report navigation, build instructions, design overview and evidence-based project status.
- **Limitations:** host-side tests are not a Tiva firmware build; no physical balancing data or completed Stage 1 schematic/BOM/CAD exists.

## 0.1.0 — 2026-10-23

- Initial Tiva C TM4C123 project structure and provisional hardware wiring plan.
- Portable complementary estimator and guarded PID with host-side tests.
- Draft TivaWare MPU6050 I2C / TB6612 PWM firmware; motor output disabled by default.
- MATLAB illustrative pitch-only dynamics and LQR exploration.
- NANENG 512 deadlines and first engineering checklist.
