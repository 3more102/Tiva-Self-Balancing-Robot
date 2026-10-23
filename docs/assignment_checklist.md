# NANENG 512 — official deliverables and evidence register

**Source of requirements:** the user-provided two-page file *Project NANENG 512 (Self Balancing Robot).pdf*. Check the official document for submission formatting and any subsequent instructor updates. The assignment specifies Arduino for Stage 3, so using Tiva C **needs explicit approval**.

## Stage 1 — due 4 November 2026 (Week 7)

- [ ] Introduction and **proper literature survey** — descriptive draft exists in [Stage 1 report](reports/STAGE1_architecture_report.md), but comparative academic literature remains incomplete.
- [x] Functional **block diagram** — [architecture diagrams](architecture.md) contain draft diagrams.
- [ ] Reviewed **schematic diagram** — [wiring](../hardware/wiring.md) is a pin map, not a schematic.
- [ ] Finalized components with **confirmed supplier prices** — [BOM](../hardware/bom.csv) exists with costs TBD.
- [ ] CAD designs for any printed parts (if needed) — no CAD geometry delivered.
- [ ] Final **Stage 1 report** — [substantial draft](reports/STAGE1_architecture_report.md), **not submission ready**.

## Stage 2 — due 6 December 2026 (Week 12)

- [ ] Complete hardware assembly — not built or reported.
- [ ] Block diagram and mathematical model / system identification — [theory](control_theory.md) and [MATLAB script](../matlab/model_and_design.m) are **illustrative**, no identified plant.
- [ ] MATLAB analysis using measured plant parameters — pending.
- [ ] [Stage 2 report](reports/STAGE2_modeling_report.md) — methodology and report tables drafted, experimental results blank.

## Stage 3 — due 27 December 2026 (Week 15)

- [ ] Design digital controller using MATLAB for measured plant — existing LQR demo is not an implemented digital controller.
- [ ] Obtain written approval to use Tiva **instead of assignment-required Arduino**.
- [ ] Compile, run and verify Tiva C embedded controller — firmware source exists but is not target-tested.
- [ ] Supply all MCU source code, startup/linker configuration and reproduction instructions — draft source exists; validated CCS project missing.
- [ ] [Final report](reports/STAGE3_final_report.md) and presentation — framework only, no experimental data.

## Cross-cutting implementation

- [x] Portable C complementary estimator, PID and guarded arming/fault state logic
- [x] Host C tests wired into GitHub Actions
- [x] Editable control, hardware/power and state diagrams
- [x] Detailed draft reports and controls derivation
- [ ] Physical protection, final schematic and current-limited driver sizing
- [ ] Real board build, calibration, logging and repeated balance trials

**Do not convert drafts or green host tests into claims of completed experimental milestones.**
