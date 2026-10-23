# Official NANENG 512 deliverables checklist

Reference: user-provided **Project NANENG 512 (Self Balancing Robot).pdf** (2 pages). These are **requirements, not progress claims**.

## Stage 1 — 4 Nov 2026 (Week 7)
- [ ] Introduction and literature survey
- [x] Function block diagram (draft, in README)
- [ ] Final schematic diagram, with explanation of each part (**wiring table is NOT a finished schematic**)
- [ ] All components and **current, source-backed prices** (BOM file has TBD values)
- [ ] 3D CAD design, if printed parts are required
- [ ] Stage 1 report

## Stage 2 — 6 Dec 2026 (Week 12)
- [ ] Assemble complete hardware
- [ ] Block diagram and mathematical model / system identification
- [ ] MATLAB analysis of identified system
- [ ] Stage 2 report

## Stage 3 — 27 Dec 2026 (Week 15)
- [ ] Design of the digital controller using MATLAB
- [ ] **Obtain permission to substitute TI Tiva for required Arduino**
- [ ] Verified digital-controller implementation on Tiva C
- [ ] Complete microcontroller codes
- [ ] Final report and presentation
- [ ] Submit MATLAB files and documented calculations/simulations

## Current coding work
- [x] Unverified TivaWare startup application and sensor/driver routines drafted
- [x] Pure C complementary filter + PID with host unit tests
- [x] MATLAB initial illustrative analysis script
- [ ] Compile on embedded toolchain, run on board, measure latency, verify sensor and wheels
- [ ] Obtain full experimental data and update MATLAB with measured parameters
