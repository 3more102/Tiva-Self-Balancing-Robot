# NANENG 512 — Stage 1 Technical Report
## Architecture, Literature Basis, Circuit Planning, Component Selection and Work Plan

**Document version:** 0.2 (engineering draft, 23 October 2026)  
**Team / student IDs:** *To be completed by team*  
**Course:** NANENG 512  
**Project:** Self-Balancing Robot using the TI Tiva C TM4C123GH6PM  
**Assignment deadline:** 4 November 2026 (Week 7)  
**Submission status:** **INCOMPLETE — not ready for submission**

> The official assignment specifies an Arduino controller for Stage 3. This project proposes the Tiva C alternative **subject to instructor authorization**. The existence of source code is not evidence that the assembled robot works.

## Abstract

This project investigates the modeling, digital control and physical implementation of a two-wheeled self-balancing robot using a microcontroller, inertial sensing and two independently driven geared motors. The objective is to estimate the robot's inclination relative to the vertical and command motor rotation to counteract angular deviations from its unstable upright equilibrium. The preliminary architecture uses a TI TM4C123GH6PM ARM Cortex-M4F microcontroller, an MPU6050 inertial measurement unit and a TB6612FNG dual brushed-DC motor driver. A complementary estimator and sampled PID/PD algorithm are selected for an understandable embedded baseline; MATLAB is used for preliminary modeling and digital-control studies. The planned sample rate of 200 Hz, PWM of 20 kHz and initial controller gains are **design hypotheses**, not proven performance. This report documents the architecture and expected engineering work, explicitly identifying incomplete schematic, quotations and mechanical CAD.

## 1. Problem definition and objectives

The body of a two-wheeled robot is an inverted pendulum: the upright state is locally unstable unless the wheels apply correcting forces. The controller must sense pitch accurately, compute a response with sufficient bandwidth, and apply it through a motor drive that respects current, thermal and battery limitations. Sensor errors, communication loss, an incorrect sign convention and interrupted sampling may cause unexpected motion.

### Specific engineering objectives

1. Build a mechanical system in which the mass center is above the axle and the sensor axis orientation is documented.
2. Develop a traceable electrical design, including regulator/battery isolation, common ground, fuse and physical power cutoff.
3. Estimate pitch from two accelerometer channels and one gyroscope channel; quantify gyro bias and sensor noise from recorded data.
4. Design a sampled controller from MATLAB analysis, then implement it in Tiva C source code after instructor approval.
5. Validate a safe arming/trip strategy independently of PID accuracy; instrument response time and error conditions.
6. Demonstrate repeated balancing and report measured error, settling and recovery with valid experimental methods.

The measurable numerical *acceptance limits* beyond safety gates will be chosen after hardware characterization; no balancing-quality claims are made at this stage.

## 2. Technical background and engineering basis

The single-link inverted-pendulum approximation captures why pitch about the wheel axle is unstable: gravitational torque increases small departures from upright. Real robots add wheel translation, motor inductance, gearbox friction, contact dynamics, delays and battery variation. Accordingly, the **pitch-only linear state-space model is an educational first approximation**, not yet an identified full robot model. See [Control Theory](../control_theory.md) and the preliminary [MATLAB model](../../matlab/model_and_design.m).

The MPU6050 integrates accelerometers and gyroscopes. The documented WHO_AM_I register is `0x75` and its default read value is `0x68`. The proposal uses accelerometer Y/Z and gyro X in a mounting arrangement whose rotation axis is X. A complementary filter integrates high-frequency gyro rotation while limiting drift using a low-frequency accelerometer angle. The actual filter time constant requires experimental tuning to vibration and acceleration.

The TI TM4C123GH6PM LaunchPad provides hardware I²C/PWM, an 80 MHz Cortex-M4F, debug interface and onboard switch/LEDs. Its capability motivates using Tiva C instead of Arduino, although coursework authorization is a separate issue. The Toshiba TB6612FNG provides two H-bridge channels with standby and PWM input. The **motor stall current** must be established before deciding if the IC and particular breakout are sufficient.

### Source basis

| Reference | What it supports | What it does **not** prove |
|---|---|---|
| [TI EK-TM4C123GXL product page](https://www.ti.com/tool/EK-TM4C123GXL) | MCU, on-board peripherals and software resources | This specific firmware's timing |
| [TI LaunchPad user manual](https://www.ti.com/lit/ug/spmu296/spmu296.pdf) | Board pin assignments, switches and connections | Correct assembly of purchased hardware |
| [TDK MPU6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf) | Register locations, identity and scaling configuration | Sensor noise/accuracy on this robot |
| [Toshiba TB6612FNG product/datasheet](https://toshiba.semicon-storage.com/eu/semiconductor/product/motor-driver-ics/brushed-dc-motor-driver-ics/detail.TB6612FNG.html) | Driver pins and operating limits | Sufficiency for unknown motor stall current |
| [Control derivation in repository](../control_theory.md) | Clearly labeled preliminary model and controller equations | Experimental validation |

**Before submission:** add at least three reviewed academic references relevant to self-balancing robotics and critically compare estimator and controller methods; do not label manufacturer specifications as a completed literature review.

## 3. Proposed system architecture

### 3.1 Components and responsibilities

| Subsystem | Proposed part | Role | Design dependency |
|---|---|---|---|
| Main processor | TM4C123GH6PM on EK-TM4C123GXL | I²C sampling, estimator, control, PWM, safety | CCS/TivaWare build + instructor acceptance |
| IMU | MPU6050 breakout | Angular rate and acceleration | 3.3 V-safe breakout and documented axes |
| Motor driver | TB6612FNG module | Drive two geared DC motors | Motor stall current and battery voltage |
| Actuators | 2 brushed geared motors | Differential wheel torque | Speed/torque/current selection |
| Structure | Axle, chassis, mounts, wheels | Physical pendulum and traction | CAD, COM and inertia |
| Supply | Battery, protection, regulator | Motors and MCU logic | Fuse, E-stop, safe charging |
| User controls | PF4 SW1, RGB LED | Hold-to-enable and status | Timing/debounce validation |

See [Block diagrams](../architecture.md) for **five detailed diagram sections**, covering functional flow, estimator/control pipeline, power architecture, finite-state arming logic and pin mapping.

### 3.2 Draft circuit and wiring

| Board signal | Connection |
|---|---|
| PB2/PB3 | MPU6050 SCL/SDA (I2C0; ensure correct pull-ups) |
| PB6/PB7 | TB6612 PWMA/PWMB |
| PC4..PC7 | TB6612 AIN1/AIN2/BIN1/BIN2 |
| PA5 | TB6612 STBY |
| PF4 | Onboard SW1, active-low |
| MCU logic, sensor, driver logic | Compatible 3.3 V domain / common reference |
| Dedicated motor supply | Battery -> rated fuse -> physical disconnect -> driver VM |

**Electrical deliverable still missing:** a genuine schematic drawn in a CAD tool, including bulk/ceramic decoupling, connectors, power-protection symbols and validated net names. This wiring mapping is not a substitute for the required final schematic.

### 3.3 Mathematical and software architecture

The conceptual pitch model is

```text
J*theta_ddot = m*g*l*sin(theta) - b*theta_dot - Ku*u
```

Linearizing around upright yields `A=[0 1; m*g*l/J -b/J]` and `B=[0; -Ku/J]`. Here `u` is an assumed effective pitch-equivalent torque input. The MATLAB script discretizes this at `T_s=0.005 s` using a zero-order hold. All included plant parameters are provisional and **must be identified**.

On MCU, gyro bias estimation is attempted at power-on while stationary; a complementary filter estimates pitch; a guarded controller computes command; a safety state machine requires observed switch release and upright hold before actuation. If sensor data become invalid, software commands driver standby LOW. The actual motor output is globally compile-time disabled pending physical verification. See [firmware source](../../firmware/tiva/src/main.c).

## 4. Mechanical design constraints

- Chassis height, width, mass, wheel spacing/radius, effective pitch moment of inertia, COM position and battery placement: **not yet measured**.
- Proposed construction: vertically stacked two-wheel frame, stiff axle support, low backlash gearbox, restrained sensor mount and protected battery.
- CAD deliverables needed: assembly, exploded view, mass-property estimate, motor brackets, sensor bracket, wiring path and STL/STEP exports as appropriate.
- Analysis: ensure available wheel torque and chassis geometry are compatible. Do not select motors solely from no-load RPM.

## 5. Preliminary bill of materials and cost estimation

The editable [BOM CSV](../../hardware/bom.csv) lists all preliminary categories. Current vendor prices are **TBD**; the report intentionally does not invent prices. Before submission, enter itemized unit/extended cost in EGP, supplier URL, quote date, shipping/taxes, quantities and a total. Attach dated screenshots or invoices from reputable vendors.

## 6. Safety, risks and mitigations

| Risk | Possible consequence | Mitigation and validation |
|---|---|---|
| Incorrect control polarity | Robot accelerates away from upright | Test each wheel lifted, record signs, keep motor output disabled until confirmed |
| Loss of sensor/bus | Unobserved tilt, runaway | Sensor failure causes standby low; later add watchdog and physical fault timing test |
| Excess motor current | Driver thermal stress, wiring/battery heating | Measure stall current, fuse supply, verify safe motor/driver sizing |
| MCU reset / brownout | Unexpected output or loss of control | Independent supply regulation; measure and ensure STBY pull-down hardware |
| IMU axis mismatch | Wrong pitch estimate | Axis fixture and static tilt test |
| Mechanical tip-over | Injury, damaged electronics | Tether/guard and physical cutoff during first actuation |
| Incorrect project scope | Assignment noncompliance | Obtain written approval for Tiva replacing Arduino |
| Incomplete BOM/schematic/CAD | Stage 1 submission incomplete | Assigned owners and review checklist |

## 7. Project execution schedule

| Date | Required evidence | Status as of report version |
|---|---|---|
| 4 Nov 2026 — Stage 1 | Survey, block and circuit schematic, BOM with prices, optional CAD, Stage 1 report | Documentation draft; physical/cost items outstanding |
| 6 Dec 2026 — Stage 2 | Full assembly, mathematical model/identification, MATLAB analysis, Stage 2 report | Not complete |
| 27 Dec 2026 — Stage 3 | MATLAB digital controller, approved MCU code, full report and presentation | Not complete |

## 8. Conclusions and pending evidence

The proposed design is coherent at a conceptual level and has an initial portable controller implementation. This draft is **not a complete Stage 1 submission** until source-backed survey, reviewed schematic, priced BOM, team/teacher details and applicable CAD are attached. All observed hardware performance, financial totals and demonstrated balancing remain unreported because no supporting measurements are available.

## Appendix — Submission and reviewer check

- [ ] Five team members and IDs, course details, instructor, revision control
- [ ] Academic references with publication metadata and method comparison
- [x] Functional and feedback diagrams
- [ ] Full net-connected schematic verified against actual breakout pinouts
- [ ] Dated EGP quotations and BOM sum
- [ ] Mechanical CAD/assembly drawings if 3D parts are needed
- [ ] Instructor acknowledgment of Tiva MCU substitution
- [ ] Format report according to university submission rules
