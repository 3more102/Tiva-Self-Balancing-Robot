# Verification and acceptance test plan

**Scope:** Requirements and reproducibility plan. As of 23 Oct 2026 **no actual Tiva hardware acceptance test has been reported**. Do not check off hardware tests based on the host-only C test suite.

## Verification hierarchy

| Level | Objective | Evidence available |
|---|---|---|
| L0 — source and mathematical integrity | Inspect assumptions, API use, units, docs and continuous/discrete equations | Docs and reviews; MATLAB model illustrative |
| L1 — portable host tests | Exercise estimator/PID/arming, gyro calibration, timer wrap, invalid samples and saturation | GitHub Actions GCC + UBSan; separate theoretical simulator tests |
| L2 — target build / emulator | Compile complete TM4C123 TivaWare image, verify startup and link | **Missing** |
| L3 — instrumented benchtop | Verify power/current/logic/timing without balancing | **Missing** |
| L4 — restrained closed-loop | Controlled low-energy motor/polarity and pitch tests | **Missing** |
| L5 — balancing evaluation | Repeated quantified response on the built robot | **Missing** |

## Test matrix

| ID | Test & procedure | Acceptance before proceeding | Status |
|---|---|---|---|
| C-01 | Build host C with strict GCC warnings | Compile without errors | Automated; inspect branch CI |
| C-02 | Exercise complementary estimator with zero tilt, gyro integration and known angle | Unit assertions pass | Automated; inspect branch CI |
| C-03 | Invalid samples, zero acceleration and dt > 20 ms | Returns NAN; motor application intended to shutdown | Automated algorithm tests only |
| C-04 | PID saturation and anti-windup for repeated extreme inputs | Output bounded within ±0.7, integral bounded | Automated |
| C-05 | Safety startup while held | Must stay DISARMED without prior release | Automated |
| C-06 | Release, 40 upright held samples, tilt trip, release/rearm | Transitions exactly as specified | Automated |
| C-07 | Gyro bias calibration: switch, motion, 200 valid samples and noise bounds | No completion for unstable/noisy samples | Automated host C tests; hardware pending |
| C-08 | Timer1 unsigned downcounter wrap, 5 ms acceptance, timing outliers | Accept good clock ticks, reject abnormal interval | Automated host C tests; target timing unverified |
| S-01 | Theoretical pitch-only PD / open-loop regression | Internally consistent numerical results | Automated Python, **not measured robot data** |
| D-01 | Check relative Markdown links and code fences | No broken links, balanced fences | Automated |
| T-01 | Compile all embedded source with CCS for TM4C123 | No target compiler/link errors | **Not tested** |
| T-02 | Inspect GPIO levels / power without motors connected | PA5 LOW, safe pin voltages | **Not tested** |
| T-03 | MPU WHO_AM_I via I2C0 | 0x68 at address 0x68, clean transaction | **Not tested** |
| T-04 | Upright and known ± tilt jig samples | Correct angle/gyro polarity and plausible magnitude | **Not tested** |
| T-05 | Actual timing scope/DWT counter | Nominal period ~5 ms, bounded WCET, acceptable jitter | **Not tested** |
| T-06 | PWM waveform on PB6/PB7 | Nominal 20 kHz and correct duty mapping with STBY low | **Not tested** |
| T-07 | Physical switch/debounce and safe arming | No surprise motor activation, including reset while held | **Not tested** |
| T-08 | Sensor unplug, transient NACK, CPU fault/brownout | Measured safe motor cutoff; independent hardware containment | **Not tested** |
| T-09 | Motor rotation/polarity with suspended wheels, qualified supply | Each channel matches signed command, no over-current | **Not tested** |
| R-01 | Restrained moderate tilt recovery | No positive feedback runaway; documented response | **Not tested** |
| R-02 | Repeated balance runs | Actual logs/metrics linked to measured datasets | **Not tested** |

## Operator safety procedure

1. Confirm mechanical restraint, physical motor-battery disconnect, fuse, safe wiring and suitable driver rating.
2. Keep the motor driver supply isolated when first flashing and testing sensor, switch, timers and LEDs.
3. Measure GPIO with a scope/meter **before** connecting the driver supply. Verify STBY low at startup and reset.
4. Test motor direction individually at very low duty with wheels raised; record axis/sign truth table.
5. Select conservative controller gains based on a measured model. Do not rely on constants shown as initial placeholders.
6. Only proceed to restrained balancing after hardware cutoff works independently of the MCU software.
7. Report all failures, model assumptions, date, board revision, compiler version, test operator and commit SHA.

## Performance and analysis evidence checklist

- [ ] Qualified electrical schematic and as-built photos
- [ ] Current limiting, supply brownout and stall current verification
- [ ] Reproducible CCS build (compiler/TivaWare version, startup and linker files)
- [ ] Real waveform captures for MCU GPIO, I2C, PWM and STBY
- [ ] Raw timing, safety and sensor captures (software calculations are not measured timing profiles)
- [ ] Recorded measured plant parameters
- [ ] Control gain derivation and validation on real robot
- [ ] MATLAB analysis with meaningful comparison plots
- [ ] Quantitative pitch error, settling, overload/current and repeatability tables
- [ ] Independent reviewer sign-off
