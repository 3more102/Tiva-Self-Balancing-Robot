# Experimental Protocol and Data Provenance

**All tables here are templates.** No balancing runs, measured controller gains or robot characteristics have been reported.

## A. Data integrity

There are two distinct datasets:

1. **THEORETICAL_SIMULATION_NOT_MEASURED** from `simulation/run_pitch_sim.py`. It is idealized, lacks motor/wheel dynamics and carries its own `data_origin` label.
2. **MEASURED_TIVA_ROBOT** from a *future* calibrated, instrumented Tiva build. No such CSV is currently in the repository. `matlab/plot_telemetry.m` intentionally rejects any different or missing provenance.

The future telemetry logger should save a traceable acquisition date, firmware SHA, sampling clock, sensor full-scale settings and axis orientation. For meaningful replay include at least these CSV columns:

| Column | Unit | Evidence expectation |
|---|---|---|
| time_s | s | MCU timestamp converted with documented clock |
| angle_deg | degrees | Filter estimate, optionally independent angle reference |
| gyro_x_dps | degrees/s | Measured raw/compensated selection documented |
| command | normalized [-1,1] | signed actuator command before mapping |
| data_origin | enum | `MEASURED_TIVA_ROBOT` only for actual hardware |

Recommended extras: raw accel/gyro channels, battery voltage, motor current, wheel encoder counts, actual timer ticks, loop execution time, safety state and fault code.

## B. Controlled test matrix

| Scenario ID | Test | Motor state | Gate before proceeding |
|---|---|---|---|
| S00 | Electrical inspection, fused supply, disconnected motor VM | OFF | Safety sign-off |
| S01 | IMU WHO_AM_I and noise sampling while stationary | OFF | Actual identity and sample noise verified |
| S02 | ± known mechanical pitch angle jig | OFF | Gyro/accel axis sign and orientation verified |
| S03 | Rapid bus disconnect, MCU reset and missed update | OFF | Driver standby verified and documented |
| S04 | Low-duty wheel polarity each side, wheels up | Restrained, current limited | Correct signed directions and no excessive current |
| S05 | Restrained closed-loop recovery from gentle pitch | Restrained | Verified control sign, no runaway |
| S06 | Tethered/guarded floor trials | Limited/restrained | Measured response and physical cutoff |
| S07 | Repeatable balancing trials | Controlled | Results logged with predeclared bounds |

### Minimum reporting for each run

Document test/operator/date, selected motors, battery state, mechanical geometry, commit hash, compile configuration, safety devices, number of trials, success count, data filenames and observed failure modes. Calculate RMS angle, peak absolute angle, saturation fraction and settling time with declared thresholds. Avoid editing raw data to remove failures.

## C. Engineering safety

**Safety protocol is not a certification.** Keep a physical power disconnect, fuse and restraint; confirm driver continuous current ratings against the *selected* motor. Do not energize the motor driver until schematic, rails and motor polarity are independently checked. The onboard SW1 input and MCU logic are not the physical kill switch.

## D. Acceptance criteria and scientific claims

- A green portable C test proves only that the particular synthetic host test vectors passed.
- A clean MATLAB plot proves only that the script processed input data; it does not prove their origin.
- The virtual PD plant can stabilize in simulation while being completely incorrect for the real robot.
- Balancing accuracy/performance claims require real logs, real initial conditions and repeatability statistics.
- Stage 1 requires a net-connected electrical schematic, supplier-priced BOM, proper literature survey, and CAD if needed. Draft diagrams are valuable but not a substitute for these deliverables.
