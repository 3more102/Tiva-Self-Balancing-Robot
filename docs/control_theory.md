# Control and estimation — engineering derivations

**Notation and status:** The linear plant is an **assumed pitch-only teaching model**, not an identified robot. No figures in this document are experimental. Discrete control gains are uncalibrated placeholders. Ensure the gyro axis is mechanically parallel to the wheel axle and the accelerometer frame is the one documented in [architecture.md](architecture.md).

## A. Physics and linearization

Let `theta` (rad) be body tilt about wheel axle measured from the unstable upright equilibrium; `omega = dtheta/dt`; `m` be effective body mass, `l` COM-to-axle distance, `J` effective moment of inertia, `b` pitch viscous damping, and `u` an effective normalized control command with pitch torque coefficient `Ku`.

A simplifying rigid inverted-pendulum approximation (wheel translation / full motor dynamics excluded) is

```text
J * theta_ddot = m*g*l*sin(theta) - b*theta_dot - Ku*u
```

Near upright, `sin(theta) ≈ theta`. With state `x=[theta; omega]`:

```text
x_dot = A*x + B*u
A = [0,             1
     m*g*l/J,      -b/J]

B = [0
    -Ku/J]
```

Notice that `Ku` and the motor polarity depend on motor orientation, wheel contact, gearbox and wiring. The full two-wheel robot also has forward position/velocity states and lateral dynamics; use of this pitch-only model is appropriate **only for preliminary analysis**.

## B. Zero-order-hold discretization

At sample interval `T_s = 1/200 = 0.005 s` (nominal):

```text
x[k+1] = Ad*x[k] + Bd*u[k]
Ad = expm(A*Ts)
Bd = integral(exp(A*t)*B, t=0..Ts)
```

The MATLAB `c2d(plant,Ts,'zoh')` call computes those matrices. The state-feedback demonstration uses discrete LQR weights `Q=diag([80,4])` and `R=4`, with `u=-K*x`. The numerical weights are **illustrative**, not a performance guarantee, and no LQR is deployed in Tiva firmware.

## C. Estimation

Given accelerometer `ay`, `az` measured in g along the proposed axes, and X-axis gyro `gx` in deg/s:

```text
theta_acc_deg = atan2(ay,az) * 180/pi
gx_corrected = gx - mean_stationary_gyro_bias
alpha = tau / (tau + Ts), tau = 0.5s (placeholder)
theta_hat[k] = alpha * (theta_hat[k-1] + Ts*gx_corrected)
             + (1-alpha) * theta_acc_deg
```

This complementary estimator trades gyro integration drift against accelerometer motion artifacts; it is **not** a Kalman filter. At `T_s=0.005` and `tau=0.5`, `alpha=0.5/0.505=0.990099...`. The first valid reading initializes angle from accelerometer. Startup X-gyro bias now requires 200 consecutive, stationary upright samples with button released, online mean/variance checks and a timeout (see [calibration and timing](calibration_timing.md)); those checks remain host-validated only. A yaw/roll-excited vehicle requires a more complete attitude estimator.

Current controller input validation rejects non-finite values, invalid sampling intervals (above 20 ms or non-positive) and degenerate/excessive YZ acceleration projection, and commands safe shutdown via the application when the estimator returns `NAN`. This is **software-level behavior**; real sensor self-tests remain to be performed.

## D. Controller and anti-windup

With target tilt `0 degrees` and error `e=-theta_hat`:

```text
I_trial[k] = clip(I[k-1]+e[k]*Ts, -20, 20)   [deg*s]
u_PD[k] = Kp*e[k] - Kd*gx_corrected[k]
u_request = u_PD + Ki*I_trial[k]
```

The portable PID accepts the proposed integral if unsaturated or if the error would relieve the clamp; else retains `I[k-1]`. Finally `u=clip(u_PD + Ki*I, -0.7, +0.7)`. Derivative-on-measurement helps avoid setpoint derivative kick. Code gains `Kp=0.016, Ki=0.0, Kd=0.0009` are placeholder normalized-output values and **must not be reported as tuned optimal gains**.

**Critical sign caveat:** `u`, `Ku`, and wheel motion must be reconciled experimentally. A sign error creates *positive* feedback and can cause immediate runaway. The firmware defaults to motor output disabled; testing must begin on a restrained robot.

## E. Control limits and assumptions

- Nominal control period is 5 ms and nominal PWM frequency is 20 kHz. Source now passes a Timer1-measured dt (and faults outside provisional 2.5–8 ms bounds), but actual MCU scheduling, jitter and I2C duration remain **unmeasured**.
- The current polling design assumes each I2C/algorithm update completes before the next control tick. Add measured execution time, overrun counters and a watchdog before qualification.
- MPU acceleration includes inertial acceleration, not gravity only. Fast acceleration distorts `atan2(ay,az)`; validation must include motion data.
- Motor voltage and effective torque vary with battery voltage, resistance, driver and mechanical friction. Saturation, dead-zone and backlash are currently absent from MATLAB model.
- Without wheel encoders, a pitch-only controller cannot directly regulate long-term vehicle speed or position. Future augmentation: `[position, velocity, pitch, pitch_rate]` and cascading loops or full state feedback.
- Any claimed settling time, overshoot, rise time, control energy, or RMS angle **must** cite the actual scenario, parameters and measured or simulated data source.

## F. Experiments needed before model validation

1. Measure `m, l, wheel radius, gearbox ratio`, and estimate `J` from CAD or physical experiment.
2. With motor drive disabled, acquire IMU stationary readings to estimate bias and noise versus temperature; document axis and mounting.
3. With robot physically restrained and a kill switch installed, measure driver duty vs motor speed/current under safe low-duty conditions.
4. Obtain wheel encoder data (if installed) to characterize deadzone, friction, time constant and motor saturation; note absence if not installed.
5. Fit dynamic parameters on one dataset; validate on **unseen** data and report uncertainty / residuals.
6. Use MATLAB to select a discrete controller, then implement exact sampled gains in embedded code and measure actual closed-loop response.

## G. Suggested quantitative report metrics

`RMS_pitch = sqrt(mean(theta_error.^2))`; steady-state absolute error; settling time into an explicitly declared band; maximum tilt; recovery from gentle standardized perturbations; duty saturation fraction; lost-sample rate; median/maximum loop execution time; battery voltage and observed motor current. Report repeated trials, units, uncertainty and whether results are simulation or measurement.

**Sources:** [TI MCU / board](https://www.ti.com/tool/EK-TM4C123GXL), [TDK MPU register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf), [Toshiba TB6612FNG](https://toshiba.semicon-storage.com/eu/semiconductor/product/motor-driver-ics/brushed-dc-motor-driver-ics/detail.TB6612FNG.html). Mathematical model is explicitly a first-principles simplifying assumption, not a claim made by the manufacturers.
