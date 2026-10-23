# Reproducible pitch-only theoretical simulation

This folder provides a standard-library Python simulation of a **simplified pitch-only inverted pendulum**, not a digital twin of the real two-wheel Tiva robot. It assumes a direct normalized pitch-correcting torque command, ignores wheels, motors, supply sag, sensors, sample jitter and travel dynamics, and uses demonstration parameters. These numbers are **not hardware measurements**.

Run from repository root:

```bash
python3 simulation/run_pitch_sim.py --angle 4 --output outputs/pitch_PD_THEORETICAL.csv
python3 simulation/run_pitch_sim.py --angle 4 --open-loop --output outputs/pitch_OPEN_LOOP_THEORETICAL.csv
python3 -m unittest discover -s tests -p "test_simulation.py"
```

The output CSV has the four fields expected by the MATLAB plotting script plus `data_origin=THEORETICAL_SIMULATION_NOT_MEASURED`. **Never describe this data as recorded from the real robot.** Model runs write JSON summary with peak absolute pitch, RMS pitch and actuator saturation frequency.

The virtual controller uses `Kp=0.09 command/deg` and `Kd=0.008 command/(deg/s)`, **not** the unverified source firmware's `Kp=0.016`, `Kd=0.0009` gains or physical motor polarity. The different gain choice illustrates why validation/tuning against the actual plant is essential; it does **not** validate those values for a real build.

Numerical method: zero-order-held controller inputs, fourth-order Runge–Kutta plant integration with `Ts=5 ms`. The open-loop comparison exits when pitch magnitude passes the arbitrary 45-degree validity threshold. This is not a sensor cutoff setting.

For MATLAB-side reproducibility, [`pd_theoretical_comparison.m`](../matlab/pd_theoretical_comparison.m) implements an equivalent idealized pitch-only RK4 virtual PD/open-loop study with the same nominal plant and gains. It remains **theoretical** and does not implement the firmware gains.

For the original course requirement, MATLAB controller design and real experimental analysis still must be completed. See [model_and_design.m](../matlab/model_and_design.m) and [Stage 2 report](../docs/reports/STAGE2_modeling_report.md).
