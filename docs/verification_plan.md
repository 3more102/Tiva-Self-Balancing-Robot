# Hardware and control verification plan

**Safety notice:** Spinning wheels, tip-over, battery shorts and thermal shutdown are physical risks. The firmware's 25-degree shutdown and switch-based gating do not replace a fuse, mechanical restraint and motor power kill switch. Qualified supervision is recommended.

| Test | Procedure | Acceptance criterion | Status |
|---|---|---|---|
| C controller unit tests | Compile and run `tests/test_controller.c` on host | Assertions pass with warnings treated as errors | Host-only, run locally |
| CCS firmware compile | Build for TM4C123GH6PM with installed CCS/TivaWare | Zero compile/link errors | NOT TESTED |
| I2C device identity | Probe WHO_AM_I (`0x75`) at address `0x68` | Value `0x68` | NOT TESTED |
| Static IMU | Upright +Z acceleration at rest, pitch approximately 0 | Axis/sign and sampling verified | NOT TESTED |
| Gyro bias | Keep still for initial 200 samples | Reproducible bias values | NOT TESTED |
| PWM without motors | Scope PB6/PB7 after **controlled** enabling | Approximately 20 kHz, correct complementary output mapping | NOT TESTED |
| Driver STBY | Default firmware; check PA5 | PA5 remains LOW | NOT TESTED |
| Wheel direction | Restrained chassis, low duty and physical E-stop | Both wheels move in physically correct robot-forward direction | NOT TESTED |
| Operator switch | Release SW1 while energized under safe test | Both motors disabled promptly | NOT TESTED |
| Tilt trip | Controlled tilting while supported | Trip >=25 degrees; release/re-arm required | NOT TESTED |
| Timing | Scope GPIO marker / DWT counter for I2C/control loop | Bounded 5ms period and jitter, no missed steps | NOT TESTED |
| Closed-loop balance | Tethered robot, incremental conservative gain tune | No runaway and stable bounded recovery | NOT TESTED |
| MATLAB match | Capture verified measured logs and fit parameters | Model and observed responses compared with quantitative error | NOT TESTED |

**Known next engineering requirements:** independent watchdog, latency profiling, motor current monitoring, measured model, telemetry collection, systematic polarity/tuning procedure, encoder support if accurate cart velocity/state feedback is needed.

## Firmware configuration warnings

- `MOTOR_OUTPUT_ALLOWED` is set to zero in `main.c`; compiled default **will not drive the motors**.
- `MOTOR_POLARITY`, `LEFT_MOTOR_POLARITY`, `RIGHT_MOTOR_POLARITY` and PID gains are provisional, not verified.
- If the IMU fails after initialization, firmware disables the motors and remains faulted until reset.
- The estimator presumes the listed physical sensor axes. Incorrect mounting gives the wrong pitch.
- Full MCU watchdog, encoder speed feedback and hardware emergency stop are not provided here.
