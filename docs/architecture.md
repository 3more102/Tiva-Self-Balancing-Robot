# Engineering architecture and block diagrams

**Status:** preliminary design, not an as-built schematic. Written for the NANENG 512 assignment, using the TM4C123GH6PM in place of Arduino **only if the instructor approves**. All signals, supply domains, polarity and motor specifications must be checked against the hardware actually purchased.

## 1. Functional block diagram

```mermaid
flowchart LR
    REF["Upright pitch reference<br/>0 degrees"] --> SUM((Σ))
    SEN["MPU6050<br/>accel Y/Z + gyro X"] --> I2C["I2C0<br/>PB2=SCL, PB3=SDA"]
    I2C --> FILTER["Complementary estimator<br/>theta_hat, gyro_rate"]
    FILTER --> SUM
    SUM --> CTRL["Digital balance controller<br/>PD/PID @ nominal 200 Hz"]
    FILTER --> SAFETY["Safety state machine<br/>angle + sensor + SW1"]
    SWITCH["PF4 / SW1<br/>operator enable"] --> SAFETY
    CTRL --> GATE["Safe command gate"]
    SAFETY --> GATE
    GATE --> PWM["PWM0: PB6, PB7<br/>direction PC4-PC7, enable PA5"]
    PWM --> DRIVER["TB6612FNG<br/>dual H bridges"]
    BAT["Battery + fuse + physical disconnect"] --> DRIVER
    DRIVER --> MOTOR["Left and right geared DC motors"]
    MOTOR --> ROBOT["Two-wheel chassis<br/>unstable mechanical plant"]
    ROBOT -. "physical pitch" .-> SEN
```

**Why these blocks?** Pitch is inferred from the rate gyro and gravity component of acceleration; a discrete controller computes a normalized motor command, but the safety gate can always force zero output. The battery/driver controls motor energy; MCU GPIO never directly supplies motor current.

## 2. Control-loop signal-flow diagram

```mermaid
flowchart TD
    A["At 200 Hz (target): trigger / timestamp"] --> B["Read consecutive 14 MPU6050 registers"]
    B --> C{"I2C OK and sensor values usable?"}
    C -- "No" --> STOP["Motor STBY low; latch sensor fault"]
    C -- "Yes" --> D["ay, az in g<br/>gx in deg/s"]
    D --> E["theta_acc = atan2(ay, az) * 180/pi"]
    E --> F["alpha = tau/(tau+dt)<br/>theta = alpha*(theta+dt*(gx-bias))+(1-alpha)*theta_acc"]
    F --> G{"Safe state machine allows output?"}
    SW["SW1: release then hold;<br/>angle <=8deg for 40 samples"] --> G
    G -- "No" --> OFF["Reset controller integral;<br/>STBY low"]
    G -- "Yes" --> H["error=-theta<br/>u = Kp*error + Ki*integral - Kd*(gx-bias)"]
    H --> I["Conditional anti-windup + clamp<br/>abs(u)<=0.7"]
    I --> J["Verify physical sign & motor mapping"]
    J --> K["Direction PC4..PC7<br/>PWM PB6/PB7, STBY PA5"]
    OFF --> NEXT["Next sampling tick"]
    K --> NEXT
    STOP --> NEXT
```

Note: at present `dt` is a **nominal constant** 5 ms, not a measured per-cycle timestamp; real-time jitter and missed deadlines have **not** been characterized. Firmware motor output remains compile-time disabled. This diagram is a conceptual control flow, not proof of safe real-time timing.

## 3. Power and interconnection diagram

```mermaid
flowchart TD
    BAT["Motor battery<br/>voltage TBD"] --> FUSE["Fuse + physical disconnect<br/>required, not supplied"]
    FUSE --> VM["TB6612FNG VM<br/>motor power"]
    VM --> LM["Left motor"]
    VM --> RM["Right motor"]
    USB["USB/regulated MCU power"] --> TIVA["Tiva C LaunchPad"]
    TIVA --> LOGIC["3.3 V logic rail"]
    LOGIC --> MPU["MPU6050 breakout<br/>3.3 V compatible"]
    LOGIC --> VCC["TB6612 VCC logic"]
    TIVA -->|PB2 PB3| MPU
    TIVA -->|PB6 PB7, PC4-PC7, PA5| VCC
    GND["Common ground reference"] --- VM
    GND --- TIVA
    GND --- MPU
```

**Warning:** This is a functional wiring diagram, **not** a circuit schematic, PCB layout or complete electrical design. Grounding, decoupling, current paths, regulators, protection, proper driver connections and motor polarity must be drawn and verified before power-on.

## 4. Safety / arming state diagram

```mermaid
stateDiagram-v2
    [*] --> DISARMED : Power-on
    DISARMED --> DISARMED : No release observed or tilt > 8deg
    DISARMED --> ARMING : Release seen AND switch held AND tilt <= 8deg
    ARMING --> DISARMED : Switch released or tilt > 8deg
    ARMING --> ARMED : 40 consecutive accepted upright samples
    ARMED --> DISARMED : Switch released
    ARMED --> FAULT : abs(tilt) >= 25deg, bad sample or missed interval
    DISARMED --> FAULT : Non-finite/bad sensor sample
    ARMING --> FAULT : Non-finite/bad sensor sample
    FAULT --> DISARMED : Switch released AND sensor data valid
```

`imu_ok=false` is independently latched at application level; for a communication failure, hardware reset is required. **No safety state is a substitute for a physical power disconnect.** Debouncing and a hardware watchdog are not yet implemented.

## 5. Proposed electrical mapping

| Tiva pin | Signal | Destination | Direction | Important condition |
|---|---|---|---|---|
| PB2 | I2C0SCL | MPU6050 SCL | output | correct 3.3 V pull-up |
| PB3 | I2C0SDA | MPU6050 SDA | bidirectional | correct 3.3 V pull-up |
| PB6 | M0PWM0 | TB6612 PWMA | output | shares board link with PD0 |
| PB7 | M0PWM1 | TB6612 PWMB | output | shares board link with PD1 |
| PC4 / PC5 | AIN1 / AIN2 | Driver motor A | output | check forward polarity |
| PC6 / PC7 | BIN1 / BIN2 | Driver motor B | output | check forward polarity |
| PA5 | STBY | Driver standby | output | **LOW by default** |
| PF4 | SW1 | Onboard push-button | input | active-low; hold-to-enable |
| PF1 / PF2 / PF3 | RGB LEDs | Onboard LEDs | output | error / standby / enabled indication |

Compare with [hardware/wiring.md](../hardware/wiring.md) and the [EK-TM4C123GXL manual](https://www.ti.com/lit/ug/spmu296/spmu296.pdf).

## 6. Data and requirements traceability

| Requirement | Design element | Evidence available | Still missing |
|---|---|---|---|
| Sensor orientation | MPU6050 accel + gyro; complementary filter | Portable C source and host tests | I2C waveform/real sensor log |
| Closed-loop balance | PID + TB6612 PWM | Algorithm source | Actual motor sign/gains and repeatable tests |
| Safety | STBY low, off switch, trip angle, arming state machine | Static code + host tests | End-to-end physical cutoff timing |
| Control analysis | MATLAB linear model and nominal 200 Hz discretization | MATLAB script | Validated physical parameters / log data |
| Stage 1 architecture | These Mermaid diagrams, pin map, component responsibilities | This document | Reviewed electrical schematic |
| Mechanical integration | Two-wheel chassis model | Concept | CAD, dimensions, center of mass |
