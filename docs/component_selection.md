# Component-selection and power-budget worksheet

No particular gearmotor, battery, fuse or charger has been purchased or validated in this repository. **Do not select these parts based only on the prototype source or placeholder MATLAB parameters.**

## 1. Proposed controller / IMU

- MCU: [TI EK-TM4C123GXL](https://www.ti.com/tool/EK-TM4C123GXL), built around the TM4C123GH6PM. Source uses onboard SW1 and LED GPIO and TivaWare I2C/PWM.
- IMU: [TDK MPU6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf). The actual module must accept its supply and I/O levels as wired. Check AD0, pull-up voltage, sleep settings, axis mounting and WHO_AM_I.

## 2. TB6612FNG evaluation (data-sheet figures)

The [Toshiba datasheet](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG) lists an operating supply range for VCC of **2.7–5.5 V** and VM **2.5–13.5 V**. It describes an output-current operating entry of **1.0 A** per channel when VM is at least 4.5 V; higher figures in absolute maximum / peak tables are **not continuous operating design targets**. Review ambient temperature, PCB copper, motor drive conditions and available thermal margin.

| Selection check | Worksheet value |
|---|---|
| Motor rated current at chosen motor voltage | **TBD** |
| Motor starting/stall current at maximum battery voltage | **TBD** |
| TB6612FNG channel operating rating with thermal derating | From exact datasheet/board |
| Battery maximum voltage at full charge | **TBD** |
| Battery discharge capability and protected pack specs | **TBD** |
| Harness, connector, fuse and disconnect rating | **TBD** |
| MCU regulator input/output, surge and reset behavior | **TBD** |

If measured or specified motor current exceeds the safe continuous capacity of the chosen driver, **select a more suitable driver**, not a larger software PWM limit.

## 3. Preliminary sizing expressions

```text
Motor stall current (simple brushed DC estimate):
I_stall ~ V_motor / R_winding
```

This is only a first-order approximation and may vary with temperature, cable/driver losses and manufacturer definition. Use manufacturer stall-current specifications where available; avoid dangerous direct stall testing on an unprotected battery.

For ideal traction:
```text
wheel force ~ wheel torque / wheel_radius
available body-correcting force depends on gear efficiency and wheel friction
```
Two-motor instantaneous current is **not** known without each motor's load and driver losses. Battery runtime cannot be estimated responsibly without the chosen motors, duty distribution, battery capacity and discharge curves.

## 4. Schematic review checklist

- [ ] Battery wiring and protection drawn and labeled
- [ ] VM range at full charge and VCC logic range checked
- [ ] Continuous and transient current ratings including copper/wiring thermal conditions
- [ ] Common ground design / appropriate star returns for motor power
- [ ] Decoupling capacitors adjacent to modules
- [ ] I2C pull-ups to safe voltage
- [ ] Motor terminal polarity (never connect to MCU pins)
- [ ] STBY default disabled by hardware-level bias
- [ ] Reset/brownout response and external motor supply cutoff
- [ ] Mechanical clearance, wire strain relief and connector keys

## 5. BOM completion

Fill [hardware/bom.csv](../hardware/bom.csv) with exact vendor URLs, quoted price/unit and quantity, purchase date and EGP total before Stage 1 submission. Prices are intentionally unfilled until confirmed.
