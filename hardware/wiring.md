# Wiring and electrical preflight — provisional

Board: **TI EK-TM4C123GXL / TM4C123GH6PM**; driver **TB6612FNG**; sensor **MPU6050 (AD0 grounded)**.

| Source | Destination | Function |
|---|---|---|
| Tiva PB2 | MPU6050 SCL | I2C0 clock, 3.3 V pull-up |
| Tiva PB3 | MPU6050 SDA | I2C0 data, 3.3 V pull-up |
| Tiva 3.3 V | MPU6050 VCC **only if module accepts 3.3 V** | Sensor logic power |
| Tiva PB6 | TB6612 PWMA | PWM0 channel 0, left motor |
| Tiva PB7 | TB6612 PWMB | PWM0 channel 1, right motor |
| Tiva PC4 | TB6612 AIN1 | Left direction 1 |
| Tiva PC5 | TB6612 AIN2 | Left direction 2 |
| Tiva PC6 | TB6612 BIN1 | Right direction 1 |
| Tiva PC7 | TB6612 BIN2 | Right direction 2 |
| Tiva PA5 | TB6612 STBY | Driver enable, low = disabled |
| Tiva 3.3 V | TB6612 VCC | Driver logic voltage, check module specifications |
| Motor battery positive | TB6612 VM | Motor supply, size/fuse for stall current |
| Battery ground + Tiva ground + sensor ground | TB6612 GND and common GND | Required common reference |
| TB6612 AO1/AO2 | Left motor leads | Drive winding |
| TB6612 BO1/BO2 | Right motor leads | Drive winding |
| Onboard Tiva SW1 / PF4 | — | Hold-to-enable, release = coast/disable |

## Optional obstacle-range sensors — provisional HC-SR04-compatible map

An optional Tiva adapter is present for three HC-SR04-compatible trigger/echo modules, but it is **disabled by default** with `RANGE_HCSR04_BACKEND_ENABLED 0`. The actual sensor model has not been confirmed; do not enable or wire this map until its datasheet and electrical levels are checked.

| Sensor | Tiva TRIG | Tiva ECHO | Role |
|---|---|---|---|
| Front | PE0 | PE1 | Forward obstacle |
| Left | PE2 | PE3 | Left-side clearance |
| Right | PE4 | PE5 | Right-side clearance |

The optional adapter uses Timer2A to end a 12 µs trigger pulse and Port E GPIO interrupts to timestamp both Echo edges. Timer1A provides raw 80 MHz timestamps. The EK-TM4C123GXL manual shows PE0–PE5 on the LaunchPad expansion headers ([TI board manual](https://www.ti.com/lit/ug/spmu296/spmu296.pdf)).

**Voltage warning:** many HC-SR04-style modules use 5 V power and may output 5 V on Echo. Never connect a 5 V Echo signal directly to a TM4C123 GPIO. Use a verified level shifter or resistor divider unless the exact module guarantees a 3.3 V-safe Echo output. Confirm the module accepts the Tiva's 3.3 V Trigger level. Keep the backend disabled until this is verified.

The portable capture core and timing assumptions are documented in [range_sensing.md](../docs/range_sensing.md).

**Pin assignment caveat:** On the LaunchPad, PB6 is linked to PD0 and PB7 to PD1 via 0-ohm board resistors. Do **not** simultaneously configure PD0/PD1 as unrelated outputs while using PB6/PB7 PWM. Check your actual board revision and schematic.

**Safety preflight:**

1. Verify controller ground, motor power return, reverse-polarity protection and appropriate fusing. Ensure regulator current and battery motor stall-current headroom.
2. Confirm **actual** MPU module IO and voltage limits, breakout pull-ups, TB6612 logic voltage and VM range from manufacturer datasheets; do not assume generic breakout labels are safe at 5 V.
3. Motors **must never** be connected to MCU GPIO directly. Prefer physical kill switch on motor battery circuit. Keep wheels clear during direction/velocity testing.
4. Verify MPU sensor geometry: **+X axle; +Y forward; +Z up**. At rest `accel_z` should read close to +1g; when tilted forward confirm accelerometer angle sign and X-gyro sign agree.
5. With wheels lifted, assert temporary low duty and verify signed direction separately per wheel before any closed-loop trial. Sign configuration in `main.c` is a *hypothesis*.
6. Monitor supply sag/logic resets. Mechanical support/tether for first balance attempts is required.
7. Driver STBY is kept LOW by default firmware flag; to run motors require operator switch and explicit source change.

References: [TI board user manual](https://www.ti.com/lit/ug/spmu296/spmu296.pdf), [TM4C123 datasheet](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf). Consult breakout/driver datasheets before building.
