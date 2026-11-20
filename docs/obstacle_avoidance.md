# Obstacle avoidance — Tiva C integration

**Status:** host-tested software foundation; HC-SR04 wiring and behavior have not been tested on the physical robot. This feature does not enable the motor driver.

## Hardware assumption

Use one forward-facing HC-SR04-compatible ultrasonic range module:

| Signal | Tiva C connection |
|---|---|
| TRIG | PD2 |
| ECHO | PD3 through a 5 V-to-3.3 V divider |
| VCC | Regulated 5 V supply, if required by the actual module |
| GND | Common logic ground with Tiva C |

A common HC-SR04 board is powered from 5 V and may return a 5 V ECHO signal. **Never connect a 5 V ECHO output directly to a Tiva C GPIO.** For a 5 V output, a 10 kΩ resistor from ECHO to PD3 and a 20 kΩ resistor from PD3 to ground form an approximately 3.3 V divider. Verify the actual module's electrical specification before wiring. The Tiva TRIG output is 3.3 V; verify that the chosen module recognizes it as HIGH.

## Non-blocking measurement

- Timer1A is already configured as a free-running 32-bit down-counter at the 80 MHz system clock.
- PD3 GPIO interrupts capture both ECHO edges. The ISR stores only the pulse width in timer ticks.
- The main loop starts a 10 μs trigger pulse about every 60 ms and consumes completed measurements without waiting for ECHO.
- A missing ECHO becomes an invalid sample after 25 ms. Accepted pulse widths correspond approximately to 2–300 cm.
- Distance is calculated using the speed-of-sound approximation:

  d_cm = pulse_ticks × 34300 / (2 × timer_clock_hz)

The range measurement is asynchronous; it must not delay the nominal 200 Hz pitch-control loop.

## Navigation state machine

1. **CLEAR:** no steering correction. A valid reading below 25 cm starts an avoidance turn.
2. **TURNING:** apply a provisional differential steering bias of 0.16 for 650 ms. The initial direction alternates on each new obstacle.
3. **WAIT_CLEAR:** no steering correction until a valid reading reaches at least 40 cm. This hysteresis prevents immediate repeated turns on the same obstacle.

The mixer combines balance and steering terms:

- left command = balance command − steering
- right command = balance command + steering

The pitch controller remains the high-priority control loop. An invalid ultrasonic measurement suppresses navigation steering only; it must not disable the pitch correction needed to keep the robot upright. A failed MPU6050 sample or invalid balance-loop timing still follows the existing balance safety shutdown.

## Known limitations

- The 25 cm / 40 cm thresholds, 0.16 steering bias and 650 ms turn are starting values, not tuned robot parameters.
- This is a single forward sensor with timed, alternating differential steering. It does not measure yaw angle, wheel speed or turn completion.
- No wheel encoders, yaw feedback, scanning servo, velocity controller or map are implemented.
- Sensor blind zones, soft/angled surfaces, cross-talk, chassis vibration and mounting geometry require physical testing.
- The compile-time motor interlock remains disabled. Do not change it before electrical review, target build, low-power direction tests and independent motor cutoff verification.

## Host tests

    gcc -std=c11 -Wall -Wextra -Werror -pedantic \
        -Ifirmware/tiva/include \
        firmware/tiva/src/obstacle_avoidance.c \
        tests/test_obstacle_avoidance.c -lm -o test_obstacle_avoidance
    ./test_obstacle_avoidance

These tests verify state-machine behavior only; they do not simulate ultrasonic acoustics or prove physical obstacle avoidance.
