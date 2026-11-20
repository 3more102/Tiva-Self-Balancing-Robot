# Portable ultrasonic range-capture engine

**Status:** host-tested portable C module. It is not yet connected to Tiva C GPIO pins or the physical sensors.

## Purpose

The navigation policy expects front, left and right range values. This module supplies the timing/state-machine layer for those sensors without selecting board pins or blocking the 200 Hz balance loop.

Create one independent `RangeCapture` object per sensor. The hardware adapter should:

1. Trigger only one ultrasonic module at a time to reduce acoustic cross-talk.
2. Start its capture object at the trigger time.
3. Timestamp both ECHO edges from a free-running timer.
4. Call `range_capture_echo_edge()` from the corresponding GPIO edge handler.
5. Call `range_capture_poll()` from the foreground loop and publish only completed, valid readings to `NavigationInput`.

The API uses monotonically increasing 32-bit timer ticks. The current Tiva design runs Timer1A as a 32-bit down-counter at 80 MHz; pass `~TimerValueGet(TIMER1_BASE, TIMER_A)` as the timestamp to obtain an increasing tick count. Configure the HC-SR04 timing thresholds in the same tick domain:

| Parameter | At 80 MHz |
|---|---:|
| Echo timeout (25 ms) | 2,000,000 ticks |
| Minimum pulse (117 us) | 9,360 ticks |
| Maximum pulse (17.5 ms) | 1,400,000 ticks |

These are initial assumptions, not universal HC-SR04 guarantees. Confirm the exact module's range, trigger requirements, ECHO voltage and timing before selecting values.

The ISR/foreground adapter must serialize access to each capture object. On Tiva C, mask the corresponding ECHO GPIO interrupt briefly while polling/copying the result, then re-enable it. Do not wait for an echo in the balance-control path. Trigger sensors sequentially and allow sufficient quiet time between pings to reduce acoustic cross-talk.

## Distance conversion

The conversion function accepts the measured pulse width in timer ticks and the actual timer frequency:

`distance_mm = round(pulse_width_ticks * 343000 / (2 * timer_hz))`

This uses the approximate speed of sound at 20 C; the factor of two accounts for the outbound and return journey.

## Safety and limitations

- Invalid pulse widths, missing echoes, stuck-high ECHO and late edges produce invalid results.
- Unsigned elapsed-time subtraction supports 32-bit timer rollover for configured intervals no greater than 2^31-1 ticks.
- Three instances are independent, but sensors must still be triggered sequentially.
- The module does not assign GPIO pins, generate TRIG pulses, configure NVIC, or implement the TivaWare interrupt adapter.
- The navigation main loop remains hard-disabled until all required range inputs and a separate operator autonomy-enable input are wired and verified.
- `MOTOR_OUTPUT_ALLOWED` remains `0`. Host tests do not qualify the target build, electrical interface or physical motion.

## Host tests

```sh
gcc -std=c11 -O2 -Wall -Wextra -Werror -pedantic \
  -Ifirmware/tiva/include \
  firmware/tiva/src/range_capture.c tests/test_range_capture.c \
  -o /tmp/test_range_capture
/tmp/test_range_capture
```

The regression suite covers pulse-width boundaries, missing/stuck echoes, late edges, busy/result handling, timer rollover, distance conversion and three independent sensor instances.
