# Portable ultrasonic range-capture engine

**Status:** host-tested portable C module. It is not yet connected to Tiva C GPIO pins or the physical sensors.

## Purpose

The navigation policy expects front, left and right range values. This module supplies the timing/state-machine layer for those sensors without selecting board pins or blocking the 200 Hz balance loop.

Create one independent `RangeCapture` object per sensor. The hardware adapter should:

1. Trigger only one ultrasonic module at a time to reduce acoustic cross-talk.
2. Start its capture object at the trigger time.
3. Timestamp both ECHO edges from a free-running 1 MHz timer.
4. Call `range_capture_echo_edge()` from the corresponding GPIO edge handler.
5. Call `range_capture_poll()` from the foreground loop and publish only completed, valid readings to `NavigationInput`.

The ISR/foreground adapter must serialize access to each capture object. On Tiva C, mask the corresponding ECHO GPIO interrupt briefly while polling/copying the result, then re-enable it. Do not wait for an echo in the balance-control path.

## Initial HC-SR04 timing profile

The initial software profile assumes a module whose verified datasheet supports:

| Parameter | Initial value |
|---|---:|
| Echo timeout | 25,000 us |
| Minimum pulse | 117 us |
| Maximum pulse | 17,500 us |
| Timer resolution | 1 us |

These are starting values, not universal HC-SR04 guarantees. Confirm the exact module's range, trigger requirements, ECHO voltage and timing before selecting values.

Distance conversion uses the approximate speed of sound at 20 C:

`distance_mm = round(pulse_width_us * 343 / 2000)`

The factor of two accounts for the outbound and return journey.

## Safety and limitations

- Invalid pulse widths, missing echoes, stuck-high ECHO and late edges produce invalid results.
- Unsigned elapsed-time subtraction supports the 32-bit microsecond timer rollover for the configured sub-2^31-us timeout.
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
