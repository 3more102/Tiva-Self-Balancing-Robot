# Three-sensor ultrasonic scanner

**Status:** portable, host-tested round-robin scheduler, scanner-to-navigation adapter and hardware-callback acquisition coordinator. No TivaWare GPIO backend or physical sensor wiring is implemented yet.

## Architecture

The navigation policy consumes three logical samples: front, left and right. The range scanner schedules exactly one sensor at a time, forwards only the active sensor's ECHO edges to its own capture state, and publishes a completed sample with a millisecond timestamp.

```mermaid
flowchart TD
    LOOP["200 Hz foreground loop"] --> NEXT["Ask scanner for next sensor"]
    NEXT --> COORD["range_acquisition_step()"]
    COORD --> TRIG["Hardware callback emits short TRIG"]
    TRIG --> CAP["Start one RangeCapture instance"]
    ECHO["GPIO ECHO edge ISR"] --> CAP
    CAP --> POLL["Non-blocking scanner poll"]
    POLL --> VALID{"Pulse valid and within max range?"}
    VALID -->|yes| SAMPLE["Publish distance + sample timestamp"]
    VALID -->|no / timeout| INVALID["Publish invalid sample"]
    SAMPLE --> NAV["NavigationInput front / left / right"]
    INVALID --> NAV
    NAV --> LOOP
```

## API contract

- `range_scanner_next_trigger()` returns the next round-robin sensor only when no capture is active and the configured quiet interval has elapsed.
- `range_scanner_begin()` must be called immediately before the hardware TRIG pulse. It starts that sensor's independent `RangeCapture`.
- The GPIO ISR calls `range_scanner_echo_edge()` with the physical sensor ID and the current up-counting timer timestamp.
- `range_scanner_poll()` is called from the foreground loop. It never waits; on completion it publishes either a valid distance or an invalid sample.
- `range_scanner_get_sample()` copies the most recently published result. `has_sample` distinguishes "not measured yet" from a measured timeout; `valid` distinguishes a usable distance from an invalid reading.
- `range_navigation_build_input()` copies the three latest samples into the exact `NavigationInput` fields used by `navigation_update()`. It keeps the separate autonomy-enable argument independent of sensor validity and maps missing/invalid samples to `valid=false`, `distance_mm=0`.
- `range_acquisition_step()` is the foreground coordinator. It masks only the ECHO GPIO interrupts while polling the scanner, starts at most one next sensor and calls the hardware TRIG callback. It then restores ECHO interrupts. Call it once per foreground control iteration.
- `range_acquisition_echo_edge()` is called by the GPIO ISR with the physical sensor ID and pin level. It reads the current tick timestamp through the hardware callback and forwards the edge to the scanner.
- The hardware callbacks provide the free-running up-counting timer ticks, the navigation millisecond clock, ECHO interrupt masking and a short non-waiting TRIG pulse. For Timer1A's down-counter, return `~TimerValueGet(TIMER1_BASE, TIMER_A)`.
- The ECHO interrupt-enable callback must not clear pending edge flags when re-enabling. The TRIG callback must not wait for ECHO; a candidate 10 us pulse is a brief bounded operation, not a sensor timeout wait.
- The navigation policy, not the scanner adapter, evaluates each sample timestamp against the front/side freshness limits.

The coordinator serializes scanner access with the ECHO ISR by masking only ECHO GPIO interrupts around its short shared-state section. It does not mask Timer0A/Timer1A or wait for an echo. The board-specific backend must preserve pending ECHO edge flags while interrupts are masked.

## Initial timing profile

The profile below is a software starting point for a candidate HC-SR04-compatible module at an 80 MHz timer clock. Confirm the exact module datasheet before hardware use.

| Parameter | Value |
|---|---:|
| Timer frequency | 80 MHz |
| Echo timeout | 25 ms / 2,000,000 ticks |
| Accepted pulse width | 117 us–17.5 ms |
| Minimum interval between any two sensor triggers | 60 ms / 4,800,000 ticks |
| Maximum published distance | 4,000 mm |
| Three-sensor scan period | approximately 180 ms, plus foreground completion latency |

A 60 ms inter-trigger interval is conservative against acoustic cross-talk and keeps a full three-sensor round-robin cycle below the navigation policy's 250 ms sample-age limit under the stated assumptions. Verify actual acoustic behavior and sensor latency on the assembled chassis; increase the quiet interval if cross-talk is observed.

The scanner and coordinator do not assign GPIO pins, configure NVIC, or enable autonomy. A board-specific TivaWare backend must still map each physical TRIG/ECHO pin, configure both-edge GPIO interrupts, dispatch each ECHO edge to the correct sensor, and verify timing on the actual board. Physical trigger/ECHO mapping and voltage compatibility remain board- and module-specific. A 5 V ECHO output must not be connected directly to a 3.3 V TM4C GPIO; use a verified level-shifting circuit.

## Safety

- Timeout, invalid pulse width, and distances above the configured maximum publish `valid=false` and `distance_mm=0`; never treat no-echo as clear space.
- Only one sensor is active at a time.
- Unsigned elapsed-tick subtraction supports timer rollover when intervals are no greater than 2^31-1 ticks.
- A sample is timestamped when the foreground consumes the completed capture. This timestamp is slightly later than the physical echo; retain the existing conservative freshness limits.
- The navigation policy and scanner do not command motors directly. `MOTOR_OUTPUT_ALLOWED` remains `0` until hardware qualification.

## Host tests

```sh
gcc -std=c11 -O2 -Wall -Wextra -Werror -pedantic \
  -Ifirmware/tiva/include \
  firmware/tiva/src/range_capture.c firmware/tiva/src/range_scanner.c \
  tests/test_range_scanner.c -o /tmp/test_range_scanner
/tmp/test_range_scanner

gcc -std=c11 -O2 -Wall -Wextra -Werror -pedantic \
  -Ifirmware/tiva/include \
  firmware/tiva/src/range_capture.c firmware/tiva/src/range_scanner.c \
  firmware/tiva/src/range_navigation.c firmware/tiva/src/range_acquisition.c \
  tests/test_range_acquisition.c -o /tmp/test_range_acquisition
/tmp/test_range_acquisition
```

Tests cover round-robin order, one-active-sensor enforcement, wrong-sensor edge rejection, valid/invalid/timeout publication, range bounds, timer rollover and invalid API arguments. `tests/test_range_navigation.c` verifies front/left/right field mapping, missing samples, timeout handling and the independent autonomy-enable gate. `tests/test_range_acquisition.c` uses mock hardware callbacks to verify trigger sequencing, quiet-time enforcement, ECHO ISR routing, timeout invalidation and navigation-input publication.

To compile and run the scanner-to-navigation adapter tests:

```sh
gcc -std=c11 -O2 -Wall -Wextra -Werror -pedantic \
  -Ifirmware/tiva/include \
  firmware/tiva/src/navigation.c firmware/tiva/src/range_capture.c \
  firmware/tiva/src/range_scanner.c firmware/tiva/src/range_navigation.c \
  tests/test_range_navigation.c -lm -o /tmp/test_range_navigation
/tmp/test_range_navigation
```
