# GyroGuard integration sequence and branch policy

**Audit date:** 2026-11-20  
**Default branch baseline:** `main` at `4ed697512f4b2ef02eb0da5c0335c35e0fd5f839`.

This document prevents parallel feature branches from being merged blindly.
The repository has two substantial, independently developed tracks: a
rule-based navigation/range-acquisition track and the GyroGuard TinyML track.

## 1. Current tracks

### Track A — rule-based navigation and modular range acquisition

Branch: `feat/range-acquisition-coordinator-20261120`

Compared with `main`, this branch is 116 commits ahead and contains:

- `navigation.c/.h`: fail-safe rule-based obstacle policy.
- `range_capture.c/.h`: portable non-blocking pulse capture state machine.
- `range_scanner.c/.h`: sequential front/left/right scheduling.
- `range_navigation.c/.h`: scanner-to-navigation input adapter.
- `range_acquisition.c/.h`: hardware-callback acquisition coordinator.
- WDOG0 foreground supervision and related safety documentation.

The coordinator is a portable software layer. Its hardware callbacks are
mockable; it does not yet constitute a verified TivaWare CCP capture backend
or physical sensor integration.

### Track B — GyroGuard TinyML

Pull request: [PR #9](https://github.com/3more102/Tiva-Self-Balancing-Robot/pull/9)  
Branch: `feature/gyroguard-tinyml-20261120`

This branch contains:

- Native-C 5-12-8-5 MLP inference and model weights.
- Synthetic behavior-cloning dataset generator and training script.
- AI action selection and independent software safety shield.
- A separate portable range-sensing core and optional, disabled Port E GPIO
  interrupt adapter.
- MPU6050 yaw-axis decoding and shadow-mode AI integration.

The branch has now also received WDOG0 foreground supervision and expanded
classification metrics. Its range implementation overlaps conceptually with
Track A but uses different types and integration boundaries.

## 2. Stacked pull-request chain

These open pull requests form a dependency chain:

1. PR #4 — obstacle-avoidance policy foundation.
2. PR #7 — watchdog and navigation hardening; based on PR #4.
3. PR #8 — range-capture core; based on PR #7.
4. PR #10 — three-sensor scheduler; based on PR #8.
5. PR #11 — scanner-to-navigation adapter; based on PR #10.

If this PR chain is selected, review/merge strictly from the base upward, or
retarget each branch after its parent is merged. Do not merge a child before
its base.

The coordinator branch in Track A appears to consolidate these same
navigation/range layers. Choose the coordinator branch **or** the stacked PR
chain as the source of those modules; do not merge both copies without a
file-level comparison.

## 3. Duplicate watchdog proposals

PR #3, PR #5 and PR #6 are separate WDOG0 implementations. PR #7 also adds
watchdog hardening as part of the navigation stack. Track A already contains
a WDOG0 implementation, and PR #9 now has a foreground WDOG0 integration.

Do not merge all watchdog proposals. Select one reviewed implementation and
retain one source of truth for:

- reload period and second-timeout reset behavior;
- service points in startup, calibration and control loop;
- debugger-halt behavior;
- reset-time TB6612 STBY containment;
- target fault-injection evidence.

The current PR #9 implementation is source-integrated only. It is not yet
CCS/TivaWare target-compiled or board-tested.

## 4. Recommended canonical integration path

1. Preserve `main` as the stable baseline; do not merge open feature branches
   automatically.
2. Use Track A's modular range scanner/acquisition coordinator and
   rule-based navigation as the candidate navigation foundation, subject to
   reviewing its branch CI and source diffs.
3. Create a dedicated integration branch from that foundation.
4. Port only the AI model, native-C inference, training pipeline, metrics,
   AI decision logic and AI tests from Track B.
5. Add a small adapter from the Track A `RangeSample`/`NavigationInput`
   contract to the AI `AiSensorFrame`; avoid carrying two competing range
   capture engines into the integrated firmware.
6. Keep rule-based safety policy authoritative. Run AI in shadow mode first;
   AI must never bypass the balance controller, operator enable or motor
   safety gate.
7. Add host integration tests for range scanner -> navigation input -> AI
   frame -> AI inference -> independent safety shield.
8. Only after the portable integration is stable, implement the TivaWare
   GPTM CCP capture backend and perform CCS target compilation.
9. Keep all PRs open until their changes have been reviewed against the
   canonical integration branch. Do not merge or close PRs without explicit
   authorization.

## 5. Hardware-capture constraint found during audit

The current Track B optional adapter timestamps Port E GPIO interrupts with a
Timer1A read; it is not GPTM CCP hardware input capture. PE0–PE5 do not expose
CCP alternate functions in the TM4C123GH6PM pin-mux table.

A candidate hardware-capture mapping has been documented in
[range_sensing.md](range_sensing.md) and [hardware/wiring.md](../hardware/wiring.md):
PD2/WT3CCP0, PD3/WT3CCP1 and PD6/WT5CCP0. This is a datasheet-derived
candidate, not approved wiring or an implemented backend. Verify the actual
LaunchPad revision, pin mux, timer allocation, harness and Echo voltage before
use.

## 6. Evidence boundaries

- Host CI passing proves only the portable tests included in that workflow.
- It does not prove TivaWare target compilation, I2C/PWM behavior, capture
  latency, physical sensor accuracy, motor-driver safety or robot balancing.
- The current compile-time actuator interlock remains
  `MOTOR_OUTPUT_ALLOWED 0`.
- No branch in this audit is evidence of physically stable balancing or
  successful autonomous obstacle avoidance.
