# GyroGuard V1 integration sequence

**Audit date:** 2026-11-20  
**Integration branch:** `feature/gyroguard-ai-integration-20261120`  
**Base:** `feat/range-acquisition-coordinator-20261120`

## Why this branch exists

The repository had two independent software tracks:

1. A rule-based navigation and modular range-acquisition track with a
   portable range-capture engine, three-sensor scanner, navigation adapter,
   callback-based acquisition coordinator and WDOG0 foreground supervision.
2. A TinyML track with native-C inference, synthetic behavior-cloning training,
   an AI safety shield and a separate range adapter.

Merging the two tracks directly would duplicate range engines and conflict in
`main.c`. This integration branch uses the modular range-acquisition track as
the foundation and ports only the AI model, training pipeline, tests and
shadow-mode firmware integration.

## Canonical architecture

- `navigation.c` remains the rule-based navigation policy.
- `range_capture.c`, `range_scanner.c`, `range_navigation.c` and
  `range_acquisition.c` remain the canonical portable range stack.
- `ai_range_adapter.c` maps the canonical `NavigationInput` to
  `AiSensorFrame`; it does not introduce a second range-capture engine.
- `ai_navigation.c` and `ai_inference.c` run at 50 Hz after the balance
  command in shadow mode.
- MPU6050 gyro X is used for pitch control; gyro Z is calibrated separately
  and passed to the AI as yaw rate.
- AI decisions are diagnostic-only. They do not alter balance targets, motor
  mixing, GPIO or PWM.
- WDOG0 foreground supervision remains in the balance firmware path.
- `MOTOR_OUTPUT_ALLOWED 0` remains unchanged.

## Current integration boundary

The portable range stack is not yet wired into `main.c` through a TivaWare
hardware backend. `NavigationInput` starts with invalid range readings and
`autonomy_enabled=false`. The AI frame's `inference_enabled` gate follows the
balance-armed state and is independent of the rule-based autonomy gate, so the
model can evaluate shadow inputs without authorizing travel. Current front
range data is invalid, however, so the AI safety shield returns STOP and no
live sensor evaluation is yet demonstrated. This is intentional fail-safe
behavior, not a completed autonomous robot.

The existing optional GPIO-interrupt range prototype on the separate TinyML
branch is not copied. The current Track A range stack needs a real TivaWare
backend. The final backend should use GPTM CCP hardware capture, not treat a
GPIO ISR timestamp as hardware capture. Candidate PD2/PD3/PD6 mappings and
electrical caveats are documented in [hardware/wiring.md](../hardware/wiring.md).

## Branch and PR policy

- `feat/range-acquisition-coordinator-20261120` is the navigation/range
  foundation used by this branch.
- PR #4 → #7 → #8 → #10 → #11 is a stacked alternative path for the same
  navigation/range functionality. Do not merge that stack on top of this
  branch without a file-level reconciliation.
- PR #9 is the independent TinyML track. Its AI components have been ported
  here; do not merge its separate range-sensing implementation into this
  branch.
- PR #3, #5 and #6 are duplicate watchdog proposals. The canonical branch
  retains one WDOG0 implementation; do not merge duplicate watchdog copies.
- Keep all existing PRs open until reviewed. No PR is merged by this change.

## Next milestones

1. Review the integrated source; host CI passed on commit `b24d12f82b107fec6b86073986eafd87f5f25bd2` in [run 44779316855](https://github.com/3more102/Tiva-Self-Balancing-Robot/actions/runs/44779316855).
2. Add the TivaWare GPTM CCP backend and connect the range acquisition
   coordinator to the main loop.
3. Add and verify a separate operator autonomy-enable input.
4. Compile the full target with the actual CCS/TivaWare installation.
5. Perform motor-power-isolated IMU, watchdog, timer and sensor tests.
6. Measure loop timing/jitter and validate the motor-driver interface with
   wheels raised and power limited.
7. Physically balance and tune the robot before enabling any navigation action.
8. Collect real sensor runs and evaluate the model against the rule-based
   baseline; only then consider giving AI any navigation authority.

## Evidence rules

Host tests validate portable algorithms only. They do not establish CCS target
build success, GPTM capture timing, Echo voltage compatibility, motor safety,
physical balancing or collision avoidance. Record each target/hardware test
with board revision, component part numbers, toolchain versions, test setup,
raw measurements and commit SHA.
