# GyroGuard TinyML — architecture and verification boundary

## System placement

The neural network is an outer navigation decision layer. It does not replace
the balance PID and it never writes motor GPIO/PWM directly.

    Front / left / right range sensors
                  |
          Validity + freshness checks <--- MPU6050 yaw-rate axis
                  |
            Feature extraction
                  |
             5-12-8-5 MLP
                  |
           Action + confidence
                  |
      Independent rule-based safety shield
                  |
      FORWARD / TURN / STOP / REASSESS
                  |
      50 Hz shadow diagnostics
      (no actuator authority)

    Independent balance path:
      MPU6050 pitch -> estimator -> 200 Hz PID
                                  |
                              Motor driver
                                  |
                                Wheels

The 200 Hz balance loop remains highest priority. The MPU6050 backend reads
gyro X for pitch and vertical-axis gyro Z for yaw; startup calibration
estimates both offsets, and the corrected runtime yaw rate is passed to the
AI sensor frame. The main loop services the range scheduler without a busy
wait and runs inference at 50 Hz in **shadow mode**.

An optional TivaWare HC-SR04-compatible adapter exists, but is disabled by
default until the actual module and voltage levels are confirmed. With the
default setting, all range readings are invalid and the AI safety shield
returns STOP. The AI action is exposed to CCS watch variables only: it does
not change the balance target, motor mixing or PWM. No physical navigation
performance is claimed.

## Network specification

| Layer | Width | Operation |
|---|---:|---|
| Input | 5 | Normalized sensor features |
| Hidden 1 | 12 | Dense + ReLU |
| Hidden 2 | 8 | Dense + ReLU |
| Output | 5 | Dense + softmax |

There are 221 trainable parameters (884 bytes of float32 weights). Inference
uses fixed-size stack arrays and no dynamic allocation. Softmax uses a
max-subtracted exponent to avoid overflow. The model is exported as a C header;
the MCU does not need NumPy or a machine-learning runtime.

## Range sensor acquisition contract

A portable asynchronous range layer schedules front/left/right echo
captures, enforces a 60 ms inter-ping gap, rejects invalid pulse widths,
expires missing echoes, and reports timestamped readings. The AI sensor
adapter maps those readings and calibrated yaw rate into the model's input
frame. An optional Tiva adapter uses PE0–PE5, Timer2A and Port E edge
interrupts for HC-SR04-compatible modules, but
`RANGE_HCSR04_BACKEND_ENABLED` defaults to `0`. Confirm the sensor model and
Echo voltage before enabling it. See [range-sensing architecture and tests](range_sensing.md).

## Safety shield

- Front data must be valid, nonzero, and no older than 250 ms.
- Side data must be valid, nonzero, and no older than 750 ms to authorize a turn.
- FORWARD is rejected if front distance is below 500 mm.
- TURN_LEFT is rejected unless left clearance is at least 500 mm.
- TURN_RIGHT is rejected unless right clearance is at least 500 mm.
- Missing front data, invalid yaw rate, invalid model output, or insufficient
  model confidence produces a stopped/reassessment decision.
- This software shield is not a substitute for a physical motor disconnect,
  fuse, current-limited supply, mechanical restraint, or hardware emergency stop.

All distance and timing thresholds are provisional starting values. They must
be set from measured sensor latency, robot speed, braking distance, geometry,
and motor response before any floor test.

## Training method and honest model claims

The checked-in MLP was trained by supervised behavior cloning. A synthetic
scenario generator samples open passages, approaching obstacles, narrow
corridors, and asymmetric clearances. A deterministic conservative expert
policy supplies the labels. Training and validation use separate random seeds.

The recorded validation accuracy (90.93%) is only accuracy against held-out
synthetic examples from that same expert policy. It is not physical navigation
accuracy and does not establish collision avoidance. No real robot dataset has
been collected in this repository.

The next model-development milestone is to log real range/yaw sequences,
label safe actions, split evaluation by complete runs, and compare the model
against the rule-based baseline using collision rate, successful completion,
unnecessary-turn rate, and stop distance.

## Host verification

tests/test_ai_navigation.c covers probability normalization, open-path forward
decisions, left/right selection, blocked-path stopping, stale/missing front
sensor fail-safe behavior, invalid side-sensor rejection, closing-speed
feature extraction, and autonomy disable.

tests/test_ai_dataset.py covers deterministic generation, known teacher
labels, feature normalization, and output dimensions. GitHub Actions also runs
a short training smoke test. These are host checks only; they do not compile the
complete TivaWare application or verify real sensor timing.
