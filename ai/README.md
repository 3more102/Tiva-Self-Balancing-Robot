# GyroGuard TinyML

This directory contains the supervised training pipeline for the small neural
network deployed as native C on the TI TM4C123GH6PM.

## What the model does

The model chooses one of five high-level navigation actions:

1. FORWARD
2. TURN_LEFT
3. TURN_RIGHT
4. STOP
5. REASSESS

It does not control PWM, motor direction, balancing torque, or safety arming.
The existing balance controller remains the high-priority inner control loop.
A separate C safety shield rejects forward commands when front clearance is
too small and rejects turns toward invalid, stale, or insufficiently clear
side sensors.

## Model

- Architecture: 5 → 12 → 8 → 5 multilayer perceptron.
- Activations: ReLU in hidden layers, softmax at output.
- Trainable parameters: 221.
- Stored float32 weights: 884 bytes.
- Runtime: native C; no Python, ESP32, cloud, or TensorFlow runtime on the MCU.

### Input order and units

| Index | Feature | Raw unit | Normalization |
|---:|---|---|---|
| 0 | Front range | mm | clamp(distance / 2000, 0, 1) |
| 1 | Left range | mm | clamp(distance / 2000, 0, 1) |
| 2 | Right range | mm | clamp(distance / 2000, 0, 1) |
| 3 | Front closing speed | m/s | clamp(speed / 2, -1, 1); positive means approaching |
| 4 | Robot yaw rate | deg/s | clamp(rate / 180, -1, 1) |

The yaw-rate input must be mapped to the MPU6050 gyro axis that is physically
aligned with the robot's vertical axis. Verify sensor orientation before use.
A stale/invalid front sample causes an immediate STOP; stale side samples
cannot authorize a turn.

## Training and export

Python 3.10+ and NumPy are required on the development computer only:

    python3 -m pip install -r ai/requirements.txt
    python3 ai/train_model.py

This regenerates:

- firmware/tiva/include/ai_model_weights.h — weights embedded in the MCU build.
- ai/model_metrics.json — reproducibility and held-out validation metrics.

The evaluation artifact includes a labeled confusion matrix, per-class
precision/recall/F1/support, macro precision/recall/F1, and support-weighted
F1. Macro averages include all configured actions; classes with zero support
are reported as zero rather than silently omitted. These scores describe
agreement with the synthetic teacher policy only.

For a quick smoke test:

    python3 ai/train_model.py --samples 10000 --epochs 30 --output /tmp/ai_model_weights.h --metrics /tmp/ai_model_metrics.json

## Training-data provenance and limitations

generate_dataset.py creates synthetic range-sensor scenarios and labels them
with a conservative expert rule policy. The MLP is trained to imitate that
policy (supervised behavior cloning). The checked-in model's reported 90.93%
accuracy is measured only against held-out synthetic examples generated from
the same teacher policy. It is not real-robot accuracy, collision-rate
evidence, reinforcement learning, or proof of safe navigation.

Before physical autonomy, collect and label real sensor data across different
surfaces, obstacle shapes, speeds, and sensor mountings. Split by complete
runs/scenarios rather than randomly splitting near-identical frames. Retrain,
evaluate per-class recall and collision outcomes, then keep the independent
safety shield enabled.

## Native C test

    gcc -std=c11 -O2 -Wall -Wextra -Werror -pedantic -Ifirmware/tiva/include firmware/tiva/src/ai_inference.c firmware/tiva/src/ai_navigation.c tests/test_ai_navigation.c -lm -o /tmp/test_ai_navigation
    /tmp/test_ai_navigation

No TivaWare sensor backend or main.c integration is included yet. The current
module is a host-tested inference and decision layer, ready for a later
non-blocking sensor backend and main-loop integration.
