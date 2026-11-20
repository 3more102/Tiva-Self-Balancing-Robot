"""Generate reproducible synthetic obstacle-navigation examples.

Labels are produced by a conservative expert policy. This is behavior-cloning
training data, not evidence of real-world navigation performance.
"""
from __future__ import annotations

import numpy as np

ACTION_NAMES = ("FORWARD", "TURN_LEFT", "TURN_RIGHT", "STOP", "REASSESS")
MAX_DISTANCE_MM = 2000.0
MAX_CLOSING_MPS = 2.0
MAX_YAW_DPS = 180.0


def expert_action(front_mm: float, left_mm: float, right_mm: float,
                  closing_mps: float, yaw_rate_dps: float) -> int:
    """Conservative rule-based teacher used to label synthetic scenarios."""
    left_clear = left_mm >= 500.0
    right_clear = right_mm >= 500.0

    def best_turn(min_clearance: float) -> int:
        candidates = []
        if left_mm >= min_clearance:
            candidates.append((left_mm, 1))  # TURN_LEFT
        if right_mm >= min_clearance:
            candidates.append((right_mm, 2))  # TURN_RIGHT
        if not candidates:
            return 3  # STOP
        if len(candidates) == 1:
            return candidates[0][1]
        if abs(left_mm - right_mm) <= 100.0:
            if yaw_rate_dps > 25.0:
                return 1
            if yaw_rate_dps < -25.0:
                return 2
        return 1 if left_mm > right_mm else 2

    if front_mm <= 350.0:
        if left_clear or right_clear:
            return best_turn(500.0)
        return 3

    if front_mm < 500.0:
        if closing_mps >= 0.75:
            return best_turn(600.0)
        return 4  # REASSESS

    if front_mm < 800.0 and closing_mps >= 1.30:
        return best_turn(700.0)

    if front_mm < 1000.0 and abs(yaw_rate_dps) >= 140.0:
        return 4

    return 0  # FORWARD


def generate_dataset(samples: int, seed: int = 7) -> tuple[np.ndarray, np.ndarray]:
    """Return raw sensor features [N,5] and teacher action labels [N]."""
    if samples < 1:
        raise ValueError("samples must be positive")
    rng = np.random.default_rng(seed)
    x = np.empty((samples, 5), dtype=np.float32)
    kinds = rng.choice(6, size=samples, p=[0.18, 0.20, 0.22, 0.16, 0.14, 0.10])

    for i, kind in enumerate(kinds):
        if kind == 0:
            front = rng.uniform(650, 2000)
            left, right = rng.uniform(250, 2000, size=2)
            closing = rng.uniform(-0.8, 1.8)
        elif kind == 1:
            front = rng.uniform(60, 350)
            left, right = rng.uniform(100, 1800, size=2)
            closing = rng.uniform(-0.4, 2.2)
        elif kind == 2:
            front = rng.uniform(300, 650)
            left, right = rng.uniform(100, 1800, size=2)
            closing = rng.uniform(-0.4, 2.2)
        elif kind == 3:
            front = rng.uniform(500, 1100)
            left, right = rng.uniform(150, 1800, size=2)
            closing = rng.uniform(0.8, 2.5)
        elif kind == 4:
            front = rng.uniform(80, 900)
            left, right = rng.uniform(80, 650, size=2)
            closing = rng.uniform(-0.5, 2.0)
        else:
            front = rng.uniform(80, 900)
            left, right = rng.uniform(100, 1800, size=2)
            if rng.random() < 0.5:
                left = rng.uniform(80, 450)
                right = rng.uniform(550, 1800)
            else:
                right = rng.uniform(80, 450)
                left = rng.uniform(550, 1800)
            closing = rng.uniform(-0.5, 2.3)
        yaw = rng.uniform(-180, 180)
        x[i] = (front, left, right, closing, yaw)

    y = np.fromiter((expert_action(*row) for row in x),
                    dtype=np.int64, count=samples)
    return x, y


def normalize_features(raw: np.ndarray) -> np.ndarray:
    """Convert mm, m/s and deg/s to the documented bounded model inputs."""
    raw = np.asarray(raw, dtype=np.float32)
    if raw.ndim != 2 or raw.shape[1] != 5:
        raise ValueError("expected [N,5] raw feature matrix")
    out = np.empty_like(raw)
    out[:, 0:3] = np.clip(raw[:, 0:3] / MAX_DISTANCE_MM, 0.0, 1.0)
    out[:, 3] = np.clip(raw[:, 3] / MAX_CLOSING_MPS, -1.0, 1.0)
    out[:, 4] = np.clip(raw[:, 4] / MAX_YAW_DPS, -1.0, 1.0)
    return out
