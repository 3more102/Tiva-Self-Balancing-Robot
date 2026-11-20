"""Train the GyroGuard TinyML MLP and export native-C weights."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

try:
    from .generate_dataset import ACTION_NAMES, generate_dataset, normalize_features
except ImportError:
    from generate_dataset import ACTION_NAMES, generate_dataset, normalize_features

INPUTS, HIDDEN1, HIDDEN2, ACTIONS = 5, 12, 8, 5


def softmax(logits: np.ndarray) -> np.ndarray:
    shifted = logits - np.max(logits, axis=1, keepdims=True)
    exp = np.exp(shifted)
    return exp / np.sum(exp, axis=1, keepdims=True)


def forward(x: np.ndarray, p: dict[str, np.ndarray]):
    z1 = x @ p["w1"].T + p["b1"]
    h1 = np.maximum(z1, 0.0)
    z2 = h1 @ p["w2"].T + p["b2"]
    h2 = np.maximum(z2, 0.0)
    logits = h2 @ p["w3"].T + p["b3"]
    return logits, (x, z1, h1, z2, h2)


def init_parameters(seed: int) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    return {
        "w1": (rng.standard_normal((HIDDEN1, INPUTS)) *
               np.sqrt(2.0 / INPUTS)).astype(np.float32),
        "b1": np.zeros(HIDDEN1, dtype=np.float32),
        "w2": (rng.standard_normal((HIDDEN2, HIDDEN1)) *
               np.sqrt(2.0 / HIDDEN1)).astype(np.float32),
        "b2": np.zeros(HIDDEN2, dtype=np.float32),
        "w3": (rng.standard_normal((ACTIONS, HIDDEN2)) *
               np.sqrt(1.0 / HIDDEN2)).astype(np.float32),
        "b3": np.zeros(ACTIONS, dtype=np.float32),
    }


def train(samples: int = 50000, epochs: int = 120, seed: int = 23):
    raw_train, y_train = generate_dataset(samples, seed)
    raw_val, y_val = generate_dataset(max(5000, samples // 5), seed + 1)
    x_train = normalize_features(raw_train)
    x_val = normalize_features(raw_val)
    params = init_parameters(seed + 2)

    counts = np.bincount(y_train, minlength=ACTIONS).astype(np.float32)
    class_weights = np.sqrt(counts.sum() /
                            (ACTIONS * np.maximum(counts, 1.0)))
    class_weights /= class_weights.mean()

    m = {k: np.zeros_like(v) for k, v in params.items()}
    v = {k: np.zeros_like(val) for k, val in params.items()}
    beta1, beta2, eps = 0.9, 0.999, 1e-8
    learning_rate, batch_size, step = 0.008, 128, 0
    rng = np.random.default_rng(seed + 3)

    for _epoch in range(epochs):
        order = rng.permutation(len(x_train))
        for start in range(0, len(order), batch_size):
            idx = order[start:start + batch_size]
            xb, yb = x_train[idx], y_train[idx]
            logits, cache = forward(xb, params)
            probs = softmax(logits)
            n = len(xb)
            weights = class_weights[yb]
            grad_logits = probs.copy()
            grad_logits[np.arange(n), yb] -= 1.0
            grad_logits *= (weights / n)[:, None]

            x0, z1, h1, z2, h2 = cache
            grads = {}
            grads["w3"] = grad_logits.T @ h2
            grads["b3"] = grad_logits.sum(axis=0)
            grad_h2 = grad_logits @ params["w3"]
            grad_z2 = grad_h2 * (z2 > 0.0)
            grads["w2"] = grad_z2.T @ h1
            grads["b2"] = grad_z2.sum(axis=0)
            grad_h1 = grad_z2 @ params["w2"]
            grad_z1 = grad_h1 * (z1 > 0.0)
            grads["w1"] = grad_z1.T @ x0
            grads["b1"] = grad_z1.sum(axis=0)

            step += 1
            for key in params:
                m[key] = beta1 * m[key] + (1.0 - beta1) * grads[key]
                v[key] = beta2 * v[key] + (1.0 - beta2) * (grads[key] ** 2)
                m_hat = m[key] / (1.0 - beta1 ** step)
                v_hat = v[key] / (1.0 - beta2 ** step)
                params[key] -= learning_rate * m_hat / (np.sqrt(v_hat) + eps)

    val_logits, _ = forward(x_val, params)
    val_probs = softmax(val_logits)
    predicted = np.argmax(val_probs, axis=1)
    accuracy = float(np.mean(predicted == y_val))
    per_class = {
        ACTION_NAMES[i]: float(np.mean(predicted[y_val == i] == i))
        if np.any(y_val == i) else 0.0
        for i in range(ACTIONS)
    }
    metrics = {
        "model": "GyroGuard TinyML MLP",
        "architecture": [INPUTS, HIDDEN1, HIDDEN2, ACTIONS],
        "training_samples": int(samples),
        "validation_samples": int(len(y_val)),
        "epochs": int(epochs),
        "seed": int(seed),
        "validation_accuracy": accuracy,
        "per_class_recall": per_class,
        "class_names": list(ACTION_NAMES),
        "data_origin": "SYNTHETIC_EXPERT_POLICY_BEHAVIOR_CLONING_NOT_ROBOT_MEASUREMENTS",
        "parameter_count": int(sum(a.size for a in params.values())),
        "weight_bytes_float32": int(sum(a.size for a in params.values()) * 4),
    }
    return params, metrics


def c_array(name: str, value: np.ndarray) -> str:
    rows = value if value.ndim > 1 else [value]
    lines = []
    for row in rows:
        def literal(item: float) -> str:
            token = f"{float(item):.9g}"
            if "." not in token and "e" not in token.lower():
                token += ".0"
            return token + "f"
        lines.append("    {" + ", ".join(literal(v) for v in row) + "}")
    if value.ndim == 1:
        return f"static const float {name}[{value.shape[0]}] = " + lines[0] + ";\n"
    return (f"static const float {name}[{value.shape[0]}][{value.shape[1]}] = {{\n"
            + ",\n".join(lines) + "\n};\n")


def export_header(params: dict[str, np.ndarray], metrics: dict, output: Path) -> None:
    header = [
        "/* Generated by ai/train_model.py; do not edit by hand.",
        " * Training provenance: synthetic expert-policy behavior cloning.",
        " * This is not a model trained on measured robot runs.",
        f" * Validation accuracy on held-out synthetic scenarios: {metrics['validation_accuracy']:.4f}",
        " */",
        "#ifndef GYROGUARD_AI_MODEL_WEIGHTS_H",
        "#define GYROGUARD_AI_MODEL_WEIGHTS_H",
        "#define AI_MODEL_INPUTS 5",
        "#define AI_MODEL_HIDDEN1 12",
        "#define AI_MODEL_HIDDEN2 8",
        "#define AI_MODEL_ACTIONS 5",
        "#define AI_MODEL_VERSION 1",
        "",
        c_array("ai_w1", params["w1"]),
        c_array("ai_b1", params["b1"]),
        c_array("ai_w2", params["w2"]),
        c_array("ai_b2", params["b2"]),
        c_array("ai_w3", params["w3"]),
        c_array("ai_b3", params["b3"]),
        "#endif /* GYROGUARD_AI_MODEL_WEIGHTS_H */",
        "",
    ]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(header), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=50000)
    parser.add_argument("--epochs", type=int, default=120)
    parser.add_argument("--seed", type=int, default=23)
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parents[1] /
                        "firmware/tiva/include/ai_model_weights.h")
    parser.add_argument("--metrics", type=Path, default=None)
    args = parser.parse_args()

    params, metrics = train(args.samples, args.epochs, args.seed)
    export_header(params, metrics, args.output)
    metrics_path = args.metrics or args.output.with_suffix(".metrics.json")
    metrics_path.write_text(json.dumps(metrics, indent=2) + "\n",
                            encoding="utf-8")
    print(json.dumps(metrics, indent=2))
    return 0 if metrics["validation_accuracy"] >= 0.85 else 2


if __name__ == "__main__":
    raise SystemExit(main())
