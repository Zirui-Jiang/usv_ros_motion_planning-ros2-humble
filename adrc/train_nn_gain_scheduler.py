"""Train a compact MLP gain scheduler from the validated Level1 policy."""

import json
import math

import numpy as np
from sklearn.neural_network import MLPRegressor


def target_policy(features):
    error, rate, accel, adaptation = features.T
    h1 = np.tanh(2.2 * error - 0.55 * rate + 0.08 * accel)
    h2 = np.tanh(1.35 * rate + 0.20 * accel)
    h3 = np.tanh(0.60 * error + 0.45 * accel)
    kp = np.clip(0.90 + 0.65 * h1 + 0.42 * adaptation, 0.50, 2.00)
    kd = np.clip(0.22 + 0.30 * h2 + 0.05 * h3 + 0.20 * adaptation, 0.08, 0.80)
    ki = np.clip(0.001 + 0.016 * adaptation, 0.0, 0.020)
    return np.column_stack((kp, kd, ki))


def main():
    rng = np.random.default_rng(20260911)
    features = rng.uniform(0.0, 1.0, size=(24000, 4))
    targets = target_policy(features)
    output_min = np.array([0.50, 0.08, 0.0])
    output_span = np.array([1.50, 0.72, 0.020])
    normalized_targets = (targets - output_min) / output_span

    net = MLPRegressor(
        hidden_layer_sizes=(10,),
        activation="tanh",
        solver="lbfgs",
        alpha=1e-6,
        max_iter=1000,
        random_state=20260911,
    )
    net.fit(features, normalized_targets)
    prediction = net.predict(features)
    gain_error = np.abs((prediction - normalized_targets) * output_span)
    payload = {
        "weights_input_hidden": net.coefs_[0].T.tolist(),
        "bias_hidden": net.intercepts_[0].tolist(),
        "weights_hidden_output": net.coefs_[1].T.tolist(),
        "bias_output": net.intercepts_[1].tolist(),
        "max_abs_error": gain_error.max(axis=0).tolist(),
        "mean_abs_error": gain_error.mean(axis=0).tolist(),
    }
    print(json.dumps(payload, separators=(",", ":")))


if __name__ == "__main__":
    main()
 