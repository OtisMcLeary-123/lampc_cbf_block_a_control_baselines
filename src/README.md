# Source

This directory will contain the shared Block A implementation.

Modules:

```text
dynamics.py
scenario.py
metrics.py
controllers.py
runner.py
plots.py
```

The current v1 implementation uses a deterministic NumPy random-shooting MPC controller for reproducible micro-experiments. ED and CBF baselines share the same rollout optimizer and differ only in the safety constraint evaluation:

- ED: Euclidean distance constraint.
- CBF: discrete-time CBF condition `h_{k+1} >= (1 - gamma) h_k`.
