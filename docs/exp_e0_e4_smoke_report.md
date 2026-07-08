# Block A Smoke/Dev Report

## Run

Date: 2026-07-08

Commands:

```bash
python3 scripts/run_e0_smoke.py --seeds 10
python3 scripts/run_e2_mpc_ed.py --seeds 10
python3 scripts/run_e3_mpc_cbf_sweep.py --seeds 10
python3 scripts/run_e4_compare_ed_cbf.py --seeds 10 --gamma 0.08
```

Solver implementation:

```text
numpy_random_shooting_mpc
```

This v1 implementation is a lightweight reproducible MPC baseline. It does not yet depend on CasADi, do-mpc, or IPOPT, because those packages are not installed in the current environment. The experiment still follows the same ED-vs-CBF comparison logic from the reference papers: ED uses a Euclidean distance safety constraint, and CBF uses the discrete-time condition `h_{k+1} >= (1 - gamma) h_k`.

## Results

| Experiment | Method | Success rate | Collision rate | Mean min clearance | Mean solve time |
|---|---|---:|---:|---:|---:|
| E0 | Smoke tracking, no obstacle constraint | 1.0 | 0.0 | not used | 0.008 ms |
| E2 | MPC-ED | 0.9 | 0.0 | 0.049 m | 1.772 ms |
| E3 | MPC-CBF, gamma=1.0 | 0.9 | 0.0 | 0.051 m | 1.819 ms |
| E3 | MPC-CBF, gamma=0.15 | 0.9 | 0.0 | 0.485 m | 1.818 ms |
| E3 | MPC-CBF, gamma=0.08 | 0.9 | 0.0 | 0.812 m | 1.813 ms |
| E3 | MPC-CBF, gamma=0.04 | 0.9 | 0.0 | 1.045 m | 1.821 ms |
| E3 | MPC-CBF, gamma=0.02 | 0.9 | 0.0 | 1.139 m | 1.819 ms |
| E4 | ED matched-seed comparison | 0.9 | 0.0 | 0.049 m | 1.779 ms |
| E4 | CBF gamma=0.08 matched-seed comparison | 0.9 | 0.0 | 0.812 m | 1.826 ms |

## Interpretation

- E0 confirms the no-obstacle tracking loop runs successfully.
- E3 reproduces the expected qualitative CBF trend: smaller `gamma` increases minimum clearance.
- E4 shows the intended Block A comparison: CBF is more proactive than ED under the same matched-seed scenario, with similar solve time.
- Collision rate is already zero for ED in this small dev scenario, so the most useful Block A signal is proactive clearance rather than collision reduction.

## Next

- Run 50 seeds before using numbers in a paper figure.
- Add a harder crossing scenario if collision-rate separation is needed.
- Add a CasADi/do-mpc/IPOPT implementation path once dependencies are installed.
