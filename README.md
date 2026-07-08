# Block A: Control Baselines

## Goal

This repository contains the first reproducible block of the LaMPC-CBF reference micro-experiments.

Block A focuses on control baselines before any language/LLM interface is added:

```text
E0 -> E2 -> E3 -> E4
```

The goal is to establish a reliable, measurable MPC-CBF baseline for a small dynamic-obstacle navigation scenario.

## Research Role

Block A answers the control foundation question:

```text
Does MPC-CBF provide safer and more proactive obstacle avoidance than
Euclidean-distance constrained MPC under the same scenario, seeds, and metrics?
```

This block is required before testing language interfaces in later repos:

- Block B: strong non-LLM adaptive safety baselines
- Block C: language interface comparison
- Block D: closest paper baselines
- Block E: evaluation and paper figures

## Experiments

| ID | Method | Purpose | Expected output |
|---|---|---|---|
| E0 | MPC smoke test | Verify solver stability without obstacles | Tracking error and solve-time report |
| E2 | MPC-ED | Euclidean distance constrained MPC baseline | Collision/min-distance baseline |
| E3 | MPC-CBF | Discrete-time CBF gamma sweep | Safety-performance gamma curve |
| E4 | ED vs CBF | Direct comparison under identical scenario | Evidence that CBF is more proactive than ED |

## Shared Scenario

```text
Robot: point-mass 2D
Task: move from start to target
Obstacle: dynamic circle crossing the direct path
Seeds: 10 for smoke/dev runs, 50 for paper-level results
```

Default metrics:

```text
success_rate
collision_rate
min_obstacle_distance_mean
min_obstacle_distance_std
path_length
completion_time
mean_solve_time_ms
solver_failure_rate
```

## Repository Structure

```text
configs/   scenario and experiment configs
docs/      experiment protocol, reports, and log
results/   generated summaries, traces, and plots
scripts/   runner and plotting entrypoints
src/       shared implementation
```

## Initial Acceptance Criteria

Block A is ready for GitHub/public baseline use when:

- E0, E2, E3, and E4 can run from scripted commands.
- Each experiment writes a config-linked `summary.json`.
- Each experiment records seed, method, scenario, and solver settings.
- E4 produces trajectory and distance-to-obstacle plots.
- Results can be reproduced without API keys or external LLM services.

## Next Implementation Steps

1. Implement the 2D point-mass dynamics and shared scenario loader.
2. Implement E0 solver smoke test.
3. Implement E2 MPC-ED baseline.
4. Implement E3 MPC-CBF gamma sweep.
5. Implement E4 ED-vs-CBF comparison script and report.
