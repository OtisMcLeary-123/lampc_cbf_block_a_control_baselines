#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from block_a.runner import run_experiment


def main() -> None:
    parser = argparse.ArgumentParser(description="Run E0 MPC smoke test without obstacle constraints.")
    parser.add_argument("--scenario", default=ROOT / "configs/scenario_point_mass_2d.json")
    parser.add_argument("--output", default=ROOT / "results/exp_e0")
    parser.add_argument("--seeds", type=int, default=3)
    args = parser.parse_args()
    run_experiment("E0", args.scenario, args.output, method="smoke", seeds=args.seeds, obstacle_enabled=False)
    print(f"Wrote E0 results to {args.output}")


if __name__ == "__main__":
    main()
