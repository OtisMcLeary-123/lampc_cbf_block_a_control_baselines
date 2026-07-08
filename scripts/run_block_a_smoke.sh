#!/usr/bin/env bash
set -euo pipefail

python3 scripts/run_e0_smoke.py --seeds 3
python3 scripts/run_e2_mpc_ed.py --seeds 3
python3 scripts/run_e3_mpc_cbf_sweep.py --seeds 3
python3 scripts/run_e4_compare_ed_cbf.py --seeds 3 --gamma 0.08
