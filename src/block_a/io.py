from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

from .metrics import json_ready


def ensure_dir(path: str | Path) -> Path:
    output = Path(path)
    output.mkdir(parents=True, exist_ok=True)
    return output


def write_json(path: str | Path, data: dict[str, Any]) -> None:
    Path(path).write_text(json.dumps(json_ready(data), indent=2) + "\n")


def write_trace_csv(path: str | Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise ValueError("No trace rows to write")
    with Path(path).open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
