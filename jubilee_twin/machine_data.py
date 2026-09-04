"""Read/write pipeline_data/machine.json — the single machine handover file.

Stdlib only: this module is imported both from the CLI (your venv) and from
Blender's bundled interpreter.

Schema::

    {
      "source": "live (192.168.1.2)",
      "head_position": {"X": 0.0, "Y": 0.0, "Z": 0.0} | null,
      "active_tool": -1,
      "tools": [
        {"id": 0, "name": "maag_pen",
         "park": [x, y, z], "offsets": [x, y, z],
         "blend": "...", "park_post": "..."}
      ]
    }
"""

from __future__ import annotations

import json
from pathlib import Path

FILENAME = "machine.json"


def path_for(pipeline_data_dir) -> Path:
    return Path(pipeline_data_dir) / FILENAME


def write(pipeline_data_dir, data: dict) -> Path:
    out = path_for(pipeline_data_dir)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, indent=2))
    return out


def load(pipeline_data_dir) -> dict:
    """Read machine.json; return an empty skeleton if it does not exist."""
    path = path_for(pipeline_data_dir)
    if not path.is_file():
        return {"source": None, "head_position": None, "active_tool": -1, "tools": []}
    return json.loads(path.read_text())


def tool_by_id(data: dict, tool_id) -> dict | None:
    try:
        wanted = int(float(tool_id))
    except (TypeError, ValueError):
        return None
    for tool in data.get("tools", []):
        if int(tool.get("id", -1)) == wanted:
            return tool
    return None


def nearest_tool_id(data: dict, x: float, y: float, max_dist: float = 30.0) -> int | None:
    """Return the id of the tool parked nearest (x, y), or None if none is close."""
    best_id = None
    best_dist = float("inf")
    for tool in data.get("tools", []):
        park = tool.get("park") or [0.0, 0.0, 0.0]
        dist = ((float(park[0]) - x) ** 2 + (float(park[1]) - y) ** 2) ** 0.5
        if dist < best_dist:
            best_dist = dist
            best_id = int(tool.get("id", -1))
    return best_id if best_dist < max_dist else None
