"""
camera_params.py
Read camera intrinsics inlined into pipeline_data/jubilee_paths.json by
`jubilee-twin setup-scene`. If that has not run yet, fall back to
jubilee_twin/defaults/camera_params.yaml parsed with a minimal reader
(Blender's bundled Python does not guarantee PyYAML).
"""

from __future__ import annotations

import json
from pathlib import Path


def _parse_yaml_minimal(text: str) -> dict:
    """Minimal single-level YAML parser for the camera_params.yaml schema."""
    root: dict = {}
    section: dict | None = None
    list_key: str | None = None
    for raw in text.splitlines():
        line = raw.rstrip()
        if not line or line.lstrip().startswith("#"):
            continue
        if not line[0].isspace():
            key = line.rstrip(":").strip()
            root[key] = {}
            section = root[key]
            list_key = None
            continue
        if section is None:
            continue
        stripped = line.strip()
        if stripped.startswith("- ") and list_key is not None:
            v = stripped[2:].strip()
            try:
                section[list_key].append(float(v))
            except ValueError:
                section[list_key].append(v)
            continue
        if ":" in stripped:
            k, _, v = stripped.partition(":")
            k = k.strip()
            v = v.strip()
            if v == "":
                section[k] = []
                list_key = k
            else:
                list_key = None
                try:
                    section[k] = int(v)
                except ValueError:
                    try:
                        section[k] = float(v)
                    except ValueError:
                        section[k] = v
    return root


def _load_fallback_yaml() -> dict:
    """Parse jubilee_twin/defaults/camera_params.yaml without PyYAML."""
    # This file is in blender_addon/jubilee_digital_twin/; twin root is 2 levels up.
    fallback = Path(__file__).parent.parent.parent / "jubilee_twin" / "defaults" / "camera_params.yaml"
    if not fallback.is_file():
        raise RuntimeError(f"Fallback camera_params.yaml not found at {fallback}")
    try:
        import yaml  # type: ignore
        data = yaml.safe_load(fallback.read_text()) or {}
    except ImportError:
        data = _parse_yaml_minimal(fallback.read_text())
    cam = dict(data.get("camera", {}))
    img = data.get("image", {})
    cam.setdefault("width", img.get("width", 1920))
    cam.setdefault("height", img.get("height", 1056))
    cam["_source"] = str(fallback)
    return cam


def load_camera_params(twin_root: Path) -> dict:
    """Return intrinsics dict from jubilee_paths.json; fall back to bundled YAML."""
    cache = twin_root / "pipeline_data" / "jubilee_paths.json"
    if cache.is_file():
        data = json.loads(cache.read_text())
        params = data.get("camera_params")
        if params:
            # Merge image dimensions stored at root of the yaml.
            img = data.get("image", {})
            params.setdefault("width", img.get("width", 1920))
            params.setdefault("height", img.get("height", 1056))
            params.setdefault("_source", cache.name)
            return dict(params)
    return _load_fallback_yaml()
