"""Discovery of tool-plugin twin assets and offline machine defaults.

Tool plugins register ``science_jubilee.tools.twin_assets`` entry points that
return the package's ``twin_assets/`` directory. Everything the twin needs is
derived from that directory (and the plugin repo around it); an optional
``twin_assets/twin.json`` overrides any derived field.
"""

from __future__ import annotations

import json
import os
import re
from pathlib import Path

from jubilee_twin.log import get_logger

logger = get_logger(__name__)

ASSETS_GROUP = "science_jubilee.tools.twin_assets"
TOOLS_GROUP = "science_jubilee.tools"
ENV_EXTRA_ASSETS = "JUBILEE_TWIN_TOOL_ASSETS"

_PLACEHOLDER_NAME = re.compile(r"^(unnamed|unassigned)_tool_\d+$", re.IGNORECASE)
# Blends that are shared scenery rather than the tool body itself.
_NON_TOOL_BLEND = re.compile(r"park_post|wedge_plate|working", re.IGNORECASE)


def parse_park_position(content: str) -> list[float]:
    """Extract [X, Y, Z] from a tpost{n}.g file by reading G53 lines."""
    pos = [0.0, 0.0, 0.0]
    for line in content.splitlines():
        stripped = line.strip()
        if not stripped.upper().startswith("G53"):
            continue
        for letter, idx in (("X", 0), ("Y", 1), ("Z", 2)):
            m = re.search(rf'(?<![A-Z]){letter}(-?[\d.]+)', stripped, re.IGNORECASE)
            if m:
                pos[idx] = float(m.group(1))
    return pos


def normalize(name: str) -> str:
    """Collapse a tool name to a comparable key (case/separator insensitive)."""
    return re.sub(r"[^a-z0-9]", "", str(name).lower())


def is_placeholder_name(name: str) -> bool:
    return not name or bool(_PLACEHOLDER_NAME.match(str(name).strip()))


def _find_blend(assets_dir: Path, tool_key: str) -> Path | None:
    """Locate the tool body blend. Convention: name it ``tool.blend``."""
    for stem in ("tool", tool_key):
        hits = [p for p in assets_dir.rglob("*.blend") if p.stem == stem]
        if hits:
            return hits[0]
    candidates = [p for p in assets_dir.rglob("*.blend") if not _NON_TOOL_BLEND.search(p.name)]
    if len(candidates) == 1:
        return candidates[0]
    if candidates:
        logger.info(
            "%s: %d .blend files in %s and none named tool.blend — rename the tool body "
            "to tool.blend (or set \"blend\" in twin_assets/twin.json)",
            tool_key, len(candidates), assets_dir,
        )
    return None


def _find_park_post(repo_root: Path) -> Path | None:
    for base in (repo_root / "twin_assets", repo_root / "templates", repo_root):
        if not base.is_dir():
            continue
        hits = sorted(base.glob("park_post*.blend"))
        if hits:
            return hits[0]
    return None


def _gcode_sources(repo_root: Path, prefix: str = "tpost") -> list[Path]:
    """The tool's own Duet macros. ``.g.template`` files are ignored — they still
    hold ``{{placeholder}}`` values rather than coordinates."""
    out: list[Path] = []
    for base in (repo_root / "templates", repo_root):
        if base.is_dir():
            out.extend(sorted(base.glob(f"{prefix}*.g")))
    return out


def _derive_tool_number(gcode: Path | None) -> int | None:
    """The slot number is the N in tpost{N}.g."""
    if gcode is None:
        return None
    m = re.search(r"tpost(\d+)\.g", gcode.name)
    return int(m.group(1)) if m else None


def _derive_offsets(repo_root: Path) -> list[float] | None:
    """Read G10 offsets from the tool's own macros only."""
    sources: list[Path] = []
    for prefix in ("tpre", "tpost", "tfree"):
        sources.extend(_gcode_sources(repo_root, prefix))
    for path in sources:
        try:
            text = path.read_text()
        except OSError:
            continue
        m = re.search(r"G10\s+P\d+((?:\s+[XYZ]-?[\d.]+)+)", text, re.IGNORECASE)
        if not m:
            continue
        off = [0.0, 0.0, 0.0]
        for letter, idx in (("X", 0), ("Y", 1), ("Z", 2)):
            v = re.search(rf'(?<![A-Z]){letter}(-?[\d.]+)', m.group(1), re.IGNORECASE)
            if v:
                off[idx] = float(v.group(1))
        return off
    return None


def _derive_fallback(repo_root: Path) -> dict | None:
    """Offline park/offset data, read only from the plugin's own Duet macros.

    A freshly created plugin ships zeroed macros, so its park is [0, 0, 0] until
    it is calibrated. Returns None when the macros are absent entirely.
    """
    for src in _gcode_sources(repo_root):
        tool_number = _derive_tool_number(src)
        if tool_number is None:
            continue
        try:
            park = parse_park_position(src.read_text())
        except OSError:
            continue
        fallback = {"tool_number": tool_number, "park": park, "source": str(src)}
        offsets = _derive_offsets(repo_root)
        if offsets:
            fallback["offsets"] = offsets
        return fallback
    return None


def _class_names_by_key() -> dict[str, str]:
    """Map tool_key → implementation class name, without importing the plugin."""
    from importlib.metadata import entry_points

    out: dict[str, str] = {}
    try:
        for ep in entry_points(group=TOOLS_GROUP):
            out[ep.name] = ep.value.rsplit(":", 1)[-1]
    except Exception:
        pass
    return out


def _describe(tool_key: str, assets_dir: Path, source: str, class_names: dict[str, str]) -> dict:
    repo_root = assets_dir.parent
    blend = _find_blend(assets_dir, tool_key)
    entry: dict = {
        "tool_key": tool_key,
        "display_name": tool_key,
        "assets_dir": str(assets_dir),
        "blend": str(blend) if blend else None,
        "park_post_blend": None,
        "aliases": [],
        "fallback": _derive_fallback(repo_root),
        "source": source,
    }
    park_post = _find_park_post(repo_root)
    if park_post:
        entry["park_post_blend"] = str(park_post)

    manifest = assets_dir / "twin.json"
    if manifest.is_file():
        try:
            override = json.loads(manifest.read_text())
        except Exception as exc:
            logger.warning("Ignoring malformed %s: %s", manifest, exc)
        else:
            override = {k: v for k, v in override.items() if not k.startswith("_")}
            for key in ("blend", "park_post_blend"):
                rel = override.pop(key, None)
                if rel:
                    override[key] = str((assets_dir / rel).resolve())
            # Merge the fallback block key-by-key so a partly-filled manifest
            # does not wipe out values derived from the plugin's own macros.
            manual_fallback = override.pop("fallback", None)
            if isinstance(manual_fallback, dict):
                merged = dict(entry["fallback"] or {})
                merged.update({k: v for k, v in manual_fallback.items() if v is not None})
                entry["fallback"] = merged or None
            entry.update({k: v for k, v in override.items() if v is not None})
            entry["source"] = f"{source}+twin.json"

    aliases = set(entry.get("aliases") or [])
    aliases.update({tool_key, entry["display_name"]})
    if tool_key in class_names:
        aliases.add(class_names[tool_key])
    entry["aliases"] = sorted(a for a in aliases if a)
    return entry


def discover_tool_assets() -> dict[str, dict]:
    """Return {tool_key: asset_info} for every discoverable tool plugin."""
    from importlib.metadata import entry_points
    from jubilee_twin.trace import session

    sec = session().section("Tool plugins", reset=True)
    class_names = _class_names_by_key()
    found: dict[str, dict] = {}

    eps = list(entry_points(group=ASSETS_GROUP))
    if not eps:
        sec.failed("entry points", f"no {ASSETS_GROUP} registered")
    for ep in eps:
        try:
            assets_dir = Path(ep.load()())
        except Exception as exc:
            logger.warning("Could not resolve %s/%s: %s", ASSETS_GROUP, ep.name, exc)
            sec.failed(ep.name, f"entry point failed: {exc}")
            continue
        if not assets_dir.is_dir():
            logger.warning("%s: twin_assets dir does not exist: %s", ep.name, assets_dir)
            sec.failed(ep.name, f"twin_assets missing: {assets_dir}")
            continue
        found[ep.name] = _describe(ep.name, assets_dir, "entry_point", class_names)
        _trace_asset(sec, ep.name, found[ep.name])

    for raw in os.environ.get(ENV_EXTRA_ASSETS, "").split(os.pathsep):
        if not raw.strip():
            continue
        assets_dir = Path(raw.strip()).expanduser()
        if not assets_dir.is_dir():
            logger.warning("%s entry is not a directory: %s", ENV_EXTRA_ASSETS, assets_dir)
            sec.failed(f"${ENV_EXTRA_ASSETS}", f"not a directory: {assets_dir}")
            continue
        key = assets_dir.parent.name.replace("-", "_")
        manifest = assets_dir / "twin.json"
        if manifest.is_file():
            try:
                key = json.loads(manifest.read_text()).get("tool_key", key)
            except Exception:
                pass
        found[key] = _describe(key, assets_dir, "env", class_names)
        _trace_asset(sec, f"{key} (env)", found[key])

    return found


def _trace_asset(sec, label: str, info: dict) -> None:
    fallback = info.get("fallback") or {}
    if not info.get("blend"):
        sec.partial(label, "no tool.blend in twin_assets/")
        return
    if fallback.get("park"):
        sec.ok(label, f"{Path(info['blend']).name} · park from {Path(fallback['source']).name}")
    else:
        sec.partial(
            label,
            f"{Path(info['blend']).name} · not calibrated: no tpost<N>.g — "
            "run calibration/SetToolParkingPositions.ipynb",
        )


def alias_index(assets: dict[str, dict]) -> dict[str, str]:
    """Map every normalized alias to its tool_key."""
    index: dict[str, str] = {}
    for key, info in assets.items():
        for alias in info.get("aliases", []):
            index.setdefault(normalize(alias), key)
    return index


def merge_plugin_fallback(state: dict, assets: dict[str, dict] | None = None) -> list[str]:
    """Fill placeholder slots in a machine_state dict from plugin metadata.

    Only empty or placeholder fields are touched, so live Duet data always wins.
    Returns the tool keys that contributed.
    """
    if assets is None:
        assets = discover_tool_assets()

    tools = state.setdefault("tools", {})
    parks = state.setdefault("tool_parks", {})
    offsets = state.setdefault("tool_offsets", {})
    applied: list[str] = []

    for key, info in assets.items():
        fallback = info.get("fallback") or {}
        idx = fallback.get("tool_number")
        if idx is None:
            continue
        slot = str(idx)
        used = False

        current = tools.get(slot)
        current_name = current.get("name") if isinstance(current, dict) else current
        if is_placeholder_name(current_name):
            tools[slot] = {"name": info.get("display_name") or key}
            used = True

        if fallback.get("park") and not any(parks.get(slot) or []):
            parks[slot] = list(fallback["park"])
            used = True

        if fallback.get("offsets") and not any(offsets.get(slot) or []):
            offsets[slot] = list(fallback["offsets"])
            used = True

        if used:
            applied.append(key)

    return applied
