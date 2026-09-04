import os
import json
import numpy as np
from pathlib import Path
from jubilee_twin.paths import resolve, twin_dir
from jubilee_twin.log import get_logger
from jubilee_twin import machine_data
from jubilee_twin import trace as trace_mod
from jubilee_twin.trace import Section
from jubilee_twin.tool_plugins import (
    alias_index,
    discover_tool_assets,
    merge_plugin_fallback,
    normalize,
)

logger = get_logger(__name__)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_STATE = Path(__file__).parent.parent / "defaults" / "machine_state.json"


def _science_jubilee_resolver():
    """science_jubilee owns the live/snapshot chain; the twin runs without it."""
    try:
        from science_jubilee.machine_state import resolve as _resolve
        return _resolve
    except ImportError:
        return None


def _env_hardware_address() -> str | None:
    """Address from .env.hardware, via the path the driver cached for Blender."""
    resolver = _science_jubilee_resolver()
    if resolver is None:
        return None
    from science_jubilee.machine_state import read_env_address

    try:
        return read_env_address()
    except Exception:
        pass
    # Blender addon context: jubilee_dir is not importable, but its path was cached.
    try:
        paths_json = twin_dir() / "pipeline_data" / "jubilee_paths.json"
        env_path = json.loads(paths_json.read_text()).get("env_hardware")
        return read_env_address(Path(env_path)) if env_path else None
    except Exception:
        return None


def _load_machine_state(machine_state_path: str | None = None,
                        trace: Section | None = None) -> tuple[dict, str]:
    """Return (state_dict, source_label) using a 3-step fallback.

    1. science_jubilee's shared chain: live Duet → snapshot address → snapshot
    2. Installed tool plugins (names/parks/offsets from their own Duet macros)
    3. Built-in defaults (standard 4-tool Jubilee park positions)
    """
    trace = trace or trace_mod.session().section("Machine state")

    resolver = _science_jubilee_resolver()
    if resolver is None:
        trace.failed("science_jubilee", "not installed — cannot reach a machine")
    else:
        saved_path = Path(machine_state_path) if machine_state_path else None
        try:
            state, source = resolver(
                address=None if machine_state_path else _env_hardware_address(),
                saved_path=saved_path,
                allow_live=not machine_state_path,
            )
        except Exception as exc:
            trace.failed("science_jubilee", str(exc))
        else:
            if source != "empty machine":
                trace.ok(source.split(" ")[0], source)
                return state, source
            trace.failed("live Duet / saved snapshot", "no machine and no snapshot")

    # Tool plugins, then built-in defaults for whatever is left
    logger.warning("No live machine and no saved state found — falling back to plugins/defaults")
    state = {"tools": {}, "tool_offsets": {}, "tool_parks": {}}
    applied = merge_plugin_fallback(state)
    if applied:
        trace.partial("installed tool plugins", ", ".join(applied))
    else:
        trace.failed("installed tool plugins", "no plugin declares a tool number")

    with open(_DEFAULT_STATE) as f:
        defaults = json.load(f)
    for section in ("tools", "tool_offsets", "tool_parks"):
        for slot, value in defaults.get(section, {}).items():
            state[section].setdefault(slot, value)
    for key, value in defaults.items():
        state.setdefault(key, value)
    trace.ok("bundled defaults", _DEFAULT_STATE.name)

    if applied:
        return state, f"plugins ({', '.join(applied)}) + defaults ({_DEFAULT_STATE.name})"
    return state, f"defaults ({_DEFAULT_STATE.name})"


def _backfill_parks(state: dict, trace) -> list[str]:
    """Give slots the standard Jubilee park position when the source had none.

    A snapshot can carry tool names and offsets but an empty ``tool_parks``, and
    an all-zero park would stack every tool at the origin.
    """
    with open(_DEFAULT_STATE) as f:
        defaults = json.load(f).get("tool_parks", {})

    parks = state.setdefault("tool_parks", {})
    filled: list[str] = []
    for slot, value in defaults.items():
        if not any(parks.get(slot) or []):
            parks[slot] = value
            filled.append(slot)
    if filled:
        trace.partial("bundled park positions", f"slots {', '.join(filled)} had none")
    return filled


def run(output_dir: str = None, machine_state_path: str = None) -> str:
    """Build pipeline_data/machine.json. Returns the output path.

    Discovery order: live Duet query → gcode_logs/machine_state.json → tool plugins
    → built-in defaults. Pass machine_state_path= to skip discovery and use a
    specific file. Model paths for each slot are resolved from the installed tool
    plugins and written into the same records.
    """
    trace = trace_mod.session(output_dir).section("Machine state", reset=True)
    state, source = _load_machine_state(machine_state_path, trace)

    assets = discover_tool_assets()

    # A live/saved state can still have unnamed slots; plugins fill only those.
    if not source.startswith("plugins"):
        applied = merge_plugin_fallback(state, assets)
        if applied:
            source = f"{source} + plugins ({', '.join(applied)})"

    if _backfill_parks(state, trace):
        source = f"{source} + default parks"

    tool_names: dict = {}
    tool_offsets: dict = {}
    tool_parks: dict = {}

    for raw_idx, info in state.get("tools", {}).items():
        try:
            idx = int(raw_idx)
            tool_names[idx] = (
                info.get("name", f"Unnamed_Tool_{idx}")
                if isinstance(info, dict)
                else f"Unnamed_Tool_{idx}"
            )
        except (ValueError, TypeError):
            pass

    for raw_idx, off_list in state.get("tool_offsets", {}).items():
        try:
            idx = int(raw_idx)
            if isinstance(off_list, (list, tuple)) and len(off_list) >= 3:
                tool_offsets[idx] = np.array([float(v) for v in off_list[:3]])
        except (ValueError, TypeError):
            pass

    for raw_idx, pos_list in state.get("tool_parks", {}).items():
        try:
            idx = int(raw_idx)
            if isinstance(pos_list, (list, tuple)) and len(pos_list) >= 3:
                tool_parks[idx] = np.array([float(v) for v in pos_list[:3]])
        except (ValueError, TypeError):
            pass

    out_dir = output_dir or os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "pipeline_data"))
    os.makedirs(out_dir, exist_ok=True)

    recap = trace_mod.session(out_dir)
    models = recap.section("Tool models", reset=True)
    index = alias_index(assets)
    default_park_post = twin_dir() / "Tool Post STL" / "park_post_47.blend"

    tools = []
    for i in range(4):
        name = tool_names.get(i, f"Unassigned_Tool_{i}")
        park = tool_parks.get(i, np.array([0.0, 0.0, 0.0]))
        offset = tool_offsets.get(i, np.array([0.0, 0.0, 0.0]))

        info = assets.get(index.get(normalize(name), "")) or {}
        blend = info.get("blend") or None
        if blend:
            logger.info("Tool %d (%s) ← %s", i, name, blend)
            models.ok(f"tool {i} · {name}", Path(blend).name)
        elif info:
            models.partial(f"tool {i} · {name}", "plugin matched but ships no .blend")
        else:
            models.failed(f"tool {i} · {name}", "no tool plugin matches this name")

        tools.append({
            "id": i,
            "name": name,
            "park": [float(v) for v in park[:3]],
            "offsets": [float(v) for v in offset[:3]],
            "blend": blend,
            "park_post": info.get("park_post_blend") or (str(default_park_post) if blend else None),
        })

    positions = state.get("positions", {})
    head = {k: positions[k] for k in ("X", "Y", "Z") if k in positions} or None

    output_path = machine_data.write(out_dir, {
        "source": source,
        "head_position": head,
        "active_tool": state.get("active_tool", -1),
        "tools": tools,
    })

    recap.result(
        f"machine.json  —  source: {source}",
        Path(output_path).read_text(),
        output_path,
    )
    trace_path = trace_mod.flush(out_dir)

    logger.info("Machine state: %s", source)
    logger.warning("machine.json → %s", output_path)
    if trace_path:
        logger.warning("trace recap  → %s", trace_path)
    if head:
        logger.warning("Head position  X=%.1f  Y=%.1f  Z=%.1f",
                       head.get("X", 0.0), head.get("Y", 0.0), head.get("Z", 0.0))
    return str(output_path)