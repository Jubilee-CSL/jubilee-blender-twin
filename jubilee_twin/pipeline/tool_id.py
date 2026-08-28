import os
import csv
import json
import re
import warnings
import numpy as np
from pathlib import Path
from jubilee_twin.paths import resolve, twin_dir
from jubilee_twin.log import get_logger

logger = get_logger(__name__)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_STATE = Path(__file__).parent.parent / "defaults" / "machine_state.json"


def _parse_park_position(content: str) -> list:
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


def _try_live_query(address: str, timeout: float = 3.0) -> dict | None:
    """Fetch a fresh machine_state dict from the Duet via HTTP. Returns None on failure."""
    try:
        import urllib.request

        def post(cmd: str) -> str:
            req = urllib.request.Request(
                f"http://{address}/machine/code",
                data=cmd.encode(),
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return r.read().decode().strip()

        def download_sys(filename: str) -> str:
            for url in (
                f"http://{address}/machine/file/0:/sys/{filename}",
                f"http://{address}/rr_download?name=0:/sys/{filename}",
            ):
                try:
                    with urllib.request.urlopen(url, timeout=timeout) as r:
                        if r.status == 200:
                            return r.read().decode()
                except Exception:
                    continue
            raise OSError(f"Could not download {filename}")

        state: dict = {"transport": "HTTPTransport", "address": address}

        reply = post("M114")
        positions = {}
        for part in reply.split():
            letter, sep, val = part.partition(":")
            if sep and letter.upper() in ("X", "Y", "Z", "U"):
                try:
                    positions[letter.upper()] = float(val)
                except ValueError:
                    pass
        state["positions"] = positions

        m = re.search(r'\bTool\s+(\d+)', post("T"), re.IGNORECASE)
        state["active_tool"] = int(m.group(1)) if m else -1

        raw = json.loads(post('M409 K"tools"')).get("result") or []
        state["tools"] = {str(t["number"]): {"name": t.get("name", "")} for t in raw}
        state["tool_offsets"] = {str(t["number"]): t.get("offsets", [0.0, 0.0, 0.0]) for t in raw}

        parks = {}
        for idx in range(4):
            try:
                parks[str(idx)] = _parse_park_position(download_sys(f"tpost{idx}.g"))
            except Exception:
                pass
        state["tool_parks"] = parks

        return state
    except Exception:
        return None


def _read_env_address() -> str | None:
    """Read JUBILEE_ADDRESS from science-jubilee .env.hardware (only when JUBILEE_TRANSPORT=hardware)."""
    def _parse_env(env_file: Path) -> str | None:
        values: dict[str, str] = {}
        for line in env_file.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, _, v = line.partition("=")
                values[k.strip()] = v.strip()
        if values.get("JUBILEE_TRANSPORT", "").lower() == "hardware":
            return values.get("JUBILEE_ADDRESS") or None
        return None

    # Primary: entry point (CLI/pixi context).
    try:
        env_file = resolve("jubilee_dir") / ".env.hardware"
        if env_file.is_file():
            return _parse_env(env_file)
    except Exception:
        pass

    # Fallback: path pre-written into jubilee_paths.json by the driver (Blender addon context).
    try:
        paths_json = twin_dir() / "pipeline_data" / "jubilee_paths.json"
        env_path = json.loads(paths_json.read_text()).get("env_hardware")
        if env_path:
            env_file = Path(env_path)
            if env_file.is_file():
                return _parse_env(env_file)
    except Exception:
        pass

    return None


def _load_machine_state(machine_state_path: str | None = None) -> tuple[dict, str]:
    """Return (state_dict, source_label) using a 3-step fallback.

    1. Live Duet query via JUBILEE_ADDRESS from science-jubilee .env.hardware
       (only during autodiscovery — skipped when machine_state_path is explicit)
    2. Live Duet query via address stored in gcode_logs/machine_state.json
    3. Latest machine_state.json from gcode_logs/
    4. Built-in defaults (standard 4-tool Jubilee park positions)
    """
    # Step 1 — live query via .env.hardware (autodiscovery only)
    if not machine_state_path:
        hw_address = _read_env_address()
        if hw_address:
            live = _try_live_query(hw_address)
            if live is not None:
                return live, f"live (.env.hardware {hw_address})"

    # Step 2 — live query via address from last saved state
    saved_path: Path | None = None
    if machine_state_path:
        saved_path = Path(machine_state_path)
    else:
        try:
            candidate = resolve("jubilee_dir") / "gcode_logs" / "machine_state.json"
            if candidate.exists():
                saved_path = candidate
        except RuntimeError:
            pass

    if saved_path and saved_path.is_file():
        try:
            with open(saved_path) as f:
                saved = json.load(f)
            address = saved.get("address")
            if address:
                live = _try_live_query(address)
                if live is not None:
                    return live, f"live ({address})"
        except Exception:
            pass

    # Step 2 — saved machine_state.json
    if saved_path and saved_path.is_file():
        try:
            with open(saved_path) as f:
                return json.load(f), str(saved_path)
        except Exception as exc:
            logger.warning("Could not read %s: %s", saved_path, exc)

    # Step 3 — built-in defaults
    logger.warning("No live machine and no saved state found — using built-in defaults")
    with open(_DEFAULT_STATE) as f:
        return json.load(f), f"defaults ({_DEFAULT_STATE.name})"


def run(output_dir: str = None, machine_state_path: str = None) -> str:
    """Build tool_data.csv. Returns the output path.

    Discovery order: live Duet query → gcode_logs/machine_state.json → built-in defaults.
    Pass machine_state_path= to skip discovery and use a specific file.
    """
    state, source = _load_machine_state(machine_state_path)

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
    output_csv = os.path.join(out_dir, "tool_data.csv")

    with open(output_csv, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Tool_ID", "Name", "Park_X", "Park_Y", "Park_Z", "Offset_X", "Offset_Y", "Offset_Z"])
        for i in range(4):
            name = tool_names.get(i, f"Unassigned_Tool_{i}")
            park = tool_parks.get(i, np.array([0.0, 0.0, 0.0]))
            offset = tool_offsets.get(i, np.array([0.0, 0.0, 0.0]))
            writer.writerow([i, name, park[0], park[1], park[2], offset[0], offset[1], offset[2]])

    # Write current head position so Blender can place the gantry at the real position.
    positions = state.get("positions", {})
    if positions:
        status = {
            "head_position": {k: positions[k] for k in ("X", "Y", "Z") if k in positions},
            "active_tool": state.get("active_tool", -1),
            "source": source,
        }
        status_path = os.path.join(out_dir, "machine_status.json")
        with open(status_path, "w") as f:
            json.dump(status, f, indent=2)

    logger.info("Machine state: %s", source)
    logger.warning("tool_data.csv → %s", output_csv)
    if positions:
        pos = {k: positions[k] for k in ("X", "Y", "Z") if k in positions}
        logger.warning("Head position  X=%.1f  Y=%.1f  Z=%.1f",
                       pos.get("X", 0.0), pos.get("Y", 0.0), pos.get("Z", 0.0))
        logger.warning("machine_status → %s", status_path)
    return output_csv