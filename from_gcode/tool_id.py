import sys
import os
import csv
import numpy as np
from utils import _extract_numeric_after, find_coord

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def _discover_sys_dir() -> str:
    """Resolve firmware/sys/ via the jubilee.paths/jubilee_dir entry point."""
    from importlib.metadata import entry_points
    eps = [ep for ep in entry_points(group="jubilee.paths") if ep.name == "jubilee_dir"]
    if not eps:
        raise RuntimeError(
            "jubilee.paths/jubilee_dir entry point not found. "
            "Check that science-jubilee is installed in the environment and that jubilee.paths/jubilee_dir is registered."
        )
    jubilee_root = eps[0].load()()
    candidate = os.path.join(str(jubilee_root), "firmware", "sys")
    if not os.path.isdir(candidate):
        raise RuntimeError(f"firmware/sys not found at: {candidate}")
    return candidate

def extract_names(sys_dir):
    names = {}
    fn = os.path.join(sys_dir, "config.g")
    
    if not os.path.exists(fn):
        print(f"Warning: Could not find {fn}")
        return names

    with open(fn, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith("M563"):
                parts = line.split()
                num = None
                name = None
                for p in parts:
                    if p.startswith("P"):
                        try:
                            num = int(p[1:])
                        except ValueError:
                            pass  # non-integer P token — malformed line, skip
                    elif p.startswith("S"):
                        name = p[1:].strip('"')

                if num is not None:
                    if name is None:
                        print(f"Warning: M563 P{num} has no S (name) field — tool will be unnamed")
                    names[num] = name or f"Unnamed_Tool_{num}"

    return names

def extract_parks(sys_dir):
    park_pos = {}

    for idx in range(4): # Assuming a max of 3 tools for Jubilee
        fn = os.path.join(sys_dir, f"tpost{idx}.g")
        park_pos[idx] = np.array([0.0, 0.0, 0.0]) 
        
        if os.path.exists(fn):
            with open(fn, 'r') as f:
                nl = np.array([0.0, 0.0, 0.0, 0.0, 0.0,-1.0])        
                for line in f:
                    split_line = line.split()
                    if split_line and split_line[0] == "G53":
                        nl, _ = find_coord(line, nl)
                
                park_pos[idx] = nl[:3]

    return park_pos

def extract_offset(sys_dir):
    offsets = {}
    fn = os.path.join(sys_dir, "toffsets.g")
    
    for idx in range(4):
        offsets[idx] = np.array([0.0, 0.0, 0.0])

    if os.path.exists(fn):
        with open(fn, 'r') as f:
            for line in f:
                split_line = line.split()
                if split_line and split_line[0] == "G10":
                    for p in split_line:
                        if p.startswith("P"):
                            try:
                                idx = int(p[1:])
                            except ValueError:
                                print(f"Warning: non-integer P token in toffsets.g: {p!r} — line skipped")
                                break
                            nl, _ = find_coord(line, np.array([0.0, 0.0, 0.0, 0.0, 0.0, -1.0]))
                            offsets[idx] = nl[:3]
    return offsets

if __name__ == "__main__":

    if len(sys.argv) > 1:
        target_sys_dir = os.path.abspath(sys.argv[1])
    else:
        target_sys_dir = _discover_sys_dir()

    print(f"Targeting config directory: {target_sys_dir}")
    
    tool_names = extract_names(target_sys_dir)
    tool_parks = extract_parks(target_sys_dir)
    tool_offsets = extract_offset(target_sys_dir)

    pipeline_data_dir = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "pipeline_data"))
    os.makedirs(pipeline_data_dir, exist_ok=True)
    output_csv = os.path.join(pipeline_data_dir, "tool_data.csv")
    
    with open(output_csv, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # Write Headers
        writer.writerow([
            "Tool_ID", "Name", 
            "Park_X", "Park_Y", "Park_Z", 
            "Offset_X", "Offset_Y", "Offset_Z"
        ])
        
        # Write rows for Tools 0 through 3
        for i in range(4):
            name = tool_names.get(i, f"Unassigned_Tool_{i}")
            park = tool_parks.get(i, np.array([0.0, 0.0, 0.0]))
            offset = tool_offsets.get(i, np.array([0.0, 0.0, 0.0]))
            
            writer.writerow([
                i, name,
                park[0], park[1], park[2],
                offset[0], offset[1], offset[2]
            ])
            
    print(f"Extraction successful! Data saved to {output_csv}")