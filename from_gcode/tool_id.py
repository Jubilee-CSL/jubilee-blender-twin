import sys
import os
import csv
import numpy as np
from utils import _extract_numeric_after, find_coord

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

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
                name = "No Name"
                for p in parts:
                    if p.startswith("P"):
                        try:
                            num = int(p[1:])
                        except ValueError:
                            pass
                    elif p.startswith("S"):
                        name = p[1:].strip('"')
                
                if num is not None:
                    names[num] = name

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
                                nl, _ = find_coord(line, np.array([0.0, 0.0, 0.0, 0.0, 0.0,-1.0]))
                                offsets[idx] = nl[:3]
                            except ValueError:
                                pass
    return offsets

if __name__ == "__main__":

    target_sys_dir = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "science_jubilee_twin_link", "firmware","sys")) #!!!!! A CHANGER
    
    print(f"Targeting config directory: {target_sys_dir}")

    # Extract Data
    tool_names = extract_names(target_sys_dir)
    tool_parks = extract_parks(target_sys_dir)
    tool_offsets = extract_offset(target_sys_dir)

    # Output to CSV in the from_gcode directory
    output_csv = os.path.join(SCRIPT_DIR, "tool_data.csv")
    
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