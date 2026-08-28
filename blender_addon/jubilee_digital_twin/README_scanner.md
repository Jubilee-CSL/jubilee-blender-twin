# Virtual Scanner — Quick Start

Generates a synthetic image dataset by driving the digital twin over a grid of
`(toolhead_x, toolhead_y, bed_z)` machine positions and rendering one image per
point through a calibrated pinhole camera parented to the toolhead.

Output is intended for testing / validating the Science Jubilee 3D
reconstruction pipelines (Marigold, 3DGS-MCMC).

## Prerequisites

1. Build and install the addon:
   ```powershell
   cd blender_addon
   .\build_addon.ps1
   ```
   Then in Blender: `Edit → Preferences → Add-ons → Install from Disk…` and
   pick `jubilee_digital_twin.zip`.
2. Run once from the repo root:
   ```powershell
   jubilee-twin setup-scene
   ```
   (populates `pipeline_data/jubilee_paths.json` and `tool_data.csv`).
3. Open `blender_models/jubilee.blend`, or use the CLI shortcut which opens
   the working blend file in the Blender GUI:
   ```powershell
   jubilee-twin open
   ```
   The scene must contain the objects `X-axis`, `Y-axis` and `Z-axis`
   (already present in the shipped file).

## Using the panel

`View3D → Sidebar → Twin → Digital Twin → Virtual Scanner`

### 1. Grid (machine mm)

| Field | Default | Source |
|---|---|---|
| X min / max / steps | 110 / 250 / 5 | inferred from `images_justin/` |
| Y min / max / steps |  80 / 200 / 5 | inferred from `images_justin/` |
| Z min / max / steps | 280 / 320 / 3 | inferred from `images_justin/` |

`X`/`Y` are the toolhead XY carriage position; `Z` is the **bed height**
(camera stays fixed, bed moves). Same convention as the reference dataset
filenames `img_x{X}_y{Y}_z{Z}.jpg`.

Click **Reload Defaults from Reference Folder** to re-scan
`images_justin/` and repopulate the ranges/steps from whatever filenames it
finds.

### 2. Camera

| Field | Default (Justin's calibration) |
|---|---|
| W × H | 1920 × 1056 |
| fx, fy | 1467.554, 1476.769 |
| cx, cy | 961.213, 538.565 |
| Offset (mm) | `(0, -20, 0)` — camera position relative to toolhead |

Click **Setup Scanner Camera** once. This
- creates (or refreshes) `ScannerCamera` in the scene,
- parents it to the `X-axis` object via a `CHILD_OF` constraint (translation
  only — camera keeps world-down orientation),
- applies the local offset in metres,
- sets render resolution + Blender lens / sensor / pixel-aspect / shift so
  the pinhole model matches `(fx, fy, cx, cy)`.

You can freely tweak the camera orientation in Blender after setup; the
scan operator won't overwrite it as long as the object is named
`ScannerCamera`.

> **Note on distortion.** Blender's camera is pinhole; the 5 Brown-Conrady
> `dist` coefficients from the calibration are **not** applied to the render.
> They are copied verbatim into the emitted `camera.yaml` so downstream
> pipelines know the physical calibration.

### 3. Run

Click **Run Virtual Scan**. For every `(x, y, z)` on the grid the addon
- sets `X-axis`, `Y-axis`, `Z-axis` locations using the same
  `axis_max - mm/1000` mapping used by `animate_path.py`,
- renders one JPEG.

## Output layout

```
<repo>/Scans/YYYYmmdd_HHMMSS/
├── img_x110_y80_z280.jpg
├── img_x110_y80_z300.jpg
├── ...
├── camera.yaml     # intrinsics + image size + distortion_applied: false
└── manifest.json   # grid config + per-frame evaluated camera world pose
```

`Scans/` is git-ignored.

## Headless / batch mode

The same code runs without the GUI, which is the recommended way to
generate large datasets:

```powershell
blender blender_models/jubilee.blend --background --python-expr `
"from jubilee_digital_twin.virtual_scanner import run_scan, ScanConfig; `
 run_scan(ScanConfig(x_min=110, x_max=250, x_steps=5, `
                     y_min=80,  y_max=200, y_steps=5, `
                     z_min=280, z_max=320, z_steps=3))"
```

Any field of `ScanConfig` (image size, intrinsics dict, `output_root`) can be
overridden from the command line.

## Troubleshooting

- **"Scene is missing one of 'X-axis', 'Y-axis', 'Z-axis'"** — you opened a
  `.blend` other than `blender_models/jubilee.blend`, or the axis objects
  were renamed. The addon locates them by exact name.
- **"No reference images found in …/images_justin"** — the reference folder
  is missing. The defaults in the panel still work; you just can't re-derive
  them from disk.
- **Camera not moving with the toolhead** — click **Setup Scanner Camera**
  again; the `CHILD_OF` constraint's inverse matrix is recomputed each time.
- **fx ≠ fy handling** — the difference is folded into
  `render.pixel_aspect_y = fx / fy`. For Justin's calibration this is
  ≈ 0.9938 (a 0.6 % correction).
