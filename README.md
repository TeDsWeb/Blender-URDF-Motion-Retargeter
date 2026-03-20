# URDF Retargeting – Blender Extension

> Imports URDF robots and transfers BVH motion capture animations onto their kinematics.  
> Exports finished trajectories in the **Beyond Mimic** CSV/JSON format.

---

## Table of Contents

1. [Repository Overview](#1-repository-overview)
2. [Development Environment Setup](#2-development-environment-setup)
3. [CI/CD – Automated Build](#3-cicd--automated-build)
4. [Installing the Extension](#4-installing-the-extension)
5. [Features](#5-features)
6. [Quick-Start Workflow](#6-quick-start-workflow)

---

## 1. Repository Overview

```
Blender-URDF-Robot-Importer-Retargeting-Optimized/
│
├── urdf_retargeting/          # Blender extension source code
│   ├── __init__.py            # Addon entry point, bl_info, registration
│   ├── blender_manifest.toml  # Blender extension manifest (version, license)
│   ├── armature.py            # Create URDF armature & bind meshes
│   ├── data_structures.py     # PropertyGroup data models (mappings, chains …)
│   ├── export.py              # Beyond Mimic CSV/JSON export
│   ├── import_csv.py          # Beyond Mimic CSV import
│   ├── mapping_io.py          # JSON preset load/save
│   ├── operators.py           # Blender operators (import, mapping, calibrate …)
│   ├── retargeting.py         # Core retargeting engine (FK, IK, foot planting)
│   ├── ui.py                  # Panels & lists in the Blender sidebar
│   ├── urdf.py                # URDF parser
│   ├── utils.py               # Helper functions (twist, smoothing, limits …)
│   └── presets/mappings/      # Bundled mapping preset JSON files
│
├── .github/workflows/
│   └── build.yml              # CI workflow: build ZIP + create release
│
├── urdf_retargeting.zip       # Last locally built ZIP (do not commit)
└── README.md
```

**Dependencies:** The extension uses only Blender-internal APIs (`bpy`, `mathutils`). No external Python libraries are required.

---

## 2. Development Environment Setup

### Prerequisites

| Tool | Minimum version |
|---|---|
| [Blender](https://www.blender.org/download/) | 5.0 |
| Python | 3.11 (Blender's internal Python) |
| Git | any |

### Clone the repository

```bash
git clone https://github.com/<user>/Blender-URDF-Robot-Importer-Retargeting-Optimized.git
cd Blender-URDF-Robot-Importer-Retargeting-Optimized
```

### Optional local venv (for linting / IDE support)

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install fake-bpy-module   # bpy type stubs – for the IDE only
```

> The venv is **not** used by Blender. It exists solely to provide autocompletion and type checks in VS Code or PyCharm.

### Build the ZIP locally

```powershell
cd urdf_retargeting
Compress-Archive -Path * -DestinationPath ..\urdf_retargeting.zip -Force
```

---

## 3. CI/CD – Automated Build

On every **push** or **pull request** to any branch, the ZIP is built automatically and saved as a **workflow artifact** (available for 90 days):

```
GitHub → Actions → <Workflow Run> → Artifacts → urdf_retargeting-v1.0.0-<sha>
```

### Creating a permanent release

When a Git tag with the `v` prefix is pushed, the workflow additionally creates a **GitHub Release** with the ZIP as a downloadable asset (available indefinitely):

```bash
git tag v1.0.0
git push origin v1.0.0
```

The release will appear under `GitHub → Releases` and includes an automatically generated changelog compiled from commit messages since the last tag.

---

## 4. Installing the Extension

### Option A – From a release (recommended)

1. Go to **Releases** on GitHub and open the desired version.
2. Download `urdf_retargeting.zip`.
3. In Blender: **Edit → Preferences → Add-ons → Install from Disk…**
4. Select the downloaded ZIP and confirm with **Install Add-on**.
5. Search for **URDF Retargeting** in the list and enable the checkbox.

### Option B – Local ZIP

1. Build the ZIP locally (see Section 2).
2. Follow steps 3–5 from Option A.

### Option C – Developer mode (symlink)

To see source code changes reflected in Blender immediately, link the `urdf_retargeting/` folder directly into Blender's addon directory:

```powershell
# Blender 5.x – adjust path if needed
$target = "$env:APPDATA\Blender Foundation\Blender\5.0\scripts\addons\urdf_retargeting"
New-Item -ItemType SymbolicLink -Path $target -Target (Resolve-Path .\urdf_retargeting)
```

After a code change, **Edit → Preferences → Add-ons → Reload Scripts** (or `F3 → "Reload Scripts"`) in Blender is sufficient.

---

## 5. Features

### URDF Import

- Parses `.urdf` files and builds a complete armature in Blender.
- Automatically binds referenced mesh files (`.stl`, `.dae`, `.obj`) to the joints.
- Accessible via **File → Import → URDF Humanoid**.

### BVH-to-URDF Mapping

- Creates a configurable mapping list of all BVH bones.
- Per BVH bone: any number of URDF joints can be assigned, each with axis selection (`X/Y/Z`) and sign control.
- Mappings can be saved and reused as **JSON presets**.  
  Bundled preset: `BoosterK1_OptiTrack_v1.json`.
- Presets also persist all global tuning parameters and the custom default pose alongside the mappings.

### Retargeting Modes

The current UI is built around a hybrid-first workflow:

| Mode | Description |
|---|---|
| **FK Only** | Direct twist extraction from BVH quaternions onto URDF joints without IK correction. |
| **Hybrid FK + IK** | Adaptive blend between FK baseline and IK correction per chain; the intended production workflow. |
| **IK Only (Debug)** | Debug-only mode that bypasses the FK baseline to inspect solver behavior in isolation. |

### Key Runtime Controls

The most important global tuning properties are now grouped conceptually as follows:

| Group | Canonical properties | Purpose |
|---|---|---|
| **Output smoothing** | `output_joint_smoothing`, `output_smoothing_policy`, `ik_output_smoothing_min` | Controls post-process damping and how strongly IK target smoothing suppresses additional output damping. |
| **Solver** | `solver_iterations`, `solver_tolerance`, `solver_step_limit`, `solver_target_scale`, `solver_proportion_blend`, `solver_ground_lock_strength`, `solver_target_smoothing` | Controls FABRIK-C quality, stability and end-effector target behavior. |
| **Hybrid blend** | `hybrid_master_blend`, `hybrid_auto_blend`, `hybrid_blend_min`, `hybrid_blend_error_low`, `hybrid_blend_error_high` | Controls how strongly IK corrects the FK baseline and how that influence scales with residual error. |
| **Per-chain override** | `solver_orientation_weight`, `use_auto_blend_override`, `auto_blend_min`, `auto_blend_error_low`, `auto_blend_error_high` | Overrides the global hybrid behavior for a specific kinematic chain. |

Older presets are still supported. On import, legacy field names are migrated automatically to the new canonical property names.

### Motion Quality & Stabilisation

- **Zero-Lag Smoothing**: Bidirectional (forward + backward) filtering of BVH F-curves – reduces noise without phase delay.
- **Output Joint Smoothing**: Exponential smoothing of URDF joint angles per frame after FK, IK and foot-alignment writes have been merged.
- **Smoothing Policy**: `output_smoothing_policy` avoids double damping when `solver_target_smoothing` is already high; runtime telemetry exposes the effective output smoothing used each frame.
- **Foot Contact Detection**: Hysteresis-based contact detection; stance foot is anchored to the ground (foot planting).
- **Anti-Sinking / Auto-Grounding**: Automatically raises the robot so the lowest point never drops below Z=0.
- **Jump Support**: Persistent corrections are frozen during airborne phases; new anchor points are set on landing.
- **Shared FK/IK Continuity Guard**: FK, IK and foot-alignment now pass through a shared continuity/jump-threshold write path before final output smoothing.
- **Persistent Correction with Decay**: Gradual rollback of corrections (`correction_decay`) keeps the trajectory close to the original.

### Export – Beyond Mimic

- Exports the finished URDF animation as a **CSV** (joint trajectory) plus an optional **JSON metadata** file.
- Configurable target Hz (resampling to any output frequency).
- Supports **Custom Default Pose**: root rotation and joint offsets are baked in as the zero position.

### Import – Beyond Mimic

- Reads an existing Beyond Mimic CSV and plays back the trajectory as a keyframe animation on the loaded URDF rig.
- Automatic Hz detection via accompanying `_meta.json` or manual override.

---

## 6. Quick-Start Workflow

```
1.  Load URDF        File → Import → URDF Humanoid  →  select .urdf file
2.  Load BVH         File → Import → BVH             →  select .bvh file
3.  Assign rigs      Sidebar (N) → URDF Retargeting  →  set BVH Rig & URDF Rig
4.  Generate mapping  "Generate Mapping List"         →  assign bones manually
    or load preset    Preset dropdown → "Load Preset"
5.  Retarget         "Apply Mapping"                 →  live preview from frame 0 (includes calibration)
6.  Export           "Export Beyond Mimic"            →  save CSV + JSON
```

> **Tip:** Saving a mapping preset (`Save Preset`) persists all tuning parameters, kinematic chains, and the custom default pose — making the entire setup instantly restorable on another machine or for a different BVH instance.

> **Testing Tip:** Use [BLENDER_RUNTIME_TEST_PLAN.md](BLENDER_RUNTIME_TEST_PLAN.md) to verify Hybrid mode, IK-only debug behavior, smoothing telemetry, preset migration, Bake and Export after changes.
