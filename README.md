# Blender URDF Robot Importer Retargeting-Optimized

## Step-by-Step Tutorial for New Users

### Goal
Import a humanoid URDF robot, retarget a BVH motion, and export a Beyond Mimic trajectory.

### Prerequisites
- Blender 5.0 or newer
- One robot `.urdf` file with referenced meshes
- One motion capture `.bvh` file

### Installation
1. Download `urdf_retargeting.zip` from the GitHub Release or build it locally.
2. Open Blender.
3. Go to **Edit → Preferences → Add-ons → Install from Disk…**.
4. Select `urdf_retargeting.zip`.
5. Enable **URDF Retargeting** in the add-on list.

### First Run: End-to-End Workflow
1. Import robot: **File → Import → URDF Humanoid**
2. Import motion: **File → Import → BVH**
3. Open sidebar: **N → URDF Retargeting**
4. Assign rigs:
   - `BVH Rig` = imported BVH armature
   - `URDF Rig` = imported URDF armature
5. Click **Generate Mapping List**
6. Either:
   - manually map BVH bones to URDF joints, or
   - load a preset from `Preset dropdown → Load Preset`
7. Ensure foot setup is valid (left/right foot names exist and differ)
8. Click **Apply Mapping**
9. Scrub timeline from frame 0 and verify robot movement follows the BVH source
10. Click **Export Beyond Mimic** and save CSV (+ optional JSON metadata)

### Expected Result
- A retargeted URDF animation plays in Blender
- Exported trajectory files can be consumed by downstream Beyond Mimic tooling

### Troubleshooting
1. Add-on not visible after install: restart Blender and verify version is 5.0+
2. Robot does not move: verify mapping coverage and check that `Apply Mapping` was run from frame 0
3. Sudden foot artifacts: verify left/right foot names and reduce aggressive solver settings
4. Export mismatch: ensure scene frame range and target Hz are set before export

---
## Roadmap
- [X] Add Zero-Lag Filter for the BVH Model
- [X] Add Projected Root logic to compensate foot slipping and scrubbing
  - [X] Combine Projected Root with Auto-Grounding
- [X] Retarget jump motions
