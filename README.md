# Blender URDF Robot Importer Retargeting-Optimized
## TODO
- [X] Add Zero-Lag Filter for the BVH Model
- [X] Add Projected Root logic to compensate foot slipping and scrubbing
  - [X] Combine Projected Root with Auto-Grounding
- [X] Find a way to also retarget jump motions 

## Retargeting Modes

- Angle Mapping: Existing per-joint twist extraction from BVH to URDF.
- Kinematic IK: Alternative chain-based retargeting that maps BVH end-effector targets onto URDF joint chains with a CCD solver.

For the kinematic mode, configure explicit chains in Blender by assigning:

- a BVH target bone
- a URDF root joint
- a URDF end joint

This mode is intended for limbs or other serial chains where end-effector tracking is more useful than direct joint-angle transfer.
