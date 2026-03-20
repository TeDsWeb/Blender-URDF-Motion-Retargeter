# Mapping Preset Library

This folder stores robot/skeleton-specific mapping presets as JSON files.

Recommended naming:
- `Robot_MoCap_Variant.json`
- Example: `BoosterK1_OptiTrack_v1.json`

Each preset should represent one specific pair:
- target robot profile
- source MoCap skeleton/profile

Use the addon UI in `Actions -> Mapping Preset (JSON)`:
- `Save To Library` to create/update presets in this folder
- `Load Library Preset` to apply a saved preset

Schema notes:
- New exports use `format = urdf_retargeting_mapping_v3` with `schema_version = 3`.
- Older presets are still loadable; import applies key migration aliases for renamed setting fields.
- Per-chain `solver_orientation_weight` is now serialized/restored explicitly.
