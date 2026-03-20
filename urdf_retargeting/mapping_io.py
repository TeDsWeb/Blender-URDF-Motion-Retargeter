"""Utilities for JSON mapping import/export and quaternion-mode checks."""

import json
import os
import re


PRESET_SCHEMA_VERSION = 3
PRESET_FORMAT = "urdf_retargeting_mapping_v3"

_SETTINGS_ALIASES = {
    "joint_smoothing": "output_joint_smoothing",
    "ik_iterations": "solver_iterations",
    "quality_ik_iterations": "solver_iterations",
    "ik_tolerance": "solver_tolerance",
    "ik_max_step_angle": "solver_step_limit",
    "ik_target_scale": "solver_target_scale",
    "ik_proportion_blend": "solver_proportion_blend",
    "ik_ground_lock_strength": "solver_ground_lock_strength",
    "ik_target_smoothing": "solver_target_smoothing",
    "hybrid_ik_blend": "hybrid_master_blend",
    "hybrid_adaptive_ik": "hybrid_auto_blend",
    "hybrid_min_ik_blend": "hybrid_blend_min",
    "hybrid_error_low": "hybrid_blend_error_low",
    "hybrid_error_high": "hybrid_blend_error_high",
}

_CHAIN_ALIASES = {
    "use_hybrid_adaptive_override": "use_auto_blend_override",
    "hybrid_min_ik_blend": "auto_blend_min",
    "hybrid_error_low": "auto_blend_error_low",
    "hybrid_error_high": "auto_blend_error_high",
    "orientation_weight": "solver_orientation_weight",
}


PRESET_SETTING_FIELDS = [
    "retargeting_method",
    "bvh_smoothing",
    "output_joint_smoothing",
    "output_smoothing_policy",
    "ik_output_smoothing_min",
    "ik_only_mode",
    "max_jump_threshold",
    "solver_iterations",
    "solver_tolerance",
    "solver_step_limit",
    "solver_target_scale",
    "solver_proportion_blend",
    "solver_ground_lock_strength",
    "solver_target_smoothing",
    "hybrid_master_blend",
    "hybrid_auto_blend",
    "hybrid_blend_min",
    "hybrid_blend_error_low",
    "hybrid_blend_error_high",
    "foot_l_name",
    "foot_r_name",
    "jump_threshold",
    "foot_contact_hysteresis",
    "foot_flattening_height",
    "foot_flattening_strength",
    "foot_pin_xy_max_step",
    "foot_pin_yaw_max_step",
    "adaptive_foot_pinning",
    "foot_pin_adaptive_gain",
    "correction_decay",
    "correction_decay_airborne_only",
    "root_scale",
    "location_offset",
    "rotation_offset",
    "target_hz",
    "default_pose_root_rotation",
    "import_use_meta_hz",
    "import_manual_hz",
    "import_set_scene_fps",
    "mapping_preset_name",
    "mapping_robot_profile",
    "mapping_mocap_profile",
    "stability_debug_metrics",
]


def _migrate_settings_dict(settings_data: dict) -> dict:
    """Normalize settings keys from legacy/new aliases to canonical property names."""
    migrated = {}
    for key, value in settings_data.items():
        canonical = _SETTINGS_ALIASES.get(key, key)
        migrated[canonical] = value

    if "retargeting_method" in migrated:
        migrated["retargeting_method"] = {
            "0": "FK_ONLY",
            "1": "IK_ONLY",
            "2": "HYBRID",
        }.get(str(migrated["retargeting_method"]), migrated["retargeting_method"])

    return migrated


def _migrate_chain_dict(chain_data: dict) -> dict:
    """Normalize chain keys from legacy/new aliases to canonical property names."""
    migrated = {}
    for key, value in chain_data.items():
        canonical = _CHAIN_ALIASES.get(key, key)
        migrated[canonical] = value
    return migrated


def _to_json_value(value):
    """Convert Blender property values to JSON-serializable values."""
    if isinstance(value, (list, tuple)):
        return list(value)
    try:
        # Blender float vectors behave like sequences but are not plain tuples/lists.
        if hasattr(value, "__len__") and not isinstance(value, (str, bytes, dict)):
            return [value[i] for i in range(len(value))]
    except Exception:
        pass
    return value


def get_preset_library_dir():
    """Return bundled preset-library directory path."""
    return os.path.join(os.path.dirname(__file__), "presets", "mappings")


def ensure_preset_library_dir():
    """Ensure preset-library directory exists and return path."""
    path = get_preset_library_dir()
    os.makedirs(path, exist_ok=True)
    return path


def sanitize_preset_filename(name):
    """Convert user preset name to a safe JSON filename."""
    clean = re.sub(r"[^A-Za-z0-9._-]+", "_", (name or "").strip())
    clean = clean.strip("._")
    return f"{clean or 'mapping_preset'}.json"


def export_mapping_to_json(filepath, settings, metadata=None):
    """Write mapping + kinematic chain configuration to JSON."""
    data = {
        "format": PRESET_FORMAT,
        "schema_version": PRESET_SCHEMA_VERSION,
        "metadata": metadata or {},
        "settings": {},
        "mappings": [],
        "kinematic_chains": [],
        "default_pose": {
            "joints": [],
        },
    }

    for key in PRESET_SETTING_FIELDS:
        if hasattr(settings, key):
            data["settings"][key] = _to_json_value(getattr(settings, key))

    for item in settings.mappings:
        data["mappings"].append(
            {
                "bvh_bone_name": item.bvh_bone_name,
                "urdf_bones": [
                    {
                        "urdf_bone_name": ub.urdf_bone_name,
                        "source_axis": ub.source_axis,
                        "sign": ub.sign,
                        "invert_alignment": ub.invert_alignment,
                        "neutral_offset": ub.neutral_offset,
                    }
                    for ub in item.urdf_bones
                ],
                "ref_rot": list(item.ref_rot),
            }
        )

    for chain in settings.kinematic_chains:
        data["kinematic_chains"].append(
            {
                "label": chain.label,
                "bvh_target_bone_name": chain.bvh_target_bone_name,
                "bvh_root_bone_name": chain.bvh_root_bone_name,
                "urdf_root_bone_name": chain.urdf_root_bone_name,
                "urdf_end_bone_name": chain.urdf_end_bone_name,
                "influence": chain.influence,
                "use_auto_blend_override": chain.use_auto_blend_override,
                "auto_blend_min": chain.auto_blend_min,
                "auto_blend_error_low": chain.auto_blend_error_low,
                "auto_blend_error_high": chain.auto_blend_error_high,
                "solver_orientation_weight": chain.solver_orientation_weight,
            }
        )

    for item in getattr(settings, "default_pose_joints", []):
        data["default_pose"]["joints"].append(
            {
                "joint_name": item.joint_name,
                "angle": item.angle,
            }
        )

    with open(filepath, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2)


def import_mapping_from_json(filepath, settings):
    """Load mapping + kinematic chain configuration from JSON.

    Returns:
        dict: metadata section from file (or {}).
    """
    with open(filepath, "r", encoding="utf-8") as handle:
        data = json.load(handle)

    migrated_settings = _migrate_settings_dict(data.get("settings", {}))
    for key, value in migrated_settings.items():
        if not hasattr(settings, key):
            continue
        try:
            current_value = getattr(settings, key)
            if isinstance(value, list) and not isinstance(current_value, str):
                setattr(settings, key, tuple(value))
            else:
                setattr(settings, key, value)
        except Exception:
            # Keep import robust against version mismatches.
            continue

    if hasattr(settings, "default_pose_root_rotation"):
        r = settings.default_pose_root_rotation
        settings.default_pose_root_rotation = (r[0], r[1], 0.0)

    settings.mappings.clear()
    for m in data.get("mappings", []):
        item = settings.mappings.add()
        item.bvh_bone_name = m.get("bvh_bone_name", "")
        ref_rot = m.get("ref_rot")
        if ref_rot and len(ref_rot) == 4:
            item.ref_rot = ref_rot

        for ub in m.get("urdf_bones", []):
            u_bone = item.urdf_bones.add()
            u_bone.urdf_bone_name = ub.get("urdf_bone_name", "")
            u_bone.source_axis = ub.get("source_axis", "X")
            u_bone.sign = ub.get("sign", "NEG")
            u_bone.invert_alignment = bool(ub.get("invert_alignment", False))
            u_bone.neutral_offset = float(ub.get("neutral_offset", 0.0))

    settings.kinematic_chains.clear()
    for c in data.get("kinematic_chains", []):
        chain_data = _migrate_chain_dict(c)
        chain = settings.kinematic_chains.add()
        chain.label = chain_data.get("label", "")
        chain.bvh_target_bone_name = chain_data.get("bvh_target_bone_name", "")
        chain.bvh_root_bone_name = chain_data.get("bvh_root_bone_name", "")
        chain.urdf_root_bone_name = chain_data.get("urdf_root_bone_name", "")
        chain.urdf_end_bone_name = chain_data.get("urdf_end_bone_name", "")
        chain.influence = float(chain_data.get("influence", 1.0))
        chain.use_auto_blend_override = bool(
            chain_data.get("use_auto_blend_override", False)
        )
        chain.auto_blend_min = float(chain_data.get("auto_blend_min", 0.2))
        chain.auto_blend_error_low = float(chain_data.get("auto_blend_error_low", 0.01))
        chain.auto_blend_error_high = float(
            chain_data.get("auto_blend_error_high", 0.08)
        )
        chain.solver_orientation_weight = float(
            chain_data.get("solver_orientation_weight", 0.55)
        )

    settings.default_pose_joints.clear()
    for item in data.get("default_pose", {}).get("joints", []):
        pose_item = settings.default_pose_joints.add()
        pose_item.joint_name = item.get("joint_name", "")
        pose_item.angle = float(item.get("angle", 0.0))

    settings.active_mapping_index = 0
    settings.active_urdf_index = 0
    settings.active_kinematic_chain_index = 0
    settings.default_pose_active_index = 0
    return data.get("metadata", {})


def count_non_quaternion_bones(armature_obj):
    """Count pose bones that are not in QUATERNION rotation mode."""
    if not armature_obj or armature_obj.type != "ARMATURE":
        return 0

    count = 0
    for pbone in armature_obj.pose.bones:
        if getattr(pbone, "rotation_mode", "QUATERNION") != "QUATERNION":
            count += 1
    return count
