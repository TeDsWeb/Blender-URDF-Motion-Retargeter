"""
Motion retargeting engine for BVH-to-URDF animation mapping.

Provides the core retargeting handler that synchronizes BVH motion capture
data to URDF robot kinematics, including root motion projection,
joint retargeting, foot alignment, and anti-sinking corrections.
"""

import bpy
import mathutils
from bpy.app.handlers import persistent
from .utils import (
    extract_twist_angle,
    apply_continuity_correction,
    apply_velocity_limiting,
    apply_exponential_smoothing,
    clamp_to_limits,
    get_bone_property,
    set_bone_property,
)


def get_lowest_z_world(urdf_obj: bpy.types.Object) -> float:
    """
    Find the lowest Z-coordinate (height) of all visual meshes in world space.

    This is used to determine the ground plane position for foot contact detection
    and anti-sinking corrections.

    Args:
        urdf_obj: The URDF armature object (may contain child mesh objects).

    Returns:
        The minimum Z-coordinate found. If no meshes exist, uses bone positions.
    """
    global_min_z = float("inf")
    found_mesh = False

    bpy.context.view_layer.update()

    # Iterate through all objects parented to the rig (links/visuals)
    for child in urdf_obj.children:
        if child.type == "MESH":
            found_mesh = True
            # Transform local corners of the mesh bounding box to world space
            matrix_world = child.matrix_world
            for corner in child.bound_box:
                world_corner = matrix_world @ mathutils.Vector(corner)
                if world_corner.z < global_min_z:
                    global_min_z = world_corner.z

    # Fallback: If no meshes are found, use bone heads
    if not found_mesh:
        global_min_z = min(
            (urdf_obj.matrix_world @ pb.head).z for pb in urdf_obj.pose.bones
        )

    return global_min_z


def apply_root_motion_correction(
    scene: bpy.types.Scene,
    urdf_obj: bpy.types.Object,
    bvh_obj: bpy.types.Object,
    settings,
) -> tuple[mathutils.Vector, mathutils.Quaternion]:
    """
    Calculate root position and rotation corrections for pivot-based anchoring.

    Uses "sticky foot" logic: if a foot remains in contact, its position is held constant
    while the root moves, preventing foot-slip artifacts. Includes smoothed drift correction
    to reduce jitter from measurement noise.

    Args:
        scene: The Blender scene object (contains cached state).
        urdf_obj: The URDF armature to be retargeted.
        bvh_obj: The BVH source armature.
        settings: BVHMappingSettings with foot bone names and smoothing parameters.

    Returns:
        Tuple of (pivot_correction, rotation_correction) to apply to the URDF root.
    """
    pivot_correction = mathutils.Vector((0, 0, 0))
    rotation_correction = mathutils.Quaternion((1, 0, 0, 0))

    # Get BVH root position and rotation
    bvh_root_mat = bvh_obj.matrix_world @ bvh_obj.pose.bones[0].matrix
    bvh_floor_offset = scene.get("bvh_floor_offset", 0.0)

    # Identify which feet are in contact (sticky anchor selection)
    active_bvh_contacts = []
    for f_name in [settings.foot_l_name, settings.foot_r_name]:
        bvh_bone = bvh_obj.pose.bones.get(f_name)
        if bvh_bone:
            foot_world_pos = (bvh_obj.matrix_world @ bvh_bone.matrix).to_translation()
            if (foot_world_pos.z - bvh_floor_offset) <= settings.jump_threshold:
                active_bvh_contacts.append(f_name)

    last_anchor_name = scene.get("_active_anchor_name", "")
    anchor_bone_name = ""

    # 1. Sticky Selection: keep current foot if it's still in contact
    if last_anchor_name in active_bvh_contacts:
        anchor_bone_name = last_anchor_name
        is_hard_switch = False
    elif active_bvh_contacts:
        anchor_bone_name = sorted(active_bvh_contacts)[0]
        is_hard_switch = True
    else:
        anchor_bone_name = ""

    if anchor_bone_name:
        bvh_anchor_bone = bvh_obj.pose.bones.get(anchor_bone_name)
        current_bvh_anchor_mat = bvh_obj.matrix_world @ bvh_anchor_bone.matrix
        current_bvh_anchor_pos = current_bvh_anchor_mat.to_translation()
        current_bvh_anchor_quat = current_bvh_anchor_mat.to_quaternion()
        current_rel_pos = bvh_root_mat.inverted() @ current_bvh_anchor_pos

        if is_hard_switch or last_anchor_name == "":
            # Offset-Transfer: prevents jump when switching feet
            scene["_last_bvh_rel_pos"] = current_rel_pos
            scene["_last_bvh_anchor_quat"] = current_bvh_anchor_quat
            scene["_last_applied_pivot_corr"] = mathutils.Vector((0, 0, 0))
        else:
            # Calculate drift and apply smoothing
            last_rel_pos = mathutils.Vector(
                scene.get("_last_bvh_rel_pos", current_rel_pos)
            )
            raw_drift = (current_rel_pos - last_rel_pos) * settings.root_scale

            # Damping: smooth out high-frequency drift noise
            smoothing_factor = 0.6
            last_corr = mathutils.Vector(
                scene.get("_last_applied_pivot_corr", (0, 0, 0))
            )
            pivot_correction = last_corr.lerp(raw_drift, smoothing_factor)
            pivot_correction.z = 0  # Only XY plane adjustment for foot planting

            # Rotation drift correction
            last_anchor_quat = mathutils.Quaternion(
                scene.get("_last_bvh_anchor_quat", current_bvh_anchor_quat)
            )
            rot_drift = current_bvh_anchor_quat @ last_anchor_quat.inverted()
            rot_drift_euler = rot_drift.to_euler()
            rot_drift_quat = mathutils.Euler(
                (
                    -rot_drift_euler.x * 0.5,
                    rot_drift_euler.y * 0.5,
                    -rot_drift_euler.z,
                ),
                "XYZ",
            ).to_quaternion()
            rotation_correction = mathutils.Quaternion((1, 0, 0, 0)).slerp(
                rot_drift_quat.inverted(), smoothing_factor
            )

        # Update state cache
        scene["_active_anchor_name"] = anchor_bone_name
        scene["_last_bvh_rel_pos"] = current_rel_pos
        scene["_last_bvh_anchor_quat"] = current_bvh_anchor_quat
        scene["_last_applied_pivot_corr"] = pivot_correction
    else:
        scene["_active_anchor_name"] = ""
        scene["_last_applied_pivot_corr"] = mathutils.Vector((0, 0, 0))

    return pivot_correction, rotation_correction


def apply_joint_retargeting(
    urdf_obj: bpy.types.Object,
    bvh_obj: bpy.types.Object,
    mapping_item,
    smooth_cache: dict,
    settings,
    scene: bpy.types.Scene,
) -> None:
    """
    Retarget a single joint mapping from BVH to URDF.

    Extracts the per-axis rotation component from the BVH bone, applies continuity
    corrections, velocity limiting, and smoothing, then outputs to the URDF bone.

    Args:
        urdf_obj: The URDF armature object.
        bvh_obj: The BVH source armature object.
        mapping_item: A BVHMappingItem defining the BVH-to-URDF mapping.
        smooth_cache: Dictionary for caching continuity and smoothing state.
        settings: BVHMappingSettings with smoothing factors.
        scene: The Blender scene (for FPS and frame data).
    """
    bvh_b = bvh_obj.pose.bones.get(mapping_item.bvh_bone_name)
    if not bvh_b:
        return

    # Calculate the delta rotation relative to the reference (T-Pose)
    ref_q = mathutils.Quaternion(mapping_item.ref_rot)
    current_q = bvh_b.matrix_basis.to_quaternion()
    delta_q = ref_q.inverted() @ current_q

    # A single BVH bone might drive multiple URDF joints
    for urdf_bone_mapping in mapping_item.urdf_bones:
        urdf_b = urdf_obj.pose.bones.get(urdf_bone_mapping.urdf_bone_name)
        if not urdf_b:
            continue

        # Only handle revolute and continuous joints
        jtype = get_bone_property(urdf_b, "joint_type", "revolute")
        if jtype not in {"revolute", "continuous"}:
            continue

        # Define the twist axis in BVH local space
        if urdf_bone_mapping.source_axis == "X":
            twist_axis = mathutils.Vector((1, 0, 0))
        elif urdf_bone_mapping.source_axis == "Y":
            twist_axis = mathutils.Vector((0, 1, 0))
        else:
            twist_axis = mathutils.Vector((0, 0, 1))

        # Extract rotation around the twist axis
        val = extract_twist_angle(delta_q, twist_axis)

        # Apply continuity correction (anti-flip)
        cache_val_key = f"val_{bvh_obj.name}_{urdf_bone_mapping.urdf_bone_name}"
        if cache_val_key in smooth_cache:
            prev_val = smooth_cache[cache_val_key]
            val = apply_continuity_correction(val, prev_val)
        smooth_cache[cache_val_key] = val

        # Apply velocity limiting
        velocity_limit = get_bone_property(urdf_b, "velocity_limit", 10.0)
        last_raw_key = f"last_raw_{bvh_obj.name}_{urdf_bone_mapping.urdf_bone_name}"
        last_raw_val = smooth_cache.get(last_raw_key, val)

        target_hz = scene.render.fps if scene.render.fps > 0 else 60.0
        dt = 1.0 / target_hz

        val, was_limited = apply_velocity_limiting(
            val, last_raw_val, velocity_limit, dt
        )
        set_bone_property(urdf_b, "is_velocity_limited", was_limited)
        smooth_cache[last_raw_key] = val

        # Apply sign and neutral offset
        if urdf_bone_mapping.sign == "NEG":
            val = -val

        if "offset" not in urdf_b:
            set_bone_property(urdf_b, "offset", val)

        # Combine with calibration offset
        target_angle = (
            val
            - get_bone_property(urdf_b, "offset", 0.0)
            + urdf_bone_mapping.neutral_offset
        )

        # Apply exponential smoothing
        last_angle = get_bone_property(urdf_b, "_last_urdf_angle", target_angle)
        alpha_joint = 1.0 - settings.joint_smoothing
        final_angle = apply_exponential_smoothing(
            target_angle, last_angle, settings.joint_smoothing
        )
        set_bone_property(urdf_b, "_last_urdf_angle", final_angle)

        # Clamp to joint limits
        if jtype != "continuous":
            l_min = get_bone_property(urdf_b, "limit_lower", -3.14)
            l_max = get_bone_property(urdf_b, "limit_upper", 3.14)
            final_angle = clamp_to_limits(final_angle, l_min, l_max)

        # Store for export
        set_bone_property(urdf_b, "_joint_angle", final_angle)

        # Apply rotation
        urdf_b.rotation_mode = "QUATERNION"
        urdf_b.rotation_quaternion = mathutils.Quaternion((0, 1, 0), final_angle)


def apply_foot_alignment(
    urdf_obj: bpy.types.Object,
    bvh_obj: bpy.types.Object,
    settings,
    smooth_cache: dict,
    scene: bpy.types.Scene,
) -> None:
    """
    Apply dynamic foot alignment to prevent penetration and create natural foot planting.

    Detects when feet are near the ground and gradually aligns them to
    a flat orientation while respecting joint limits.

    Args:
        urdf_obj: The URDF armature object.
        bvh_obj: The BVH source armature object.
        settings: BVHMappingSettings with foot alignment parameters.
        smooth_cache: Dictionary for caching continuity state.
        scene: The Blender scene.
    """
    bpy.context.view_layer.update()

    flatten_str = settings.foot_flattening_strength
    flatten_height = settings.foot_flattening_height

    if flatten_str <= 0.0:
        return

    # Build BVH-to-URDF foot bone mapping
    bvh_to_urdf_map = {}
    for item in settings.mappings:
        if item.bvh_bone_name in [settings.foot_l_name, settings.foot_r_name]:
            urdf_data = [
                (b.urdf_bone_name, b.invert_alignment) for b in item.urdf_bones
            ]
            bvh_to_urdf_map[item.bvh_bone_name] = urdf_data

    for bvh_foot_name in [settings.foot_l_name, settings.foot_r_name]:
        if bvh_foot_name not in bvh_to_urdf_map:
            continue

        for u_name, invert_alignment in bvh_to_urdf_map[bvh_foot_name]:
            urdf_b = urdf_obj.pose.bones.get(u_name)
            if not urdf_b:
                continue

            # Only apply to revolute/continuous joints
            jtype = get_bone_property(urdf_b, "joint_type", "revolute")
            if jtype not in {"revolute", "continuous"}:
                continue

            # Get foot world position
            foot_mat_world = urdf_obj.matrix_world @ urdf_b.matrix
            foot_pos = foot_mat_world.to_translation()

            # Distance from foot to floor
            dist_to_floor = foot_pos.z - scene.get("urdf_foot_height_offset", 0.0)

            if dist_to_floor >= flatten_height:
                continue

            # Calculate influence based on proximity to floor
            influence = (1.0 - (dist_to_floor / flatten_height)) * flatten_str
            if influence <= 0.001:
                continue

            # Calculate alignment correction
            current_foot_mat = urdf_obj.matrix_world @ urdf_b.matrix
            current_foot_q = current_foot_mat.to_quaternion()

            actual_up = current_foot_q @ mathutils.Vector((0, 0, 1))
            target_up = mathutils.Vector((0, 0, 1))

            error_q = actual_up.rotation_difference(target_up)
            local_error_q = current_foot_q.inverted() @ (error_q @ current_foot_q)

            # Extract twist component
            twist_axis = mathutils.Vector((0, 1, 0))
            val = extract_twist_angle(local_error_q, twist_axis)

            # Apply invert flag
            if invert_alignment:
                val *= -1.0

            # Apply continuity correction
            cache_val_key = f"align_val_{bvh_obj.name}_{u_name}"
            if cache_val_key in smooth_cache:
                prev_val = smooth_cache[cache_val_key]
                val = apply_continuity_correction(val, prev_val)
            smooth_cache[cache_val_key] = val

            # Blend with current angle and apply limits
            current_joint_angle = get_bone_property(urdf_b, "_joint_angle", 0.0)
            final_angle = current_joint_angle + val * influence

            if jtype != "continuous":
                l_min = get_bone_property(urdf_b, "limit_lower", -3.14)
                l_max = get_bone_property(urdf_b, "limit_upper", 3.14)
                final_angle = clamp_to_limits(final_angle, l_min, l_max)

            # Store and apply
            set_bone_property(urdf_b, "_joint_angle", final_angle)
            urdf_b.rotation_mode = "QUATERNION"
            urdf_b.rotation_quaternion = mathutils.Quaternion((0, 1, 0), final_angle)


@persistent
def retarget_frame(scene: bpy.types.Scene) -> None:
    """
    Frame change handler that performs real-time BVH-to-URDF retargeting.

    This function is called on every frame change. It processes root motion projection,
    joint retargeting, foot alignment, and anti-sinking corrections to synchronize
    the URDF robot with BVH motion capture data.

    Args:
        scene: The current Blender scene.
    """
    settings = scene.bvh_mapping_settings
    if not settings.live_retarget:
        return

    urdf = scene.urdf_rig_object
    bvh = scene.smoothed_bvh_rig_object
    if not urdf or not bvh:
        return

    # Initialize smoothing cache if needed
    if "_bvh_smooth_cache" not in scene:
        scene["_bvh_smooth_cache"] = {}
    smooth_cache = scene["_bvh_smooth_cache"]

    if not bvh.pose.bones:
        return

    # --- ROOT MOTION & PIVOT CORRECTION ---
    pivot_correction, rotation_correction = apply_root_motion_correction(
        scene, urdf, bvh, settings
    )

    # Apply root rotation
    urdf.rotation_mode = "QUATERNION"
    urdf.rotation_quaternion = rotation_correction @ urdf.rotation_quaternion

    # Get reference position for Z-axis handling
    bvh_root_mat = bvh.matrix_world @ bvh.pose.bones[0].matrix
    current_bvh_pos = bvh_root_mat.to_translation()
    ref_pos = mathutils.Vector(scene.get("ref_root_pos", current_bvh_pos))
    bvh_floor_offset = scene.get("bvh_floor_offset", 0.0)

    # Calculate vertical movement
    corrected_current_z = current_bvh_pos.z - bvh_floor_offset
    delta_z = (
        corrected_current_z - (ref_pos.z - bvh_floor_offset)
    ) * settings.root_scale

    # Apply position
    target_loc = (
        mathutils.Vector(
            (
                (current_bvh_pos.x - ref_pos.x) * settings.root_scale,
                (current_bvh_pos.y - ref_pos.y) * settings.root_scale,
                delta_z,
            )
        )
        - pivot_correction
    )

    offset_xy = settings.location_offset
    loc_offset = mathutils.Vector(
        (offset_xy[0], offset_xy[1], scene.get("urdf_height_offset", 0.0))
    )
    urdf.location = target_loc + loc_offset

    # Apply root rotation offset
    ref_rot = mathutils.Quaternion(scene.get("ref_root_rot", current_bvh_pos))
    off_q = mathutils.Euler(settings.rotation_offset).to_quaternion()
    current_bvh_rot = bvh_root_mat.to_quaternion()
    delta_rot = (current_bvh_rot @ ref_rot.inverted()).inverted()
    delta_rot.x *= -1
    delta_rot.z *= -1
    urdf.rotation_quaternion = off_q @ delta_rot

    # --- JOINT RETARGETING ---
    for item in settings.mappings:
        apply_joint_retargeting(urdf, bvh, item, smooth_cache, settings, scene)

    # --- FOOT ALIGNMENT ---
    apply_foot_alignment(urdf, bvh, settings, smooth_cache, scene)

    # --- ANTI-SINKING CHECK ---
    bpy.context.view_layer.update()
    current_min_z = get_lowest_z_world(urdf)

    if current_min_z < -1e-4:  # Allow tiny epsilon for numerical stability
        urdf.location.z += abs(current_min_z)
