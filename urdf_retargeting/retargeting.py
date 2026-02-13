"""
Motion retargeting engine for BVH-to-URDF animation mapping.

Provides the core retargeting handler that synchronizes BVH motion capture
data to URDF robot kinematics, including root motion projection,
joint retargeting, foot alignment, and anti-sinking corrections.

Foot-planting uses a single-pass "persistent correction" approach:
  1. Root is positioned at BVH_delta + persistent_correction + manual offset.
  2. Full FK (joint angles + foot alignment) is applied, then the scene is updated.
  3. The world position of the stance foot is compared against a fixed anchor.
  4. Any XY drift is subtracted from the root and written back into
     persistent_correction so the fix carries over to the next frame.
  5. On stance-foot switch the NEW foot's current (already-corrected) world
     position becomes the anchor — no jump, no fade required.
  6. During airborne phases (jump) persistent_correction is frozen; on landing
     the first grounded foot sets a fresh anchor.
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


def detect_stance_foot(
    scene: bpy.types.Scene,
    bvh_obj: bpy.types.Object,
    settings,
) -> tuple[str, bool]:
    """
    Determine which BVH foot is the current stance foot (ground anchor).

    Uses ground-contact height threshold combined with per-foot velocity to
    pick the most stable grounded foot.  Hysteresis prevents rapid flicker
    between feet.

    Args:
        scene:   The Blender scene (stores inter-frame tracking state).
        bvh_obj: The BVH source armature.
        settings: BVHMappingSettings with foot bone names and jump_threshold.

    Returns:
        (anchor_bone_name, is_anchor_switch)
        anchor_bone_name is "" when both feet are airborne (jump).
    """
    bvh_floor_offset = scene.get("bvh_floor_offset", 0.0)

    # --- velocity tracking ---
    if "_foot_positions" not in scene:
        scene["_foot_positions"] = {}
    last_positions = scene["_foot_positions"]

    active_contacts: list[str] = []
    foot_velocities: dict[str, float] = {}

    for f_name in (settings.foot_l_name, settings.foot_r_name):
        bvh_bone = bvh_obj.pose.bones.get(f_name)
        if not bvh_bone:
            continue

        foot_world = (bvh_obj.matrix_world @ bvh_bone.matrix).to_translation()
        foot_height = foot_world.z - bvh_floor_offset

        # velocity (frame-to-frame displacement)
        if f_name in last_positions:
            vel = (foot_world - mathutils.Vector(last_positions[f_name])).length
        else:
            vel = 0.0
        foot_velocities[f_name] = vel
        last_positions[f_name] = foot_world.copy()

        if foot_height <= settings.jump_threshold:
            active_contacts.append(f_name)

    # --- anchor selection ---
    last_anchor = scene.get("_active_anchor_name", "")
    anchor_bone_name = ""
    is_switch = False

    if last_anchor in active_contacts:
        # Current anchor still grounded — keep it unless the other foot is
        # significantly more stable (< 50 % velocity AND < 1 cm/frame).
        anchor_bone_name = last_anchor
        cur_vel = foot_velocities.get(last_anchor, 0.0)
        for other in active_contacts:
            if other != last_anchor:
                other_vel = foot_velocities.get(other, 0.0)
                if other_vel < cur_vel * 0.5 and other_vel < 0.01:
                    anchor_bone_name = other
                    is_switch = True
                    break

    elif active_contacts:
        # Previous anchor lifted — pick the stillest grounded foot.
        anchor_bone_name = min(
            active_contacts,
            key=lambda f: foot_velocities.get(f, float("inf")),
        )
        is_switch = last_anchor != anchor_bone_name and last_anchor != ""

    # else: both feet airborne → anchor_bone_name stays ""

    return anchor_bone_name, is_switch


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
            val = apply_continuity_correction(
                val, prev_val, settings.max_jump_threshold
            )
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
                val = apply_continuity_correction(
                    val, prev_val, settings.max_jump_threshold
                )
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
    Frame-change handler: single-pass BVH-to-URDF retargeting with foot planting.

    Pipeline per frame:
        1. Set URDF root = BVH_delta + persistent_correction + manual offset
        2. Set URDF root rotation from BVH (axis-corrected, damped)
        3. Joint retargeting (FK)
        4. Foot alignment (flatten near ground)
        5. scene update
        6. Detect stance foot  →  measure URDF foot world position
        7. Drift = foot_xy − anchor_xy  →  correct root, update persistent_correction
        8. Anti-sinking guard

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

    # Smoothing cache (shared across joints)
    if "_bvh_smooth_cache" not in scene:
        scene["_bvh_smooth_cache"] = {}
    smooth_cache = scene["_bvh_smooth_cache"]

    if not bvh.pose.bones:
        return

    # ------------------------------------------------------------------
    # Reset state at the first frame to avoid drift from previous plays
    # ------------------------------------------------------------------
    current_frame = scene.frame_current
    if current_frame == 0 or current_frame == scene.frame_start:
        scene["_persistent_foot_correction"] = mathutils.Vector((0, 0, 0))
        scene["_active_anchor_name"] = ""
        scene["_anchor_world_pos_xy"] = None
        scene["_foot_positions"] = {}

    # ------------------------------------------------------------------
    # 1.  BVH root position  (scaled delta from reference)
    # ------------------------------------------------------------------
    bvh_root_mat = bvh.matrix_world @ bvh.pose.bones[0].matrix
    bvh_offset = mathutils.Vector(settings.bvh_position_offset)
    current_bvh_pos = bvh_root_mat.to_translation() + bvh_offset
    ref_pos = mathutils.Vector(scene.get("ref_root_pos", current_bvh_pos))
    bvh_floor_offset = scene.get("bvh_floor_offset", 0.0)

    corrected_z = current_bvh_pos.z - bvh_floor_offset
    ref_z = ref_pos.z - bvh_floor_offset

    bvh_delta = mathutils.Vector(
        (
            (current_bvh_pos.x - ref_pos.x) * settings.root_scale,
            (current_bvh_pos.y - ref_pos.y) * settings.root_scale,
            (corrected_z - ref_z) * settings.root_scale,
        )
    )

    # Persistent correction carries the accumulated foot-pinning offset
    persistent_correction = mathutils.Vector(
        scene.get("_persistent_foot_correction", (0, 0, 0))
    )

    offset_xy = settings.location_offset
    loc_offset = mathutils.Vector(
        (offset_xy[0], offset_xy[1], scene.get("urdf_height_offset", 0.0))
    )

    urdf.location = bvh_delta + persistent_correction + loc_offset

    # ------------------------------------------------------------------
    # 2.  BVH root rotation  (axis-corrected, pitch/roll damped)
    # ------------------------------------------------------------------
    ref_rot = mathutils.Quaternion(scene.get("ref_root_rot", (1, 0, 0, 0)))
    off_q = mathutils.Euler(settings.rotation_offset).to_quaternion()
    current_bvh_rot = bvh_root_mat.to_quaternion()

    delta_rot = current_bvh_rot @ ref_rot.inverted()
    delta_rot.x *= -1
    delta_rot.y *= -1

    delta_rot_euler = delta_rot.to_euler()
    damping_xy = settings.root_scale

    delta_rot = mathutils.Euler(
        (
            delta_rot_euler.x * damping_xy,
            delta_rot_euler.y * damping_xy,
            delta_rot_euler.z,
        ),
        "XYZ",
    ).to_quaternion()

    urdf.rotation_mode = "QUATERNION"
    urdf.rotation_quaternion = off_q @ delta_rot

    # ------------------------------------------------------------------
    # 3.  Joint retargeting  (FK)
    # ------------------------------------------------------------------
    for item in settings.mappings:
        apply_joint_retargeting(urdf, bvh, item, smooth_cache, settings, scene)

    # ------------------------------------------------------------------
    # 4.  Foot alignment  (flatten near ground)
    # ------------------------------------------------------------------
    apply_foot_alignment(urdf, bvh, settings, smooth_cache, scene)

    # ------------------------------------------------------------------
    # 5.  Scene update so matrix_world is current for the measurement step
    # ------------------------------------------------------------------
    bpy.context.view_layer.update()

    # ------------------------------------------------------------------
    # 6.  Detect stance foot  (BVH space)
    # ------------------------------------------------------------------
    anchor_bone_name, is_anchor_switch = detect_stance_foot(scene, bvh, settings)

    last_anchor = scene.get("_active_anchor_name", "")
    is_first_anchor = anchor_bone_name != "" and last_anchor == ""

    # ------------------------------------------------------------------
    # 7.  Sticky-foot correction  (single pass, 100 % strength)
    # ------------------------------------------------------------------
    if anchor_bone_name:
        # Find URDF foot bone that corresponds to the BVH anchor
        urdf_foot_bone = None
        for item in settings.mappings:
            if item.bvh_bone_name == anchor_bone_name:
                if item.urdf_bones:
                    urdf_foot_name = item.urdf_bones[-1].urdf_bone_name
                    urdf_foot_bone = urdf.pose.bones.get(urdf_foot_name)
                break

        if urdf_foot_bone:
            foot_world = (urdf.matrix_world @ urdf_foot_bone.matrix).to_translation()

            if is_anchor_switch or is_first_anchor:
                # ---  New anchor  ---
                # Record where the foot IS right now (already includes all
                # previous corrections via persistent_correction).
                # No drift correction this frame → seamless transition.
                scene["_anchor_world_pos_xy"] = (foot_world.x, foot_world.y)

            # Retrieve anchor target (may have just been set above)
            anchor_xy = scene.get("_anchor_world_pos_xy", None)

            if anchor_xy is not None:
                anchor_x, anchor_y = anchor_xy

                drift_x = foot_world.x - anchor_x
                drift_y = foot_world.y - anchor_y

                # Correct root 100 % — translation is additive, so this is exact
                urdf.location.x -= drift_x
                urdf.location.y -= drift_y

                # Write correction back so it carries into next frame
                persistent_correction.x -= drift_x
                persistent_correction.y -= drift_y
                scene["_persistent_foot_correction"] = persistent_correction

        scene["_active_anchor_name"] = anchor_bone_name

    else:
        # Both feet airborne (jump) — freeze correction, clear anchor.
        # persistent_correction is NOT modified, so the offset from the
        # last stance phase is preserved and landing is seamless.
        scene["_active_anchor_name"] = ""
        scene["_anchor_world_pos_xy"] = None

    # ------------------------------------------------------------------
    # 8.  Anti-sinking guard
    # ------------------------------------------------------------------
    bpy.context.view_layer.update()
    current_min_z = get_lowest_z_world(urdf)

    if current_min_z < -1e-4:
        urdf.location.z += abs(current_min_z)
