"""
Motion retargeting engine for BVH-to-URDF animation mapping.

Provides the core retargeting handler that synchronizes BVH motion capture
data to URDF robot kinematics, including root motion projection,
joint retargeting, foot alignment, and anti-sinking corrections.

Foot-planting uses a single-pass "persistent correction" approach:
  1. Root is positioned at BVH_delta + persistent_correction + manual offset.
     Root rotation includes a persistent yaw correction quaternion.
  2. Full FK (joint angles + foot alignment) is applied, then the scene is updated.
  3. The world yaw and XY position of the stance foot are compared against
     a fixed anchor.  Yaw comparison is BVH-relative: only the FK error
     (URDF yaw change minus BVH yaw change since anchor) is corrected,
     so intentional foot rotation from the motion-capture is preserved.
  4. Because persistent_rot_correction is already applied in step 1, the
     measurement captures only the residual FK error.  This residual is
     added as an increment to persistent_rot_correction.  The approach is
     self-correcting: if the FK error vanishes (pose returns to neutral)
     the residual flips sign and decays the correction back to identity.
  5. The yaw correction pivots around the stance foot so the foot stays in
     place.  Any remaining XY drift is subtracted from the root.
  6. On stance-foot switch the NEW foot's current (already-corrected) world
     position AND yaw become the anchor — no jump, no fade required.
  7. During airborne phases (jump) persistent corrections are frozen; on
     landing the first grounded foot sets a fresh anchor.
  8. A configurable per-frame decay (``correction_decay``) gradually
     pulls persistent corrections back toward zero.  Anchors are shifted
     by the same amount so foot-planting is not destabilised.  This
     ensures the overall URDF trajectory stays close to the BVH path,
     which is critical for downstream transfer-learning (BeyondMimic).
"""

import bpy
import math
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


# Axis name → unit vector lookup (avoids repeated if/elif chains)
_AXIS_VECTORS = {
    "X": mathutils.Vector((1, 0, 0)),
    "Y": mathutils.Vector((0, 1, 0)),
    "Z": mathutils.Vector((0, 0, 1)),
}


def _extract_yaw(matrix) -> float:
    """
    Extract yaw (rotation around world Z) from a 4×4 matrix.

    Uses ``atan2`` on the matrix's local X-axis projected onto the
    world XY-plane.  This avoids Euler-decomposition gimbal
    instability that occurs when foot pitch approaches ±90°.
    """
    q = matrix.to_quaternion()
    forward = q @ mathutils.Vector((1, 0, 0))
    return math.atan2(forward.y, forward.x)


def _wrap_angle(angle: float) -> float:
    """Wrap *angle* to the [-π, π] range."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _apply_final_angle(
    urdf_bone,
    angle: float,
    joint_type: str,
) -> None:
    """
    Clamp *angle* to joint limits (if revolute), store it as a custom
    property, and write it into the bone's quaternion rotation.

    Args:
        urdf_bone: The URDF pose-bone to update.
        angle:     Target joint angle in radians.
        joint_type: "revolute", "continuous", etc.
    """
    if joint_type != "continuous":
        l_min = get_bone_property(urdf_bone, "limit_lower", -3.14)
        l_max = get_bone_property(urdf_bone, "limit_upper", 3.14)
        angle = clamp_to_limits(angle, l_min, l_max)

    set_bone_property(urdf_bone, "_joint_angle", angle)
    urdf_bone.rotation_mode = "QUATERNION"
    urdf_bone.rotation_quaternion = mathutils.Quaternion((0, 1, 0), angle)


def _find_urdf_foot_for_anchor(
    anchor_bone_name: str,
    settings,
    urdf_obj: bpy.types.Object,
):
    """
    Look up the URDF pose-bone that corresponds to a BVH anchor foot.

    Returns the last bone in the mapping chain (the actual foot/toe tip)
    or *None* if no mapping exists.
    """
    for item in settings.mappings:
        if item.bvh_bone_name == anchor_bone_name and item.urdf_bones:
            name = item.urdf_bones[-1].urdf_bone_name
            return urdf_obj.pose.bones.get(name)
    return None


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
) -> tuple[str, bool, bool]:
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
        (anchor_bone_name, is_anchor_switch, both_grounded)
        anchor_bone_name is "" when both feet are airborne (jump).
        both_grounded is True when both feet are in contact with the ground
        (double-support phase).  During double-support, yaw rotation
        correction should be suppressed because pivoting around one foot
        would move the other.
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

    both_grounded = len(active_contacts) >= 2

    return anchor_bone_name, is_switch, both_grounded


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

        # Extract rotation around the selected twist axis
        twist_axis = _AXIS_VECTORS.get(
            urdf_bone_mapping.source_axis, _AXIS_VECTORS["Z"]
        )
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

        # Clamp, store and apply
        _apply_final_angle(urdf_b, final_angle, jtype)


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

            # Get foot world transform
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
            current_foot_q = foot_mat_world.to_quaternion()

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

            # Blend with current angle, clamp, store and apply
            current_joint_angle = get_bone_property(urdf_b, "_joint_angle", 0.0)
            _apply_final_angle(urdf_b, current_joint_angle + val * influence, jtype)


@persistent
def retarget_frame(scene: bpy.types.Scene) -> None:
    """
    Frame-change handler: single-pass BVH-to-URDF retargeting with foot planting.

    Pipeline per frame:
        1. Set URDF root = BVH_delta + persistent_correction + manual offset
        2. Set URDF root rotation from BVH (axis-corrected, damped)
           + persistent yaw correction
        3. Joint retargeting (FK)
        4. Foot alignment (flatten near ground)
        5. scene update
        6. Detect stance foot  →  measure URDF foot world transform
        7a. Yaw residual  →  increment persistent yaw, pivot (single-support)
                              or rotate-in-place (double-support)
        7b. XY drift      →  correct root, update persistent translation
        7c. Correction decay → pull persistent offsets toward BVH
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
        scene["_persistent_foot_rot_correction"] = mathutils.Quaternion((1, 0, 0, 0))
        scene["_active_anchor_name"] = ""
        scene["_anchor_world_pos_xy"] = None
        scene["_anchor_world_yaw"] = None
        scene["_anchor_bvh_yaw"] = None
        scene["_foot_positions"] = {}

    # ------------------------------------------------------------------
    # 1.  BVH root position  (scaled delta from reference)
    # ------------------------------------------------------------------
    bvh_root_mat = bvh.matrix_world @ bvh.pose.bones[0].matrix
    current_bvh_pos = bvh_root_mat.to_translation()
    ref_pos = mathutils.Vector(scene.get("ref_root_pos", current_bvh_pos))

    bvh_delta = mathutils.Vector(
        (
            (current_bvh_pos.x - ref_pos.x) * settings.root_scale,
            (current_bvh_pos.y - ref_pos.y) * settings.root_scale,
            (current_bvh_pos.z - ref_pos.z) * settings.root_scale,
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

    persistent_rot_correction = mathutils.Quaternion(
        scene.get("_persistent_foot_rot_correction", (1, 0, 0, 0))
    )

    urdf.rotation_mode = "QUATERNION"
    urdf.rotation_quaternion = persistent_rot_correction @ off_q @ delta_rot

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
    anchor_bone_name, is_anchor_switch, both_grounded = detect_stance_foot(
        scene, bvh, settings
    )

    last_anchor = scene.get("_active_anchor_name", "")
    is_first_anchor = anchor_bone_name != "" and last_anchor == ""

    # ------------------------------------------------------------------
    # 7.  Sticky-foot correction  (yaw rotation + XY translation)
    # ------------------------------------------------------------------
    if anchor_bone_name:
        urdf_foot_bone = _find_urdf_foot_for_anchor(anchor_bone_name, settings, urdf)

        if urdf_foot_bone:
            foot_world_mat = urdf.matrix_world @ urdf_foot_bone.matrix
            foot_world = foot_world_mat.to_translation()
            foot_yaw = _extract_yaw(foot_world_mat)

            if is_anchor_switch or is_first_anchor:
                # ---  New anchor  ---
                # Record where the foot IS right now (already includes all
                # previous corrections via persistent_correction).
                # No drift correction this frame → seamless transition.
                scene["_anchor_world_pos_xy"] = (foot_world.x, foot_world.y)
                scene["_anchor_world_yaw"] = foot_yaw

                # Also record the BVH foot yaw at anchor time so we can
                # distinguish intentional rotation from FK error later.
                bvh_anchor_bone = bvh.pose.bones.get(anchor_bone_name)
                if bvh_anchor_bone:
                    bvh_foot_mat = bvh.matrix_world @ bvh_anchor_bone.matrix
                    scene["_anchor_bvh_yaw"] = _extract_yaw(bvh_foot_mat)
                else:
                    scene["_anchor_bvh_yaw"] = None

            # --- 7a.  Rotation correction (yaw / Z-axis) ---
            #
            # Persistent, residual-based yaw correction.
            # Because persistent_rot_correction was already applied in
            # step 2, the yaw we measure here already includes all
            # previous corrections.  We therefore only see the RESIDUAL
            # FK error and increment persistent_rot_correction by that
            # small amount.  This is self-correcting: when the FK error
            # vanishes (pose returns to neutral), the residual has the
            # opposite sign and decays the accumulated correction back
            # toward identity.  No unbounded growth, no flicker.
            #
            # The correction is ALWAYS active (single- AND double-support)
            # so that yaw drift cannot accumulate in any ground phase.
            # • Single-support: pivot around the stance foot so it stays
            #   in place and only the root swings.
            # • Double-support: rotate root in place (no pivot).  Step 7b
            #   then re-pins the anchor foot via XY translation.  The
            #   non-anchor foot may shift minimally, which is acceptable
            #   for the typically short double-support phases.
            #
            # Yaw is extracted via atan2 (gimbal-safe) and differences
            # are wrapped to [-π, π] to avoid 2π jumps.
            anchor_yaw = scene.get("_anchor_world_yaw", None)
            if anchor_yaw is not None:
                # How much the URDF foot yaw changed since anchor
                urdf_yaw_change = _wrap_angle(foot_yaw - anchor_yaw)

                # How much the BVH foot yaw changed since anchor
                # (= intentional rotation from the motion-capture data)
                bvh_yaw_change = 0.0
                anchor_bvh_yaw = scene.get("_anchor_bvh_yaw", None)
                if anchor_bvh_yaw is not None:
                    bvh_anchor_bone = bvh.pose.bones.get(anchor_bone_name)
                    if bvh_anchor_bone:
                        cur_bvh_yaw = _extract_yaw(
                            bvh.matrix_world @ bvh_anchor_bone.matrix
                        )
                        bvh_yaw_change = _wrap_angle(cur_bvh_yaw - anchor_bvh_yaw)

                # Residual FK error (what remains after persistent correction)
                yaw_residual = _wrap_angle(urdf_yaw_change - bvh_yaw_change)

                if abs(yaw_residual) > 1e-6:
                    yaw_fix_q = mathutils.Quaternion((0, 0, 1), -yaw_residual)

                    # --- Rotate root orientation ---
                    urdf.rotation_quaternion = yaw_fix_q @ urdf.rotation_quaternion

                    if not both_grounded:
                        # --- Single-support: Pivot around stance foot ---
                        root_pos = urdf.location.copy()
                        pivot = mathutils.Vector(
                            (foot_world.x, foot_world.y, root_pos.z)
                        )
                        offset_from_pivot = root_pos - pivot
                        rotated_offset = yaw_fix_q @ offset_from_pivot
                        urdf.location = pivot + rotated_offset

                        # Persist pivot-induced position shift
                        loc_shift = rotated_offset - offset_from_pivot
                        persistent_correction += loc_shift
                        scene["_persistent_foot_correction"] = persistent_correction
                    # else: double-support — rotate in place, 7b re-pins

                    # Accumulate into persistent correction for next frame
                    persistent_rot_correction = yaw_fix_q @ persistent_rot_correction
                    scene["_persistent_foot_rot_correction"] = persistent_rot_correction

                    # Scene must update after rotation change so that
                    # foot position reflects the new orientation before
                    # we measure XY drift.
                    bpy.context.view_layer.update()

                    # Re-measure foot position after rotation correction
                    foot_world = (
                        urdf.matrix_world @ urdf_foot_bone.matrix
                    ).to_translation()

            # --- 7b.  Translation correction (XY) ---
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
        # persistent corrections (translation + rotation) are NOT modified,
        # so the offsets from the last stance phase are preserved and
        # landing is seamless.
        scene["_active_anchor_name"] = ""
        scene["_anchor_world_pos_xy"] = None
        scene["_anchor_world_yaw"] = None
        scene["_anchor_bvh_yaw"] = None

    # ------------------------------------------------------------------
    # 7c.  Correction decay — pull persistent offsets toward BVH
    #
    # Without decay the persistent corrections accumulate without
    # bound, causing the URDF root to drift away from the BVH
    # trajectory.  A small per-frame decay acts as a leaky
    # integrator: foot-planting adds corrections, decay removes
    # them.  The anchors are shifted by the same amount so the
    # foot-planting loop does not immediately re-introduce what
    # was decayed — the net effect is a smooth pull back toward
    # the original BVH motion path.
    # ------------------------------------------------------------------
    target_hz = scene.render.fps if scene.render.fps > 0 else 120.0
    _REF_FPS = 120.0
    # UI value is a readable percentage-like number (default 0.05).
    # Divide by 10 to obtain the actual per-frame rate at 120 FPS
    # (0.05 → 0.005 per frame).
    raw_decay = settings.correction_decay / 10.0
    # FPS-independent decay: the configured rate is defined at 120 FPS.
    # At other frame-rates we adjust so the per-second decay stays equal.
    #   (1 - adjusted)^fps == (1 - raw_decay)^120
    #   adjusted = 1 - (1 - raw_decay)^(120 / fps)
    if raw_decay >= 0.1:
        decay_rate = 1.0
    elif raw_decay > 0.0:
        decay_rate = 1.0 - (1.0 - raw_decay) ** (_REF_FPS / target_hz)
    else:
        decay_rate = 0.0
    if decay_rate > 0.0:
        # Re-read (step 7 may have modified them)
        persistent_correction = mathutils.Vector(
            scene.get("_persistent_foot_correction", (0, 0, 0))
        )
        persistent_rot_correction = mathutils.Quaternion(
            scene.get("_persistent_foot_rot_correction", (1, 0, 0, 0))
        )

        # --- Translation decay (XY only, Z handled by anti-sinking) ---
        decay_x = persistent_correction.x * decay_rate
        decay_y = persistent_correction.y * decay_rate
        persistent_correction.x -= decay_x
        persistent_correction.y -= decay_y

        anchor_xy = scene.get("_anchor_world_pos_xy", None)
        if anchor_xy is not None:
            scene["_anchor_world_pos_xy"] = (
                anchor_xy[0] - decay_x,
                anchor_xy[1] - decay_y,
            )

        # --- Rotation decay (yaw around Z) ---
        identity_q = mathutils.Quaternion((1, 0, 0, 0))
        old_yaw = 2.0 * math.atan2(
            persistent_rot_correction.z, persistent_rot_correction.w
        )
        persistent_rot_correction = persistent_rot_correction.slerp(
            identity_q, decay_rate
        )
        new_yaw = 2.0 * math.atan2(
            persistent_rot_correction.z, persistent_rot_correction.w
        )
        yaw_removed = old_yaw - new_yaw

        anchor_yaw = scene.get("_anchor_world_yaw", None)
        if anchor_yaw is not None:
            scene["_anchor_world_yaw"] = anchor_yaw - yaw_removed

        # Store decayed values
        scene["_persistent_foot_correction"] = persistent_correction
        scene["_persistent_foot_rot_correction"] = persistent_rot_correction

    # ------------------------------------------------------------------
    # 8.  Anti-sinking guard
    # ------------------------------------------------------------------
    bpy.context.view_layer.update()
    current_min_z = get_lowest_z_world(urdf)

    if current_min_z < -1e-4:
        urdf.location.z += abs(current_min_z)
