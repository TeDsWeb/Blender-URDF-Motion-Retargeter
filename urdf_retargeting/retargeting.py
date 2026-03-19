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
import hashlib
import time
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


def _get_retargeting_mode(settings) -> str:
    """Return normalized retargeting mode with legacy value compatibility."""
    raw_mode = settings.get("retargeting_method", None)
    if raw_mode is None:
        return "HYBRID"

    mode_str = str(raw_mode)
    legacy_map = {
        "0": "FK_ONLY",
        "1": "IK_ONLY",
        "2": "HYBRID",
    }
    normalized = legacy_map.get(mode_str, mode_str)

    if normalized not in {"FK_ONLY", "IK_ONLY", "HYBRID"}:
        normalized = "HYBRID"

    # Persist migration so Blender no longer keeps legacy enum identifiers.
    if mode_str != normalized:
        try:
            settings["retargeting_method"] = normalized
        except Exception:
            pass

    return normalized


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

    for chain in settings.kinematic_chains:
        if chain.bvh_target_bone_name == anchor_bone_name and chain.urdf_end_bone_name:
            return urdf_obj.pose.bones.get(chain.urdf_end_bone_name)

    return None


def _get_pose_bone_world_position(
    armature_obj: bpy.types.Object,
    pose_bone: bpy.types.PoseBone,
) -> mathutils.Vector:
    """Return the world-space joint position of a pose bone."""
    return (armature_obj.matrix_world @ pose_bone.matrix).to_translation()


def _get_pose_bone_world_axis(
    armature_obj: bpy.types.Object,
    pose_bone: bpy.types.PoseBone,
) -> mathutils.Vector:
    """Return the world-space joint axis for a URDF pose bone."""
    bone_world_q = (armature_obj.matrix_world @ pose_bone.matrix).to_quaternion()
    axis = bone_world_q @ mathutils.Vector((0, 1, 0))
    if axis.length_squared == 0.0:
        return mathutils.Vector((0, 1, 0))
    return axis.normalized()


def _build_urdf_chain(
    urdf_obj: bpy.types.Object,
    root_bone_name: str,
    end_bone_name: str,
) -> list[bpy.types.PoseBone]:
    """Build a serial URDF pose-bone chain from root to end bone."""
    root_bone = urdf_obj.pose.bones.get(root_bone_name)
    end_bone = urdf_obj.pose.bones.get(end_bone_name)
    if not root_bone or not end_bone:
        return []

    chain = []
    current = end_bone
    while current is not None:
        chain.append(current)
        if current.name == root_bone.name:
            return list(reversed(chain))
        current = current.parent

    return []


def _signed_angle_around_axis(
    source: mathutils.Vector,
    target: mathutils.Vector,
    axis: mathutils.Vector,
) -> float:
    """Return signed angle from source to target around axis."""
    cross = source.cross(target)
    dot = max(-1.0, min(1.0, source.dot(target)))
    return math.atan2(axis.dot(cross), dot)


def _smooth_target_position(
    cache: dict,
    cache_key: str,
    target_world: mathutils.Vector,
    smoothing_factor: float,
    initial_value: mathutils.Vector | None = None,
) -> mathutils.Vector:
    """Apply EMA-like smoothing to world-space IK targets."""
    if cache_key not in cache and initial_value is not None:
        cache[cache_key] = initial_value.copy()

    if cache_key in cache:
        last_target = mathutils.Vector(cache[cache_key])
        # Keep a small minimum alpha so the target never fully freezes,
        # even if the user sets smoothing close to 1.0.
        alpha = max(0.02, 1.0 - smoothing_factor)
        target_world = last_target.lerp(target_world, alpha)

    cache[cache_key] = target_world.copy()
    return target_world


def _compute_adaptive_step_limit(
    base_limit: float,
    residual_abs: float,
    adaptive_gain: float,
    min_scale: float = 0.35,
    max_scale: float = 3.0,
) -> float:
    """
    Compute a smooth residual-dependent step limit.

    Compared to hard clamping with a fixed per-frame step, this produces
    smaller steps near zero residual (less visible jitter) and larger steps
    for large residuals (faster convergence without sudden mode switches).
    """
    base = max(0.0, float(base_limit))
    if base <= 0.0:
        return 0.0

    residual = max(0.0, float(residual_abs))
    gain = max(0.0, float(adaptive_gain))

    norm = residual / max(base, 1e-9)
    response = 1.0 - math.exp(-gain * norm)
    scale = min_scale + (max_scale - min_scale) * response
    return base * scale


def _compute_joint_path_length(
    points: list[mathutils.Vector],
    collapse_epsilon: float = 1e-6,
) -> float:
    """Sum distances between consecutive unique joint positions."""
    if len(points) < 2:
        return 0.0

    total = 0.0
    last_point = points[0]
    for point in points[1:]:
        step = (point - last_point).length
        if step <= collapse_epsilon:
            continue
        total += step
        last_point = point
    return total


def _short_cache_id(*parts: str) -> str:
    """Return a short deterministic IDProperty-safe key suffix."""
    payload = "|".join(parts).encode("utf-8", errors="ignore")
    return hashlib.blake2b(payload, digest_size=8).hexdigest()


def _get_custom_default_joint_angles(settings) -> dict[str, float]:
    """Return stored custom default joint angles keyed by URDF joint name."""
    return {
        item.joint_name: item.angle
        for item in getattr(settings, "default_pose_joints", [])
    }


def _get_custom_default_root_quaternion(settings) -> mathutils.Quaternion:
    """Return the stored custom default root rotation or identity."""
    root_rotation = getattr(settings, "default_pose_root_rotation", (0.0, 0.0, 0.0))
    # UI intentionally exposes only roll/pitch for neutral pose root rotation.
    # Force yaw to zero so stored settings cannot introduce hidden heading.
    root_euler = mathutils.Euler((root_rotation[0], root_rotation[1], 0.0), "XYZ")
    return root_euler.to_quaternion()


def _compute_custom_default_end_local(
    urdf_obj: bpy.types.Object,
    settings,
    end_bone_name: str,
) -> mathutils.Vector | None:
    """Evaluate end-effector local position in custom default pose."""
    end_bone = urdf_obj.pose.bones.get(end_bone_name)
    if not end_bone:
        return None

    joint_defaults = _get_custom_default_joint_angles(settings)

    prev_root_q = urdf_obj.rotation_quaternion.copy()
    prev_bone_q = {}

    for pb in urdf_obj.pose.bones:
        prev_bone_q[pb.name] = pb.rotation_quaternion.copy()

    try:
        urdf_obj.rotation_mode = "QUATERNION"
        urdf_obj.rotation_quaternion = _get_custom_default_root_quaternion(settings)

        for joint_name, angle in joint_defaults.items():
            pb = urdf_obj.pose.bones.get(joint_name)
            if not pb:
                continue

            jtype = get_bone_property(pb, "joint_type", "revolute")
            if jtype not in {"revolute", "continuous"}:
                continue

            if jtype != "continuous":
                l_min = get_bone_property(pb, "limit_lower", -3.14)
                l_max = get_bone_property(pb, "limit_upper", 3.14)
                angle = clamp_to_limits(angle, l_min, l_max)

            pb.rotation_mode = "QUATERNION"
            pb.rotation_quaternion = mathutils.Quaternion((0, 1, 0), angle)

        bpy.context.view_layer.update()
        return end_bone.matrix.to_translation().copy()
    finally:
        urdf_obj.rotation_quaternion = prev_root_q
        for pb in urdf_obj.pose.bones:
            if pb.name in prev_bone_q:
                pb.rotation_quaternion = prev_bone_q[pb.name]
        bpy.context.view_layer.update()


def _compute_custom_default_chain_reference(
    urdf_obj: bpy.types.Object,
    settings,
    solver_bones: list[bpy.types.PoseBone],
    end_bone: bpy.types.PoseBone,
) -> dict | None:
    """
    Capture chain-local neutral pose reference for FABRIK-C.

    Returns local-space points, segment lengths, hinge axes and base segment
    directions (all in armature-object space) so runtime solving can avoid
    repeated scene updates and pose baking.
    """
    if not solver_bones:
        return None

    joint_defaults = _get_custom_default_joint_angles(settings)

    prev_root_q = urdf_obj.rotation_quaternion.copy()
    prev_bone_q = {}
    for pb in urdf_obj.pose.bones:
        prev_bone_q[pb.name] = pb.rotation_quaternion.copy()

    try:
        urdf_obj.rotation_mode = "QUATERNION"
        urdf_obj.rotation_quaternion = _get_custom_default_root_quaternion(settings)

        for joint_name, angle in joint_defaults.items():
            pb = urdf_obj.pose.bones.get(joint_name)
            if not pb:
                continue

            jtype = get_bone_property(pb, "joint_type", "revolute")
            if jtype not in {"revolute", "continuous"}:
                continue

            if jtype != "continuous":
                l_min = get_bone_property(pb, "limit_lower", -3.14)
                l_max = get_bone_property(pb, "limit_upper", 3.14)
                angle = clamp_to_limits(angle, l_min, l_max)

            pb.rotation_mode = "QUATERNION"
            pb.rotation_quaternion = mathutils.Quaternion((0, 1, 0), angle)

        bpy.context.view_layer.update()

        points_local = [pb.matrix.to_translation().copy() for pb in solver_bones]
        points_local.append(end_bone.matrix.to_translation().copy())

        hinge_axes_local = []
        for pb in solver_bones:
            axis = pb.matrix.to_quaternion() @ mathutils.Vector((0, 1, 0))
            if axis.length_squared < 1e-12:
                axis = mathutils.Vector((0, 1, 0))
            else:
                axis = axis.normalized()
            hinge_axes_local.append(axis)

        segment_lengths = []
        base_dirs_local = []
        for i in range(len(points_local) - 1):
            seg = points_local[i + 1] - points_local[i]
            seg_len = max(1e-9, seg.length)
            segment_lengths.append(seg_len)
            if seg.length_squared < 1e-12:
                base_dirs_local.append(mathutils.Vector((1, 0, 0)))
            else:
                base_dirs_local.append(seg.normalized())

        return {
            "points_local": [tuple(v) for v in points_local],
            "hinge_axes_local": [tuple(v) for v in hinge_axes_local],
            "base_dirs_local": [tuple(v) for v in base_dirs_local],
            "segment_lengths": segment_lengths,
        }
    finally:
        urdf_obj.rotation_quaternion = prev_root_q
        for pb in urdf_obj.pose.bones:
            if pb.name in prev_bone_q:
                pb.rotation_quaternion = prev_bone_q[pb.name]
        bpy.context.view_layer.update()


def _ensure_joint_angles_initialized(
    urdf_obj: bpy.types.Object,
    scene: bpy.types.Scene,
) -> None:
    """
    Ensure all revolute/continuous joints have initial angle values.

    For IK-only mode, this guarantees that _joint_angle exists before
    any FK or IK operations. Extracts angles from quaternion if needed.

    Args:
        urdf_obj: The URDF armature object.
        scene: The Blender scene.
    """
    if scene.get("_joint_angles_initialized", False):
        return

    for pose_bone in urdf_obj.pose.bones:
        jtype = get_bone_property(pose_bone, "joint_type", "revolute")
        if jtype not in {"revolute", "continuous"}:
            continue

        # Check if _joint_angle already exists
        if "_joint_angle" in pose_bone:
            continue

        # Try to use _last_urdf_angle as baseline
        last_angle = get_bone_property(pose_bone, "_last_urdf_angle", None)
        if last_angle is not None:
            set_bone_property(pose_bone, "_joint_angle", last_angle)
        else:
            # Extract angle from current quaternion rotation
            q = pose_bone.rotation_quaternion
            # For a single-axis rotation around Y (standard hinge)
            axis_name = get_bone_property(pose_bone, "axis", "Y")
            axis = _AXIS_VECTORS.get(axis_name, _AXIS_VECTORS["Y"])

            # Extract twist angle around the joint axis
            extracted_angle = extract_twist_angle(q, axis)

            set_bone_property(pose_bone, "_joint_angle", extracted_angle)
            set_bone_property(pose_bone, "_last_urdf_angle", extracted_angle)

    scene["_joint_angles_initialized"] = True


def _apply_hinge_direction_constraint(
    raw_dir: mathutils.Vector,
    axis_world: mathutils.Vector,
    base_dir_world: mathutils.Vector,
) -> mathutils.Vector:
    """
    Constrain a segment direction to a revolute (hinge) manifold.

    The constrained direction preserves the base direction's component
    along the hinge axis and rotates only inside the perpendicular plane.
    """
    if raw_dir.length_squared < 1e-12:
        return base_dir_world.normalized()

    axis = (
        axis_world.normalized()
        if axis_world.length_squared > 1e-12
        else mathutils.Vector((0, 1, 0))
    )
    base = (
        base_dir_world.normalized()
        if base_dir_world.length_squared > 1e-12
        else mathutils.Vector((1, 0, 0))
    )
    raw = raw_dir.normalized()

    base_axis_component = clamp_to_limits(base.dot(axis), -1.0, 1.0)
    base_perp_len = math.sqrt(max(0.0, 1.0 - base_axis_component * base_axis_component))

    raw_perp = raw - axis * raw.dot(axis)
    if raw_perp.length_squared < 1e-12:
        base_perp = base - axis * base_axis_component
        if base_perp.length_squared < 1e-12:
            perp_axis = axis.cross(mathutils.Vector((1, 0, 0)))
            if perp_axis.length_squared < 1e-12:
                perp_axis = axis.cross(mathutils.Vector((0, 0, 1)))
            raw_perp_dir = perp_axis.normalized()
        else:
            raw_perp_dir = base_perp.normalized()
    else:
        raw_perp_dir = raw_perp.normalized()

    constrained = (axis * base_axis_component) + (raw_perp_dir * base_perp_len)
    if constrained.length_squared < 1e-12:
        return base
    return constrained.normalized()


def _solve_fabrik_c_positions(
    initial_points: list[mathutils.Vector],
    target_world: mathutils.Vector,
    segment_lengths: list[float],
    max_iterations: int,
    tolerance: float,
    hinge_axes_world: list[mathutils.Vector] | None = None,
    hinge_base_dirs_world: list[mathutils.Vector] | None = None,
    target_orientation: mathutils.Quaternion | None = None,
    orientation_weight: float = 0.0,
) -> tuple[list[mathutils.Vector], int, float]:
    """
    Solve chain positions with FABRIK-C (position solve + hinge constraints).

    This function runs purely in math space. No Blender dependency-graph
    updates are performed inside the iteration loop.
    """
    if len(initial_points) < 2 or len(segment_lengths) != len(initial_points) - 1:
        points = [p.copy() for p in initial_points]
        residual = (points[-1] - target_world).length if points else 0.0
        return points, 0, residual

    points = [p.copy() for p in initial_points]
    n_seg = len(segment_lengths)
    root = points[0].copy()
    total_reach = sum(segment_lengths)

    to_target = target_world - root
    if to_target.length > total_reach and total_reach > 1e-9:
        direction = to_target.normalized()
        for i in range(n_seg):
            if hinge_axes_world is not None and hinge_base_dirs_world is not None:
                direction = _apply_hinge_direction_constraint(
                    direction,
                    hinge_axes_world[i],
                    hinge_base_dirs_world[i],
                )
            points[i + 1] = points[i] + direction * segment_lengths[i]
        residual = (points[-1] - target_world).length
        return points, 1, residual

    tol = max(1e-6, tolerance)
    iterations_used = 0
    for iter_idx in range(max(1, max_iterations)):
        iterations_used = iter_idx + 1
        points[-1] = target_world.copy()

        if target_orientation is not None and orientation_weight > 1e-6 and n_seg >= 1:
            desired_dir = target_orientation @ mathutils.Vector((1, 0, 0))
            if desired_dir.length_squared > 1e-12:
                desired_dir.normalize()
                cur_dir = points[-1] - points[-2]
                if cur_dir.length_squared > 1e-12:
                    cur_dir.normalize()
                else:
                    cur_dir = desired_dir.copy()
                w = clamp_to_limits(orientation_weight, 0.0, 1.0)
                blend_dir = cur_dir.lerp(desired_dir, w)
                if blend_dir.length_squared > 1e-12:
                    blend_dir.normalize()
                    points[-2] = points[-1] - blend_dir * segment_lengths[-1]

        for i in range(n_seg - 1, -1, -1):
            dist = (points[i + 1] - points[i]).length
            if dist < 1e-12:
                continue
            lam = segment_lengths[i] / dist
            proposed_joint = ((1.0 - lam) * points[i + 1]) + (lam * points[i])

            raw_dir = points[i + 1] - proposed_joint
            if hinge_axes_world is not None and hinge_base_dirs_world is not None:
                dir_constrained = _apply_hinge_direction_constraint(
                    raw_dir,
                    hinge_axes_world[i],
                    hinge_base_dirs_world[i],
                )
            elif raw_dir.length_squared > 1e-12:
                dir_constrained = raw_dir.normalized()
            else:
                dir_constrained = mathutils.Vector((1, 0, 0))

            points[i] = points[i + 1] - dir_constrained * segment_lengths[i]

        points[0] = root.copy()

        for i in range(n_seg):
            dist = (points[i + 1] - points[i]).length
            if dist < 1e-12:
                raw_dir = mathutils.Vector((1, 0, 0))
            else:
                lam = segment_lengths[i] / dist
                proposed_child = ((1.0 - lam) * points[i]) + (lam * points[i + 1])
                raw_dir = proposed_child - points[i]

            if hinge_axes_world is not None and hinge_base_dirs_world is not None:
                dir_constrained = _apply_hinge_direction_constraint(
                    raw_dir,
                    hinge_axes_world[i],
                    hinge_base_dirs_world[i],
                )
            elif raw_dir.length_squared > 1e-12:
                dir_constrained = raw_dir.normalized()
            else:
                dir_constrained = mathutils.Vector((1, 0, 0))

            points[i + 1] = points[i] + dir_constrained * segment_lengths[i]

        if (points[-1] - target_world).length <= tol:
            break

    residual = (points[-1] - target_world).length
    return points, iterations_used, residual


def _collect_chain_points_world(
    urdf_obj: bpy.types.Object,
    solver_bones: list[bpy.types.PoseBone],
    end_bone: bpy.types.PoseBone,
) -> list[mathutils.Vector]:
    """Collect world-space points for all solver joints plus end-effector."""
    points = [_get_pose_bone_world_position(urdf_obj, pb) for pb in solver_bones]
    points.append(_get_pose_bone_world_position(urdf_obj, end_bone))
    return points


def apply_custom_default_pose_to_urdf(
    urdf_obj: bpy.types.Object,
    settings,
) -> bool:
    """Apply stored custom default pose values to the current URDF rig."""
    joint_defaults = _get_custom_default_joint_angles(settings)

    urdf_obj.rotation_mode = "QUATERNION"
    urdf_obj.rotation_quaternion = _get_custom_default_root_quaternion(settings)

    for joint_name, target_angle in joint_defaults.items():
        pb = urdf_obj.pose.bones.get(joint_name)
        if not pb:
            continue

        joint_type = get_bone_property(pb, "joint_type", "revolute")
        if joint_type not in {"revolute", "continuous"}:
            continue

        _apply_final_angle(pb, target_angle, joint_type)
        set_bone_property(
            pb, "_last_urdf_angle", get_bone_property(pb, "_joint_angle", 0.0)
        )

    bpy.context.view_layer.update()
    return True


def apply_kinematic_retargeting(
    urdf_obj: bpy.types.Object,
    bvh_obj: bpy.types.Object,
    settings,
    scene: bpy.types.Scene,
    correction_blend: float = 1.0,
    max_iterations_override: int | None = None,
) -> None:
    """Retarget URDF joint chains by solving BVH end-effector targets with FABRIK-C."""
    if "_kinematic_target_cache" not in scene:
        scene["_kinematic_target_cache"] = {}
    target_cache = scene["_kinematic_target_cache"]

    solver_pin_anchor = str(scene.get("_ik_solver_pin_anchor", ""))
    solver_pin_pos = scene.get("_ik_solver_pin_pos", None)
    solver_pin_enabled = bool(solver_pin_anchor and solver_pin_pos is not None)

    total_iters = 0.0
    total_residual = 0.0
    solved_chains = 0
    prop_scale_min = float("inf")
    prop_scale_max = float("-inf")
    motion_delta_z_max = 0.0
    retarget_mode = _get_retargeting_mode(settings)

    for chain in settings.kinematic_chains:
        if not chain.bvh_target_bone_name or not chain.urdf_end_bone_name:
            continue

        bvh_target_bone = bvh_obj.pose.bones.get(chain.bvh_target_bone_name)
        if not bvh_target_bone:
            continue

        full_chain = _build_urdf_chain(
            urdf_obj,
            chain.urdf_root_bone_name,
            chain.urdf_end_bone_name,
        )
        if not full_chain:
            continue

        solver_bones = [
            bone
            for bone in full_chain
            if get_bone_property(bone, "joint_type", "revolute")
            in {"revolute", "continuous"}
        ]
        if not solver_bones:
            continue

        end_bone = full_chain[-1]
        bvh_target_world = _get_pose_bone_world_position(bvh_obj, bvh_target_bone)

        # Per-chain BVH root matrix: use the configured bvh_root_bone_name if set,
        # falling back to the first pose bone (typically the hip/root in BVH imports).
        bvh_chain_root_name = str(getattr(chain, "bvh_root_bone_name", "")).strip()
        bvh_root_bone = (
            bvh_obj.pose.bones.get(bvh_chain_root_name)
            if bvh_chain_root_name
            else None
        ) or bvh_obj.pose.bones[0]
        bvh_root_mat = bvh_obj.matrix_world @ bvh_root_bone.matrix

        target_local = bvh_root_mat.inverted() @ bvh_target_world
        end_world_now = _get_pose_bone_world_position(urdf_obj, end_bone)
        end_local_now = urdf_obj.matrix_world.inverted() @ end_world_now

        chain_key = _short_cache_id(
            bvh_obj.name,
            chain.bvh_target_bone_name,
            getattr(chain, "bvh_root_bone_name", ""),
            chain.urdf_root_bone_name,
            chain.urdf_end_bone_name,
        )
        ref_urdf_key = f"ik_r_u_{chain_key}"
        ref_bvh_key = f"ik_r_b_{chain_key}"

        if ref_urdf_key not in target_cache:
            # Always use neutral pose reference, not current end-effector position
            custom_ref = _compute_custom_default_end_local(
                urdf_obj,
                settings,
                chain.urdf_end_bone_name,
            )
            if custom_ref is not None:
                target_cache[ref_urdf_key] = custom_ref.copy()
            else:
                # Fallback: use current position
                target_cache[ref_urdf_key] = end_local_now.copy()
        if ref_bvh_key not in target_cache:
            target_cache[ref_bvh_key] = target_local.copy()

        ref_urdf_local = mathutils.Vector(target_cache[ref_urdf_key])
        ref_bvh_local = mathutils.Vector(target_cache[ref_bvh_key])

        # Build/cached neutral chain reference before proportional scaling.
        ref_chain_key = f"ik_r_chain_{chain_key}"
        if ref_chain_key not in target_cache:
            ref_data = _compute_custom_default_chain_reference(
                urdf_obj,
                settings,
                solver_bones,
                end_bone,
            )
            if ref_data is not None:
                target_cache[ref_chain_key] = ref_data
        ref_data = target_cache.get(ref_chain_key, None)

        # Compute proportion scale from complete chain profile (not just end-effector radius).
        # This improves arm reach and prevents unnatural stretching/shortening.
        bvh_chain_length = 0.0
        urdf_chain_length = 0.0

        # URDF chain length must follow the neutral motor-to-motor path, not the
        # imported bone display lengths. Consecutive co-located motors are collapsed
        # so multi-axis joints at the same pivot do not distort the chain length.
        urdf_chain_length_key = f"ik_urdf_chain_len_{chain_key}"
        if urdf_chain_length_key not in target_cache:
            if ref_data is not None and "points_local" in ref_data:
                urdf_chain_points = [
                    mathutils.Vector(v) for v in ref_data["points_local"]
                ]
            else:
                urdf_chain_points = [
                    pb.matrix.to_translation().copy() for pb in solver_bones
                ]
                urdf_chain_points.append(end_bone.matrix.to_translation().copy())
            target_cache[urdf_chain_length_key] = _compute_joint_path_length(
                urdf_chain_points
            )
        urdf_chain_length = float(target_cache[urdf_chain_length_key])

        # BVH chain length: sum REST-pose bone lengths walking up the ancestor chain.
        # Using rest-pose (bone.bone.length) avoids pose-dependent distance inflation
        # that would corrupt proportion_scale when cached on a dynamic animation frame.
        bvh_chain_length_key = f"ik_bvh_chain_len_{chain_key}"
        if bvh_chain_length_key not in target_cache:
            bvh_segment_lengths = []
            current_bvh = bvh_target_bone
            wanted_bvh_root = str(getattr(chain, "bvh_root_bone_name", "")).strip()
            reached_wanted_root = False
            ancestor_depth = 0
            max_ancestor_depth = max(2, len(solver_bones) + 2)
            while ancestor_depth < max_ancestor_depth:
                # Use rest-pose bone length (pose-independent).
                rest_len = getattr(current_bvh.bone, "length", 0.0)
                bvh_segment_lengths.insert(0, max(rest_len, 1e-9))

                if wanted_bvh_root and current_bvh.name == wanted_bvh_root:
                    reached_wanted_root = True
                    break

                if not current_bvh.parent:
                    break

                current_bvh = current_bvh.parent
                ancestor_depth += 1

            # If an explicit BVH root was configured but not reached, force fallback.
            if wanted_bvh_root and not reached_wanted_root:
                bvh_chain_length = 0.0
            else:
                bvh_chain_length = sum(bvh_segment_lengths)
            target_cache[bvh_chain_length_key] = bvh_chain_length
        else:
            bvh_chain_length = target_cache[bvh_chain_length_key]

        # Proportion scale: URDF total chain length / BVH total chain length
        # Provides better skeletal proportion compensation across different rigs
        if bvh_chain_length > 1e-6 and urdf_chain_length > 1e-6:
            proportion_scale = urdf_chain_length / bvh_chain_length
        else:
            # Fallback to end-effector distance ratioing
            bvh_ref_radius = max(ref_bvh_local.length, 1e-6)
            urdf_ref_radius = ref_urdf_local.length
            proportion_scale = urdf_ref_radius / bvh_ref_radius

        prop_scale_min = min(prop_scale_min, proportion_scale)
        prop_scale_max = max(prop_scale_max, proportion_scale)

        # Convert BVH root-space motion delta into URDF armature-local space.
        # target_local/ref_bvh_local live in BVH root-bone space, while ref_urdf_local
        # lives in URDF armature space. Mixing them directly breaks motion direction
        # whenever BVH root orientation differs from the URDF object orientation.
        bvh_motion_delta = target_local - ref_bvh_local
        bvh_motion_world = bvh_root_mat.to_3x3() @ bvh_motion_delta
        urdf_motion_delta = urdf_obj.matrix_world.to_3x3().inverted() @ bvh_motion_world
        motion_delta_z_max = max(motion_delta_z_max, abs(urdf_motion_delta.z))

        # Absolute target keeps the neutral/reference frame aligned.
        absolute_target = ref_urdf_local + urdf_motion_delta

        # Proportion-scaled motion target transfers BVH motion with chain-size compensation.
        scaled_motion_target = ref_urdf_local + (
            urdf_motion_delta * proportion_scale * settings.ik_target_scale
        )

        # Blend both strategies for robust behavior across very different body proportions.
        desired_local = absolute_target.lerp(
            scaled_motion_target,
            settings.ik_proportion_blend,
        )

        # Keep planted feet close to custom/default URDF reference height,
        # but only when the foot is truly stationary (low velocity).
        # During swing phases the foot moves quickly even though it may be
        # close to the ground; locking Z at those moments suppresses the
        # backward swing and causes the asymmetric one-sided motion.
        if chain.bvh_target_bone_name in {settings.foot_l_name, settings.foot_r_name}:
            bvh_floor_offset = scene.get("bvh_floor_offset", 0.0)
            foot_height = bvh_target_world.z - bvh_floor_offset
            if foot_height <= settings.jump_threshold:
                # Use per-frame velocity snapshot from stance detection.
                foot_velocities = scene.get("_foot_velocities", {})
                foot_vel = float(foot_velocities.get(chain.bvh_target_bone_name, 0.0))

                # Scale ground-lock strength down when the foot is moving.
                # A velocity > 0.02 m/frame (~1.2 m/s at 60 fps) is considered
                # swing phase — full velocity release.
                vel_scale = max(
                    0.0, 1.0 - foot_vel / max(1e-6, settings.jump_threshold * 2.0)
                )
                effective_lock = settings.ik_ground_lock_strength * vel_scale
                if effective_lock > 1e-4:
                    desired_local.z = (
                        1.0 - effective_lock
                    ) * desired_local.z + effective_lock * ref_urdf_local.z

        target_world = urdf_obj.matrix_world @ desired_local

        # Clamp target distance to reachable chain length to avoid solver instability.
        # Use neutral-pose segment lengths (true motor-to-motor distances) as the
        # maximum extension, NOT bone display lengths which are nominal motor sizes.
        chain_root_world = _get_pose_bone_world_position(urdf_obj, full_chain[0])
        chain_reach_key = f"ik_reach_{chain_key}"
        if chain_reach_key in target_cache:
            chain_reach = float(target_cache[chain_reach_key])
        else:
            if ref_data is not None and "segment_lengths" in ref_data:
                chain_reach = sum(max(0.0, float(s)) for s in ref_data["segment_lengths"])
            else:
                chain_reach = 0.0
                for i in range(1, len(full_chain)):
                    a = _get_pose_bone_world_position(urdf_obj, full_chain[i - 1])
                    b = _get_pose_bone_world_position(urdf_obj, full_chain[i])
                    chain_reach += (b - a).length
            if chain_reach <= 1e-9:
                chain_reach = 0.0
            target_cache[chain_reach_key] = chain_reach

        to_target = target_world - chain_root_world
        if chain_reach > 1e-6 and to_target.length > chain_reach:
            target_world = chain_root_world + to_target.normalized() * chain_reach

        cache_key = f"ik_t_{chain_key}"
        target_world = _smooth_target_position(
            target_cache,
            cache_key,
            target_world,
            settings.ik_target_smoothing,
            initial_value=end_world_now,
        )

        # Deep foot-pinning integration: keep the active stance-foot target
        # fixed inside the IK solver instead of relying purely on post-correction.
        if (
            solver_pin_enabled
            and chain.bvh_target_bone_name == solver_pin_anchor
            and chain.bvh_target_bone_name
            in {settings.foot_l_name, settings.foot_r_name}
        ):
            target_world = mathutils.Vector(solver_pin_pos)

        pre_error = (target_world - end_world_now).length

        is_foot_chain = chain.bvh_target_bone_name in {
            settings.foot_l_name,
            settings.foot_r_name,
        }
        desired_end_world_q = None
        orient_weight = 0.0

        # Apply orientation targeting to ALL chains (not just feet).
        # Use per-chain orientation_weight if available, fall back to global setting.
        bvh_target_world_q = (
            bvh_obj.matrix_world @ bvh_target_bone.matrix
        ).to_quaternion()
        ref_orient_key = f"ik_r_q_{chain_key}"
        if ref_orient_key not in target_cache:
            # Reference orientation is the delta from neutral URDF pose to BVH
            end_world_q = (urdf_obj.matrix_world @ end_bone.matrix).to_quaternion()
            ref_orient_q = end_world_q @ bvh_target_world_q.inverted()
            target_cache[ref_orient_key] = (
                ref_orient_q.w,
                ref_orient_q.x,
                ref_orient_q.y,
                ref_orient_q.z,
            )

        ref_orient_q = mathutils.Quaternion(target_cache[ref_orient_key])
        desired_end_world_q = ref_orient_q @ bvh_target_world_q

        # Get orientation weight: per-chain override > global foot weight > default
        per_chain_weight = getattr(chain, "orientation_weight", None)
        if per_chain_weight is not None:
            orient_weight = clamp_to_limits(per_chain_weight, 0.0, 1.0)
        elif is_foot_chain:
            # Legacy support for global foot-specific weight
            orient_weight = clamp_to_limits(
                getattr(settings, "ik_foot_orientation_weight", 0.55),
                0.0,
                1.0,
            )
        else:
            # Non-foot chains: use default per-chain weight
            orient_weight = clamp_to_limits(0.55, 0.0, 1.0)

        # Hybrid mode can adapt IK strength by current target error:
        # keep FK-dominant behavior for tiny residuals and progressively
        # increase IK influence when end-effector error grows.
        chain_blend = correction_blend
        if retarget_mode == "HYBRID" and getattr(settings, "hybrid_adaptive_ik", False):
            use_chain_override = getattr(chain, "use_hybrid_adaptive_override", False)
            if use_chain_override:
                low = max(1e-6, chain.hybrid_error_low)
                high = max(low + 1e-6, chain.hybrid_error_high)
                min_blend = clamp_to_limits(chain.hybrid_min_ik_blend, 0.0, 1.0)
            else:
                low = max(1e-6, settings.hybrid_error_low)
                high = max(low + 1e-6, settings.hybrid_error_high)
                min_blend = clamp_to_limits(settings.hybrid_min_ik_blend, 0.0, 1.0)

            t = (pre_error - low) / (high - low)
            t = clamp_to_limits(t, 0.0, 1.0)
            adaptive_gain = min_blend + (1.0 - min_blend) * t
            chain_blend *= adaptive_gain

        # FABRIK-C path uses direct adaptive blend without extra temporal EMA.
        chain_blend = clamp_to_limits(chain_blend, 0.0, 1.0)

        for pose_bone in solver_bones:
            pose_bone.rotation_mode = "QUATERNION"

        max_iterations = (
            int(max_iterations_override)
            if max_iterations_override is not None
            else int(settings.ik_iterations)
        )
        if retarget_mode == "HYBRID":
            blend_factor = clamp_to_limits(chain_blend, 0.15, 1.0)
            max_iterations = int(round(max_iterations * blend_factor))
        max_iterations = max(1, max_iterations)

        if ref_data is None:
            # Fallback: keep current state if neutral reference cannot be built.
            initial_points_world = _collect_chain_points_world(
                urdf_obj,
                solver_bones,
                end_bone,
            )
            segment_lengths = [
                max(
                    1e-9, (initial_points_world[i + 1] - initial_points_world[i]).length
                )
                for i in range(len(initial_points_world) - 1)
            ]
            hinge_axes_world = [
                _get_pose_bone_world_axis(urdf_obj, pb) for pb in solver_bones
            ]
            base_dirs_world = []
            for i in range(len(initial_points_world) - 1):
                seg = initial_points_world[i + 1] - initial_points_world[i]
                if seg.length_squared < 1e-12:
                    base_dirs_world.append(mathutils.Vector((1, 0, 0)))
                else:
                    base_dirs_world.append(seg.normalized())
        else:
            points_local = [mathutils.Vector(v) for v in ref_data["points_local"]]
            segment_lengths = [max(1e-9, float(v)) for v in ref_data["segment_lengths"]]
            hinge_axes_local = [
                mathutils.Vector(v) for v in ref_data["hinge_axes_local"]
            ]
            base_dirs_local = [mathutils.Vector(v) for v in ref_data["base_dirs_local"]]

            initial_points_world = [
                urdf_obj.matrix_world @ p_local for p_local in points_local
            ]

            root_world_now = _get_pose_bone_world_position(urdf_obj, solver_bones[0])
            root_shift = root_world_now - initial_points_world[0]
            initial_points_world = [p + root_shift for p in initial_points_world]

            world_rot = urdf_obj.matrix_world.to_3x3()
            hinge_axes_world = []
            for axis_local in hinge_axes_local:
                axis_world = world_rot @ axis_local
                if axis_world.length_squared < 1e-12:
                    axis_world = mathutils.Vector((0, 1, 0))
                else:
                    axis_world.normalize()
                hinge_axes_world.append(axis_world)

            base_dirs_world = []
            for dir_local in base_dirs_local:
                dir_world = world_rot @ dir_local
                if dir_world.length_squared < 1e-12:
                    dir_world = mathutils.Vector((1, 0, 0))
                else:
                    dir_world.normalize()
                base_dirs_world.append(dir_world)

        fabrik_points, iters_used, final_residual = _solve_fabrik_c_positions(
            initial_points_world,
            target_world,
            segment_lengths,
            max_iterations=max_iterations,
            tolerance=settings.ik_tolerance,
            hinge_axes_world=hinge_axes_world,
            hinge_base_dirs_world=base_dirs_world,
            target_orientation=desired_end_world_q,
            orientation_weight=orient_weight,
        )

        total_iters += float(iters_used)
        total_residual += float(final_residual)
        solved_chains += 1

        current_points = _collect_chain_points_world(urdf_obj, solver_bones, end_bone)
        for i, pose_bone in enumerate(solver_bones):
            to_cur = current_points[i + 1] - current_points[i]
            to_des = fabrik_points[i + 1] - fabrik_points[i]

            if to_cur.length_squared < 1e-12 or to_des.length_squared < 1e-12:
                continue

            axis_world = _get_pose_bone_world_axis(urdf_obj, pose_bone)
            cur_proj = to_cur - axis_world * to_cur.dot(axis_world)
            des_proj = to_des - axis_world * to_des.dot(axis_world)
            if cur_proj.length_squared < 1e-12 or des_proj.length_squared < 1e-12:
                continue

            angle_delta = _signed_angle_around_axis(
                cur_proj.normalized(),
                des_proj.normalized(),
                axis_world,
            )

            angle_delta *= chain.influence
            angle_delta *= chain_blend
            angle_delta = clamp_to_limits(
                angle_delta,
                -settings.ik_max_step_angle,
                settings.ik_max_step_angle,
            )

            if abs(angle_delta) < 1e-7:
                continue

            joint_type = get_bone_property(pose_bone, "joint_type", "revolute")
            current_angle = get_bone_property(pose_bone, "_joint_angle", 0.0)
            _apply_final_angle(pose_bone, current_angle + angle_delta, joint_type)

        bpy.context.view_layer.update()

    if solved_chains > 0:
        scene["_ik_fabrik_avg_iters"] = total_iters / float(solved_chains)
        scene["_ik_fabrik_avg_residual"] = total_residual / float(solved_chains)
        scene["_ik_fabrik_chains"] = solved_chains
        scene["_ik_proportion_scale_min"] = prop_scale_min
        scene["_ik_proportion_scale_max"] = prop_scale_max
        scene["_ik_motion_delta_z_max"] = motion_delta_z_max
    else:
        scene["_ik_fabrik_avg_iters"] = 0.0
        scene["_ik_fabrik_avg_residual"] = 0.0
        scene["_ik_fabrik_chains"] = 0
        scene["_ik_proportion_scale_min"] = 0.0
        scene["_ik_proportion_scale_max"] = 0.0
        scene["_ik_motion_delta_z_max"] = 0.0


def get_lowest_z_world(urdf_obj: bpy.types.Object) -> float:
    """
    Find the lowest Z-coordinate (height) of all visual meshes in world space.

    Returns:
        The minimum Z-coordinate found. If no meshes exist, uses bone positions.
    """
    global_min_z = float("inf")
    found_mesh = False

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
        (anchor_bone_name, is_switch, both_grounded)
        anchor_bone_name: active stance foot bone name or "" if airborne.
        is_switch: True if anchor switched from previous frame.
        both_grounded: True when both feet are currently grounded.
    """
    bvh_floor_offset = scene.get("bvh_floor_offset", 0.0)

    if "_foot_positions" not in scene:
        scene["_foot_positions"] = {}

    last_positions = scene["_foot_positions"]
    scene["_foot_positions_prev"] = {
        name: tuple(pos) for name, pos in last_positions.items()
    }
    if "_foot_contact_state" not in scene:
        scene["_foot_contact_state"] = {}
    contact_state = scene["_foot_contact_state"]

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

        # Contact hysteresis avoids rapid grounded/airborne toggling
        # when foot height hovers around the threshold.
        was_grounded = bool(contact_state.get(f_name, False))
        if was_grounded:
            grounded = (
                foot_height
                <= settings.jump_threshold + settings.foot_contact_hysteresis
            )
        else:
            grounded = foot_height <= settings.jump_threshold

        contact_state[f_name] = grounded
        if grounded:
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

    # Snapshot for consumers in the same frame (IK ground-lock, debug UI).
    scene["_foot_velocities"] = {name: float(v) for name, v in foot_velocities.items()}

    return anchor_bone_name, is_switch, both_grounded


def _apply_output_post_processing(
    urdf_obj,
    settings,
    smooth_cache: dict,
    scene,
) -> None:
    """
    Apply continuity correction, velocity limiting and EMA smoothing to all
    URDF joint outputs after FK, IK and foot alignment have all written to
    ``_joint_angle``.  Uniform stabilisation for all motion sources.
    """
    target_hz = scene.render.fps if scene.render.fps > 0 else 60.0
    dt = 1.0 / target_hz

    for pose_bone in urdf_obj.pose.bones:
        jtype = get_bone_property(pose_bone, "joint_type", "revolute")
        if jtype not in {"revolute", "continuous"}:
            continue

        raw_angle = get_bone_property(pose_bone, "_joint_angle", 0.0)

        # Continuity correction - prevents +/-2pi jumps
        cont_key = "pp_c_" + pose_bone.name
        if cont_key in smooth_cache:
            raw_angle = apply_continuity_correction(
                raw_angle, smooth_cache[cont_key], settings.max_jump_threshold
            )
        smooth_cache[cont_key] = raw_angle

        # Velocity limiting
        vel_key = "pp_v_" + pose_bone.name
        prev_raw = smooth_cache.get(vel_key, raw_angle)
        velocity_limit = get_bone_property(pose_bone, "velocity_limit", 10.0)
        raw_angle, was_limited = apply_velocity_limiting(
            raw_angle, prev_raw, velocity_limit, dt
        )
        set_bone_property(pose_bone, "is_velocity_limited", was_limited)
        smooth_cache[vel_key] = raw_angle

        # EMA smoothing
        # Get previous angle; if missing (IK-only initialization), use current as baseline
        last_angle = get_bone_property(pose_bone, "_last_urdf_angle", None)
        if last_angle is None:
            # First frame or IK-only mode: no prior state, use current as baseline
            last_angle = raw_angle
        final_angle = apply_exponential_smoothing(
            raw_angle, last_angle, settings.joint_smoothing
        )
        set_bone_property(pose_bone, "_last_urdf_angle", final_angle)

        _apply_final_angle(pose_bone, final_angle, jtype)


def apply_joint_retargeting(
    urdf_obj,
    bvh_obj,
    mapping_item,
    smooth_cache: dict,
    settings,
    scene,
) -> None:
    """
    Retarget a single joint mapping from BVH to URDF.

    Extracts the per-axis rotation component from the BVH bone and writes the
    raw target angle via _apply_final_angle.  Continuity correction, velocity
    limiting and EMA smoothing are applied afterward by
    _apply_output_post_processing so all motion sources benefit equally.
    """
    bvh_b = bvh_obj.pose.bones.get(mapping_item.bvh_bone_name)
    if not bvh_b:
        return

    default_joint_angles = _get_custom_default_joint_angles(settings)

    ref_q = mathutils.Quaternion(mapping_item.ref_rot)
    current_q = bvh_b.matrix_basis.to_quaternion()
    delta_q = ref_q.inverted() @ current_q

    for urdf_bone_mapping in mapping_item.urdf_bones:
        urdf_b = urdf_obj.pose.bones.get(urdf_bone_mapping.urdf_bone_name)
        if not urdf_b:
            continue

        jtype = get_bone_property(urdf_b, "joint_type", "revolute")
        if jtype not in {"revolute", "continuous"}:
            continue

        twist_axis = _AXIS_VECTORS.get(
            urdf_bone_mapping.source_axis, _AXIS_VECTORS["Z"]
        )
        val = extract_twist_angle(delta_q, twist_axis)

        if urdf_bone_mapping.sign == "NEG":
            val = -val

        if "offset" not in urdf_b:
            set_bone_property(urdf_b, "offset", val)

        default_pose_offset = default_joint_angles.get(urdf_b.name, 0.0)
        target_angle = (
            default_pose_offset
            + val
            - get_bone_property(urdf_b, "offset", 0.0)
            + urdf_bone_mapping.neutral_offset
        )

        # Write raw target angle; _apply_output_post_processing handles the rest
        _apply_final_angle(urdf_b, target_angle, jtype)


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

    # Build BVH-to-URDF foot bone mapping. A foot joint may appear both as a
    # direct FK mapping and as an IK end-effector; apply alignment only once.
    bvh_to_urdf_map = {}
    for item in settings.mappings:
        if item.bvh_bone_name in [settings.foot_l_name, settings.foot_r_name]:
            foot_targets = bvh_to_urdf_map.setdefault(item.bvh_bone_name, [])
            seen_targets = {name for name, _ in foot_targets}
            for b in item.urdf_bones:
                if not b.urdf_bone_name or b.urdf_bone_name in seen_targets:
                    continue
                foot_targets.append((b.urdf_bone_name, b.invert_alignment))
                seen_targets.add(b.urdf_bone_name)

    for chain in settings.kinematic_chains:
        if (
            chain.bvh_target_bone_name in [settings.foot_l_name, settings.foot_r_name]
            and chain.urdf_end_bone_name
        ):
            foot_targets = bvh_to_urdf_map.setdefault(chain.bvh_target_bone_name, [])
            if chain.urdf_end_bone_name not in {name for name, _ in foot_targets}:
                foot_targets.append((chain.urdf_end_bone_name, False))

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
        3. Stance detection + IK pin setup
        4. Joint retargeting (FK + IK)
        5. Foot alignment
        5b. Output post-processing (continuity, velocity limiting, EMA)
        6. Scene update
        7. Sticky-foot correction
        8. Anti-sinking guard

    Args:
        scene: The current Blender scene.
    """
    settings = scene.bvh_mapping_settings
    if not settings.live_retarget:
        return

    t_start = time.perf_counter()

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

    debug_metrics_enabled = bool(getattr(settings, "stability_debug_metrics", False))
    dbg_anchor_switches = 0
    dbg_yaw_residual_abs = 0.0
    dbg_yaw_step_abs = 0.0
    dbg_xy_drift_abs = 0.0
    dbg_xy_step_abs = 0.0

    # ------------------------------------------------------------------
    # Reset state at the first frame to avoid drift from previous plays
    # ------------------------------------------------------------------
    current_frame = scene.frame_current
    if current_frame == 0 or current_frame == scene.frame_start:
        scene["_persistent_foot_correction"] = mathutils.Vector((0, 0, 0))
        scene["_persistent_foot_rot_correction"] = mathutils.Quaternion((1, 0, 0, 0))
        scene["_active_anchor_name"] = ""
        scene["_anchor_world_pos"] = None
        scene["_anchor_world_pos_xy"] = None
        scene["_anchor_world_yaw"] = None
        scene["_anchor_bvh_yaw"] = None
        scene["_ik_solver_pin_anchor"] = ""
        scene["_ik_solver_pin_pos"] = None
        scene["_foot_positions"] = {}
        scene["_foot_positions_prev"] = {}
        scene["_foot_contact_state"] = {}

    # ------------------------------------------------------------------
    # 1. BVH root position (scaled delta from reference)
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

    persistent_correction = mathutils.Vector(
        scene.get("_persistent_foot_correction", (0, 0, 0))
    )

    offset_xy = settings.location_offset
    loc_offset = mathutils.Vector(
        (offset_xy[0], offset_xy[1], scene.get("urdf_height_offset", 0.0))
    )

    urdf.location = bvh_delta + persistent_correction + loc_offset

    # ------------------------------------------------------------------
    # 2. BVH root rotation (axis-corrected, pitch/roll damped)
    # ------------------------------------------------------------------
    ref_rot = mathutils.Quaternion(scene.get("ref_root_rot", (1, 0, 0, 0)))
    off_q = mathutils.Euler(settings.rotation_offset).to_quaternion()
    default_root_q = _get_custom_default_root_quaternion(settings)
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
    urdf.rotation_quaternion = (
        persistent_rot_correction @ off_q @ default_root_q @ delta_rot
    )

    # ------------------------------------------------------------------
    # 3. Stance detection + solver pin setup (before IK)
    # ------------------------------------------------------------------
    anchor_bone_name, is_anchor_switch, both_grounded = detect_stance_foot(
        scene, bvh, settings
    )
    if is_anchor_switch:
        dbg_anchor_switches = 1

    last_anchor = scene.get("_active_anchor_name", "")
    is_first_anchor = anchor_bone_name != "" and last_anchor == ""

    solver_pin_anchor = ""
    solver_pin_pos = None
    if anchor_bone_name and not is_anchor_switch and not is_first_anchor:
        anchor_world_pos = scene.get("_anchor_world_pos", None)
        if anchor_world_pos is not None:
            solver_pin_anchor = anchor_bone_name
            solver_pin_pos = (
                anchor_world_pos[0],
                anchor_world_pos[1],
                anchor_world_pos[2],
            )

    scene["_ik_solver_pin_anchor"] = solver_pin_anchor
    scene["_ik_solver_pin_pos"] = solver_pin_pos

    # ------------------------------------------------------------------
    # 3b. Flush urdf.matrix_world so IK world-space computations are correct.
    # Root location + rotation were set in steps 1-2.  Without this update,
    # urdf_obj.matrix_world is stale (previous frame) for all IK chain
    # position queries, causing systematic target offset — especially severe
    # in IK-only mode where there is no FK warm-start to absorb the error.
    # ------------------------------------------------------------------
    bpy.context.view_layer.update()

    retarget_mode = _get_retargeting_mode(settings)
    ik_only_active = bool(settings.ik_only_mode or retarget_mode == "IK_ONLY")
    fk_only_active = retarget_mode == "FK_ONLY"
    previous_retarget_mode = str(scene.get("_last_retarget_mode", ""))
    if previous_retarget_mode != retarget_mode:
        scene["_joint_angles_initialized"] = False
    scene["_last_retarget_mode"] = retarget_mode

    # ------------------------------------------------------------------
    # 3c. Initialize joint angles for IK-only mode and continuity
    # ------------------------------------------------------------------
    _ensure_joint_angles_initialized(urdf, scene)

    # ------------------------------------------------------------------
    # 4. Joint retargeting (FK baseline + IK correction)
    # ------------------------------------------------------------------

    t_fk_start = time.perf_counter()
    if not ik_only_active:
        for item in settings.mappings:
            apply_joint_retargeting(urdf, bvh, item, smooth_cache, settings, scene)
    scene["_retarget_ms_fk"] = (time.perf_counter() - t_fk_start) * 1000.0

    export_full_quality = bool(scene.get("_export_full_quality", False))

    ik_blend = clamp_to_limits(settings.hybrid_ik_blend, 0.0, 1.0)
    if ik_only_active:
        ik_blend = 1.0
    elif fk_only_active:
        ik_blend = 0.0

    if ik_blend <= 1e-6:
        scene["_retarget_ms_ik"] = 0.0
    else:
        hybrid_iterations_override = None
        if (
            settings.hybrid_realtime_guard
            and not export_full_quality
            and not ik_only_active
            and not fk_only_active
        ):
            avg_ms = scene.get("_retarget_ms_ema", None)
            fps = scene.render.fps if scene.render.fps > 0 else 60.0
            budget_ms = 1000.0 / fps
            if avg_ms is not None and avg_ms > budget_ms:
                overload_ratio = min(1.0, (avg_ms - budget_ms) / max(1e-6, budget_ms))
                throttle = 1.0 - 0.7 * overload_ratio
                target_iter = int(round(settings.ik_iterations * throttle))
                hybrid_iterations_override = max(
                    int(settings.hybrid_min_iterations),
                    max(1, target_iter),
                )

        t_ik_start = time.perf_counter()
        apply_kinematic_retargeting(
            urdf,
            bvh,
            settings,
            scene,
            correction_blend=ik_blend,
            max_iterations_override=hybrid_iterations_override,
        )
        scene["_retarget_ms_ik"] = (time.perf_counter() - t_ik_start) * 1000.0

    # ------------------------------------------------------------------
    # 5. Foot alignment (flatten near ground)
    # ------------------------------------------------------------------
    apply_foot_alignment(urdf, bvh, settings, smooth_cache, scene)

    # ------------------------------------------------------------------
    # 5b. Shared output post-processing (continuity, velocity, EMA)
    # ------------------------------------------------------------------
    _apply_output_post_processing(urdf, settings, smooth_cache, scene)

    # ------------------------------------------------------------------
    # 6. Scene update so matrix_world is current for measurement
    # ------------------------------------------------------------------
    bpy.context.view_layer.update()

    # ------------------------------------------------------------------
    # 7. Sticky-foot correction (yaw rotation + XY translation)
    # ------------------------------------------------------------------
    if anchor_bone_name:
        urdf_foot_bone = _find_urdf_foot_for_anchor(anchor_bone_name, settings, urdf)

        if urdf_foot_bone:
            foot_world_mat = urdf.matrix_world @ urdf_foot_bone.matrix
            foot_world = foot_world_mat.to_translation()
            foot_yaw = _extract_yaw(foot_world_mat)

            if is_anchor_switch or is_first_anchor:
                scene["_anchor_world_pos"] = (foot_world.x, foot_world.y, foot_world.z)
                scene["_anchor_world_pos_xy"] = (foot_world.x, foot_world.y)
                scene["_anchor_world_yaw"] = foot_yaw

                bvh_anchor_bone = bvh.pose.bones.get(anchor_bone_name)
                if bvh_anchor_bone:
                    bvh_foot_mat = bvh.matrix_world @ bvh_anchor_bone.matrix
                    scene["_anchor_bvh_yaw"] = _extract_yaw(bvh_foot_mat)
                else:
                    scene["_anchor_bvh_yaw"] = None

            anchor_yaw = scene.get("_anchor_world_yaw", None)
            if anchor_yaw is not None:
                urdf_yaw_change = _wrap_angle(foot_yaw - anchor_yaw)

                bvh_yaw_change = 0.0
                anchor_bvh_yaw = scene.get("_anchor_bvh_yaw", None)
                if anchor_bvh_yaw is not None:
                    bvh_anchor_bone = bvh.pose.bones.get(anchor_bone_name)
                    if bvh_anchor_bone:
                        cur_bvh_yaw = _extract_yaw(
                            bvh.matrix_world @ bvh_anchor_bone.matrix
                        )
                        bvh_yaw_change = _wrap_angle(cur_bvh_yaw - anchor_bvh_yaw)

                yaw_residual = _wrap_angle(urdf_yaw_change - bvh_yaw_change)
                dbg_yaw_residual_abs = max(dbg_yaw_residual_abs, abs(yaw_residual))

                if abs(yaw_residual) > 1e-6:
                    base_yaw = max(1e-5, settings.foot_pin_yaw_max_step)
                    if bool(getattr(settings, "adaptive_foot_pinning", True)):
                        max_yaw = _compute_adaptive_step_limit(
                            base_yaw,
                            abs(yaw_residual),
                            getattr(settings, "foot_pin_adaptive_gain", 1.5),
                        )
                    else:
                        max_yaw = base_yaw

                    yaw_step = clamp_to_limits(-yaw_residual, -max_yaw, max_yaw)
                    dbg_yaw_step_abs = max(dbg_yaw_step_abs, abs(yaw_step))
                    yaw_fix_q = mathutils.Quaternion((0, 0, 1), yaw_step)

                    urdf.rotation_quaternion = yaw_fix_q @ urdf.rotation_quaternion

                    if not both_grounded:
                        root_pos = urdf.location.copy()
                        pivot = mathutils.Vector(
                            (foot_world.x, foot_world.y, root_pos.z)
                        )
                        offset_from_pivot = root_pos - pivot
                        rotated_offset = yaw_fix_q @ offset_from_pivot
                        urdf.location = pivot + rotated_offset

                        loc_shift = rotated_offset - offset_from_pivot
                        persistent_correction += loc_shift
                        scene["_persistent_foot_correction"] = persistent_correction

                    persistent_rot_correction = yaw_fix_q @ persistent_rot_correction
                    scene["_persistent_foot_rot_correction"] = persistent_rot_correction

                    bpy.context.view_layer.update()
                    foot_world = (
                        urdf.matrix_world @ urdf_foot_bone.matrix
                    ).to_translation()

            anchor_xy = scene.get("_anchor_world_pos_xy", None)
            if anchor_xy is not None:
                anchor_x, anchor_y = anchor_xy

                drift_x = foot_world.x - anchor_x
                drift_y = foot_world.y - anchor_y

                drift_len = math.hypot(drift_x, drift_y)
                dbg_xy_drift_abs = max(dbg_xy_drift_abs, drift_len)

                base_xy = max(1e-6, settings.foot_pin_xy_max_step)
                if bool(getattr(settings, "adaptive_foot_pinning", True)):
                    max_xy = _compute_adaptive_step_limit(
                        base_xy,
                        drift_len,
                        getattr(settings, "foot_pin_adaptive_gain", 1.5),
                    )
                else:
                    max_xy = base_xy

                if drift_len > max_xy:
                    scale = max_xy / drift_len
                    drift_x *= scale
                    drift_y *= scale

                dbg_xy_step_abs = max(dbg_xy_step_abs, math.hypot(drift_x, drift_y))

                urdf.location.x -= drift_x
                urdf.location.y -= drift_y

                persistent_correction.x -= drift_x
                persistent_correction.y -= drift_y
                scene["_persistent_foot_correction"] = persistent_correction

        scene["_active_anchor_name"] = anchor_bone_name

    else:
        scene["_active_anchor_name"] = ""
        scene["_anchor_world_pos"] = None
        scene["_anchor_world_pos_xy"] = None
        scene["_anchor_world_yaw"] = None
        scene["_anchor_bvh_yaw"] = None
        scene["_ik_solver_pin_anchor"] = ""
        scene["_ik_solver_pin_pos"] = None

    # ------------------------------------------------------------------
    # 7c. Correction decay - pull persistent offsets toward BVH
    # ------------------------------------------------------------------
    target_hz = scene.render.fps if scene.render.fps > 0 else 120.0
    ref_fps = 120.0
    raw_decay = settings.correction_decay / 10.0
    if raw_decay >= 0.1:
        decay_rate = 1.0
    elif raw_decay > 0.0:
        decay_rate = 1.0 - (1.0 - raw_decay) ** (ref_fps / target_hz)
    else:
        decay_rate = 0.0

    decay_allowed = True
    if settings.correction_decay_airborne_only and anchor_bone_name:
        decay_allowed = False

    if decay_rate > 0.0 and decay_allowed:
        persistent_correction = mathutils.Vector(
            scene.get("_persistent_foot_correction", (0, 0, 0))
        )
        persistent_rot_correction = mathutils.Quaternion(
            scene.get("_persistent_foot_rot_correction", (1, 0, 0, 0))
        )

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

        anchor_pos = scene.get("_anchor_world_pos", None)
        if anchor_pos is not None:
            scene["_anchor_world_pos"] = (
                anchor_pos[0] - decay_x,
                anchor_pos[1] - decay_y,
                anchor_pos[2],
            )

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

        scene["_persistent_foot_correction"] = persistent_correction
        scene["_persistent_foot_rot_correction"] = persistent_rot_correction

    # ------------------------------------------------------------------
    # 8. Anti-sinking guard
    # ------------------------------------------------------------------
    t_sink_start = time.perf_counter()
    bpy.context.view_layer.update()
    current_min_z = get_lowest_z_world(urdf)

    if current_min_z < -1e-4:
        urdf.location.z += abs(current_min_z)
    scene["_retarget_ms_sink"] = (time.perf_counter() - t_sink_start) * 1000.0

    elapsed_ms = (time.perf_counter() - t_start) * 1000.0
    prev_ms = scene.get("_retarget_ms_ema", None)
    if prev_ms is None:
        scene["_retarget_ms_ema"] = elapsed_ms
    else:
        scene["_retarget_ms_ema"] = (0.9 * float(prev_ms)) + (0.1 * elapsed_ms)

    if debug_metrics_enabled:
        scene["_stability_dbg_anchor_switch"] = int(dbg_anchor_switches)
        scene["_stability_dbg_yaw_residual"] = float(dbg_yaw_residual_abs)
        scene["_stability_dbg_yaw_step"] = float(dbg_yaw_step_abs)
        scene["_stability_dbg_xy_drift"] = float(dbg_xy_drift_abs)
        scene["_stability_dbg_xy_step"] = float(dbg_xy_step_abs)

        # IK-specific metrics
        avg_iters = scene.get("_ik_fabrik_avg_iters", 0.0)
        avg_residual = scene.get("_ik_fabrik_avg_residual", 0.0)
        chains_solved = scene.get("_ik_fabrik_chains", 0)

        scene["_ik_debug_iterations"] = float(avg_iters)
        scene["_ik_debug_residual"] = float(avg_residual)
        scene["_ik_debug_chains_count"] = int(chains_solved)
    else:
        for key in (
            "_stability_dbg_anchor_switch",
            "_stability_dbg_yaw_residual",
            "_stability_dbg_yaw_step",
            "_stability_dbg_xy_drift",
            "_stability_dbg_xy_step",
        ):
            if key in scene:
                del scene[key]
