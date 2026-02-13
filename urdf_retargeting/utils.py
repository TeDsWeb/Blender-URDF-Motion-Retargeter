"""
Utility functions for URDF retargeting.

Provides common operations such as parsing, mathematical conversions,
and shared calculations used across the retargeting pipeline.
"""

import math
import mathutils


def parse_float_list(s: str, n: int) -> list[float]:
    """
    Parse a string of space/comma-separated floats into a list of fixed length.

    Args:
        s: String containing space or comma-separated float values.
        n: Expected length of the output list.

    Returns:
        A list of n floats, padded with zeros if necessary.
    """
    if s is None:
        return [0.0] * n

    parts = s.replace(",", " ").split()
    vals = []

    for p in parts:
        try:
            vals.append(float(p))
        except ValueError:
            vals.append(0.0)

    # Pad with zeros if needed
    while len(vals) < n:
        vals.append(0.0)

    return vals[:n]


def origin_to_matrix(xyz: list[float], rpy: list[float]) -> mathutils.Matrix:
    """
    Convert URDF origin (xyz, rpy) to a 4x4 transformation matrix.

    Args:
        xyz: Translation vector [x, y, z].
        rpy: Rotation in Euler angles (roll, pitch, yaw) in radians.

    Returns:
        A 4x4 transformation matrix combining translation and rotation.
    """
    loc = mathutils.Vector(xyz)
    rot = mathutils.Euler(rpy, "XYZ").to_matrix().to_4x4()
    return mathutils.Matrix.Translation(loc) @ rot


def extract_twist_angle(
    quaternion: mathutils.Quaternion, axis: mathutils.Vector
) -> float:
    """
    Extract the rotation angle around a specified axis from a quaternion.

    Uses atan2 for improved numerical stability compared to standard methods.

    Args:
        quaternion: The quaternion to extract rotation from.
        axis: The axis to extract rotation around (local space).

    Returns:
        The angle in radians around the specified axis.
    """
    q_vec = mathutils.Vector((quaternion.x, quaternion.y, quaternion.z))
    projection = q_vec.dot(axis)
    return 2.0 * math.atan2(projection, quaternion.w)


def apply_continuity_correction(
    current_value: float, previous_value: float, max_jump: float = 1.5
) -> float:
    """
    Apply anti-flip correction to prevent discontinuities at ±π boundaries.

    When rotations wrap around from +π to -π (or vice versa), this function
    adjusts the current value to maintain smooth continuity with the previous value.

    For jumps that exceed *max_jump* after unwrapping, the function moves
    toward the target at *max_jump* rate instead of hard-rejecting.  This
    prevents joints from getting permanently stuck when a legitimate fast
    motion triggers the sanity check, while still damping true extraction
    artifacts (quaternion singularity flips).

    Args:
        current_value: The current angle value (in radians).
        previous_value: The previous angle value (in radians).
        max_jump: Maximum allowed step per frame in radians (default: 1.5 rad ≈ 86°).

    Returns:
        The corrected angle that maintains continuity with the previous value.
    """
    diff = current_value - previous_value

    if diff > math.pi:
        current_value = current_value - 2 * math.pi
    elif diff < -math.pi:
        current_value = current_value + 2 * math.pi

    # Rate-limit: approach target at max_jump speed instead of hard reject
    clamped_diff = current_value - previous_value
    if abs(clamped_diff) > max_jump:
        direction = 1.0 if clamped_diff > 0 else -1.0
        return previous_value + direction * max_jump

    return current_value


def apply_velocity_limiting(
    raw_value: float, last_value: float, velocity_limit: float, dt: float
) -> tuple[float, bool]:
    """
    Apply velocity limiting (rate-limiting) to prevent exceeding hardware constraints.

    Args:
        raw_value: The unclamped target value.
        last_value: The previous frame's value.
        velocity_limit: Maximum allowed velocity (units/second).
        dt: Time delta since last frame (seconds).

    Returns:
        Tuple of (limited_value, was_limited). was_limited indicates if limiting was applied.
    """
    if velocity_limit <= 0.0:
        return raw_value, False

    raw_diff = raw_value - last_value

    # Re-apply anti-flip to the delta
    if raw_diff > math.pi:
        raw_diff -= 2 * math.pi
    elif raw_diff < -math.pi:
        raw_diff += 2 * math.pi

    current_velocity = abs(raw_diff) / dt

    if current_velocity > velocity_limit:
        max_step = velocity_limit * dt
        safe_diff = max_step if raw_diff > 0 else -max_step
        return last_value + safe_diff, True

    return raw_value, False


def apply_exponential_smoothing(
    target_value: float, last_value: float, smoothing_factor: float
) -> float:
    """
    Apply exponential moving average smoothing to a value.

    Simulates joint inertia and motor lag, creating smooth interpolation between frames.

    Args:
        target_value: The desired target value for this frame.
        last_value: The value from the previous frame.
        smoothing_factor: Blending factor (0.0=full smooth, 1.0=no smoothing).

    Returns:
        The smoothed value.
    """
    alpha = 1.0 - smoothing_factor
    return (alpha * target_value) + ((1.0 - alpha) * last_value)


def clamp_to_limits(value: float, lower_limit: float, upper_limit: float) -> float:
    """
    Clamp a value to specified joint limits.

    Args:
        value: The value to clamp.
        lower_limit: Minimum allowed value.
        upper_limit: Maximum allowed value.

    Returns:
        The clamped value.
    """
    return max(min(value, upper_limit), lower_limit)


def get_bone_property(bone, property_name: str, default_value=None):
    """
    Safely retrieve a custom property from a bone with a default fallback.

    Args:
        bone: The Blender pose bone object.
        property_name: Name of the custom property to retrieve.
        default_value: Value to return if property doesn't exist.

    Returns:
        The property value or the default if not found.
    """
    return bone.get(property_name, default_value)


def set_bone_property(bone, property_name: str, value):
    """
    Set a custom property on a bone.

    Args:
        bone: The Blender pose bone object.
        property_name: Name of the custom property to set.
        value: The value to set.
    """
    bone[property_name] = value
