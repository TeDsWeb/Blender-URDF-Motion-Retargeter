"""
Property Group data structures for BVH-to-URDF mapping UI.

Defines the data models for storing mapping configurations, bone constraints,
and retargeting parameters.
"""

from bpy.props import (
    StringProperty,
    CollectionProperty,
    EnumProperty,
    BoolProperty,
    FloatVectorProperty,
    FloatProperty,
    IntProperty,
)
from bpy.types import PropertyGroup


class BVHMappingBone(PropertyGroup):
    """Defines how a single URDF bone is driven by a BVH bone."""

    urdf_bone_name: StringProperty(
        name="URDF Bone", description="The URDF joint/bone to be controlled"
    )
    source_axis: EnumProperty(
        name="Source Axis",
        description="Which axis of the BVH bone rotation to extract",
        items=[("X", "X", ""), ("Y", "Y", ""), ("Z", "Z", "")],
        default="X",
    )
    sign: EnumProperty(
        name="Sign",
        description="Whether to invert the extracted rotation",
        items=[("POS", "+", ""), ("NEG", "-", "")],
        default="NEG",
    )
    invert_alignment: BoolProperty(
        name="Invert Alignment",
        description="Invert the foot alignment correction for this bone",
        default=False,
    )
    neutral_offset: FloatProperty(
        name="Neutral Offset",
        description="Calibration offset added to the extracted angle",
        subtype="ANGLE",
        default=0.0,
    )


class BVHMappingItem(PropertyGroup):
    """A complete mapping from one BVH bone to one or more URDF bones."""

    urdf_bones: CollectionProperty(
        type=BVHMappingBone,
        description="List of URDF bones controlled by this BVH bone",
    )
    bvh_bone_name: StringProperty(
        name="BVH Bone", description="The source BVH bone name"
    )
    ref_rot: FloatVectorProperty(
        name="Reference Rotation",
        description="The reference (neutral) rotation as a quaternion",
        size=4,
        default=(1.0, 0.0, 0.0, 0.0),
    )


class BVHMappingSettings(PropertyGroup):
    """Master settings for BVH-to-URDF retargeting."""

    mappings: CollectionProperty(
        type=BVHMappingItem, description="List of BVH-to-URDF bone mappings"
    )
    active_mapping_index: IntProperty(
        name="Active Mapping Index",
        description="Index of the currently selected mapping in the UI",
    )
    active_urdf_index: IntProperty(
        name="Active URDF Index",
        description="Index of the currently selected URDF bone in the mapping",
    )

    # Retargeting Control
    live_retarget: BoolProperty(
        name="Live Retargeting",
        description="Enable real-time retargeting on frame changes",
        default=False,
    )

    # Motion Smoothing
    bvh_smoothing: FloatProperty(
        name="BVH Smoothing",
        description="Zero-Lag SLERP/LERP filter on BVH joint angles (source space)",
        default=0.9,
        min=0.0,
        max=1.0,
    )
    joint_smoothing: FloatProperty(
        name="Joint Smoothing",
        description="Low-pass EMA filter on joint angles (actuator space)",
        default=0.1,
        min=0.0,
        max=1.0,
    )

    # Foot Contact & Anchoring
    foot_l_name: StringProperty(
        name="Left Foot Bone",
        description="BVH bone name for the left foot (used for contact detection)",
        default="",
    )
    foot_r_name: StringProperty(
        name="Right Foot Bone",
        description="BVH bone name for the right foot (used for contact detection)",
        default="",
    )
    jump_threshold: FloatProperty(
        name="Jump Threshold",
        description="Minimum vertical movement to register a jump (in meters)",
        default=0.01,
        min=0.0,
    )

    # Foot Flattening
    foot_flattening_height: FloatProperty(
        name="Flatten Height",
        description="Distance from floor (meters) where foot is fully aligned to BVH again",
        default=0.15,
        min=0.01,
        max=1.0,
    )
    foot_flattening_strength: FloatProperty(
        name="Flatten Strength",
        description="Influence of the ground alignment (1.0 = Force Flat, 0.0 = Off)",
        default=1.0,
        min=0.0,
        max=1.0,
    )

    # Root Motion
    root_scale: FloatProperty(
        name="Root Scale",
        description="Scale factor for root position movement",
        default=1.0,
    )
    location_offset: FloatVectorProperty(
        name="URDF Pos Offset",
        description="Manual offset to root position (XY plane)",
        subtype="TRANSLATION",
        size=2,
        default=(0.0, 0.0),
    )
    rotation_offset: FloatVectorProperty(
        name="URDF Rot Offset",
        description="Manual offset to root rotation (Euler angles)",
        subtype="EULER",
        default=(0.0, 0.0, 0.0),
    )
    bvh_position_offset: FloatVectorProperty(
        name="BVH Pos Offset",
        description="Manual offset to BVH rig position for grounding (XYZ)",
        subtype="TRANSLATION",
        size=3,
        default=(0.0, 0.0, 0.0),
    )

    # Export
    target_hz: IntProperty(
        name="Export Hz",
        description="Target frame rate for trajectory export",
        default=50,
        min=1,
        max=240,
    )
