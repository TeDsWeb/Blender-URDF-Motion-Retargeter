"""
Property Group data structures for BVH-to-URDF mapping UI.

Defines the data models for storing mapping configurations, bone constraints,
and retargeting parameters.
"""

import os

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


def _mapping_preset_items(self, context):
    """Build dropdown items from bundled preset JSON files."""
    base_dir = os.path.dirname(__file__)
    preset_dir = os.path.join(base_dir, "presets", "mappings")

    items = []
    if os.path.isdir(preset_dir):
        for name in sorted(os.listdir(preset_dir)):
            if name.lower().endswith(".json"):
                items.append((name, name, f"Load preset {name}"))

    if not items:
        items.append(("", "<no presets found>", "No preset JSON files found"))
    return items


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


class KinematicChainMapping(PropertyGroup):
    """Defines a BVH target and a URDF joint chain for IK retargeting."""

    label: StringProperty(
        name="Label",
        description="Optional name for this kinematic chain",
        default="",
    )
    bvh_target_bone_name: StringProperty(
        name="BVH Target",
        description="BVH end-effector bone whose position should be matched",
        default="",
    )
    urdf_root_bone_name: StringProperty(
        name="URDF Root Joint",
        description="First URDF joint in the kinematic chain",
        default="",
    )
    urdf_end_bone_name: StringProperty(
        name="URDF End Joint",
        description="Last URDF joint / end effector in the kinematic chain",
        default="",
    )
    influence: FloatProperty(
        name="Influence",
        description="How strongly the IK chain should follow the BVH target",
        default=1.0,
        min=0.0,
        max=1.0,
    )
    use_hybrid_adaptive_override: BoolProperty(
        name="Adaptive Override",
        description="Use chain-specific adaptive Hybrid IK parameters",
        default=False,
    )
    hybrid_min_ik_blend: FloatProperty(
        name="Min IK",
        description="Chain-specific minimum IK blend in adaptive Hybrid mode",
        default=0.2,
        min=0.0,
        max=1.0,
    )
    hybrid_error_low: FloatProperty(
        name="Error Low",
        description="Below this error, adaptive Hybrid IK uses minimum strength",
        default=0.01,
        min=0.0001,
        max=0.2,
        subtype="DISTANCE",
    )
    hybrid_error_high: FloatProperty(
        name="Error High",
        description="Above this error, adaptive Hybrid IK uses full strength",
        default=0.08,
        min=0.001,
        max=0.5,
        subtype="DISTANCE",
    )


class DefaultPoseJoint(PropertyGroup):
    """Editable default-pose angle for one URDF joint."""

    joint_name: StringProperty(
        name="Joint Name",
        description="URDF joint name",
        default="",
    )
    angle: FloatProperty(
        name="Angle",
        description="Default angle for this joint used in export blend phases and the initial retarget start pose",
        subtype="ANGLE",
        default=0.0,
    )


class BVHMappingSettings(PropertyGroup):
    """Master settings for BVH-to-URDF retargeting."""

    mappings: CollectionProperty(
        type=BVHMappingItem, description="List of BVH-to-URDF bone mappings"
    )
    kinematic_chains: CollectionProperty(
        type=KinematicChainMapping,
        description="List of BVH-to-URDF kinematic target chains",
    )
    active_mapping_index: IntProperty(
        name="Active Mapping Index",
        description="Index of the currently selected mapping in the UI",
    )
    active_kinematic_chain_index: IntProperty(
        name="Active Kinematic Chain Index",
        description="Index of the currently selected kinematic chain in the UI",
    )
    active_urdf_index: IntProperty(
        name="Active URDF Index",
        description="Index of the currently selected URDF bone in the mapping",
    )

    # Mapping Preset Metadata
    mapping_preset_name: StringProperty(
        name="Preset Name",
        description="Human-readable preset name (e.g., BoosterK1_OptiTrack_v1)",
        default="",
    )
    mapping_robot_profile: StringProperty(
        name="Robot Profile",
        description="Target robot profile this mapping was created for",
        default="",
    )
    mapping_mocap_profile: StringProperty(
        name="MoCap Profile",
        description="Source skeleton/profile this mapping was created for",
        default="",
    )
    mapping_library_preset: EnumProperty(
        name="Library Preset",
        description="Select a JSON preset from urdf_retargeting/presets/mappings",
        items=_mapping_preset_items,
    )

    # Retargeting Control
    live_retarget: BoolProperty(
        name="Live Retargeting",
        description="Enable real-time retargeting on frame changes",
        default=False,
    )
    retargeting_method: EnumProperty(
        name="Retargeting Method",
        description="Select the retargeting backend (ANGLE/KINEMATIC are legacy and mapped to HYBRID at runtime)",
        items=[
            (
                "ANGLE",
                "Angle Mapping (Legacy)",
                "Legacy mode kept for old scenes; runtime uses HYBRID",
            ),
            (
                "KINEMATIC",
                "Kinematic IK (Legacy)",
                "Legacy mode kept for old scenes; runtime uses HYBRID",
            ),
            (
                "HYBRID",
                "Hybrid FK + IK",
                "Use FK as baseline and IK as corrective end-effector refinement",
            ),
        ],
        default="HYBRID",
    )
    ui_show_root_advanced: BoolProperty(
        name="Show Root Advanced",
        description="Show advanced root-motion controls",
        default=False,
        options={"SKIP_SAVE"},
    )
    ui_show_fk_advanced: BoolProperty(
        name="Show FK Advanced",
        description="Show advanced FK baseline controls",
        default=False,
        options={"SKIP_SAVE"},
    )
    ui_show_ik_advanced: BoolProperty(
        name="Show IK Advanced",
        description="Show advanced IK correction controls",
        default=False,
        options={"SKIP_SAVE"},
    )
    ui_show_performance_advanced: BoolProperty(
        name="Show Performance Advanced",
        description="Show advanced performance controls",
        default=False,
        options={"SKIP_SAVE"},
    )
    ui_show_foot_advanced: BoolProperty(
        name="Show Foot Advanced",
        description="Show advanced foot contact controls",
        default=False,
        options={"SKIP_SAVE"},
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
    max_jump_threshold: FloatProperty(
        name="Max Jump Threshold",
        description="Maximum allowed angle jump in radians (rejects larger jumps as extraction artifacts)",
        default=0.1745,
        min=0.01,
        max=3.14159,
        subtype="ANGLE",
    )
    ik_iterations: IntProperty(
        name="IK Iterations",
        description="Maximum CCD iterations per kinematic chain and frame",
        default=12,
        min=1,
        max=64,
    )
    ik_tolerance: FloatProperty(
        name="IK Tolerance",
        description="Stop the IK solve when the end-effector is closer than this distance to the target",
        default=0.01,
        min=0.0001,
        max=0.25,
        subtype="DISTANCE",
    )
    ik_max_step_angle: FloatProperty(
        name="IK Max Step",
        description="Maximum joint-angle change per CCD update to avoid unstable flips",
        default=0.12,
        min=0.005,
        max=1.0,
        subtype="ANGLE",
    )
    ik_target_scale: FloatProperty(
        name="IK Target Scale",
        description="Manual amplitude scale for BVH end-effector motion in kinematic IK",
        default=1.0,
        min=0.1,
        max=2.0,
    )
    ik_proportion_blend: FloatProperty(
        name="IK Proportion Blend",
        description="Blend between absolute target matching (0) and proportion-scaled motion transfer (1)",
        default=0.75,
        min=0.0,
        max=1.0,
    )
    ik_ground_lock_strength: FloatProperty(
        name="IK Ground Lock",
        description="For configured foot targets: keep vertical end-effector position close to calibrated ground pose while foot is grounded",
        default=0.7,
        min=0.0,
        max=1.0,
    )
    ik_target_smoothing: FloatProperty(
        name="IK Target Smoothing",
        description="Low-pass filter on end-effector target positions",
        default=0.25,
        min=0.0,
        max=0.98,
    )
    ik_joint_smoothing: FloatProperty(
        name="IK Joint Smoothing",
        description="Low-pass smoothing on solved IK joint angles",
        default=0.12,
        min=0.0,
        max=1.0,
    )
    ik_micro_deadzone: FloatProperty(
        name="IK Micro Deadzone",
        description="Ignore tiny IK angle updates near convergence to reduce micro-jitter",
        default=0.0015,
        min=0.0,
        max=0.05,
        subtype="ANGLE",
    )
    hybrid_ik_blend: FloatProperty(
        name="Hybrid IK Blend",
        description="Strength of IK correction on top of FK baseline (0 = FK only, 1 = full IK correction)",
        default=0.5,
        min=0.0,
        max=1.0,
    )
    hybrid_adaptive_ik: BoolProperty(
        name="Adaptive Hybrid IK",
        description=(
            "Scale IK correction strength by end-effector error in Hybrid mode "
            "(small error => less IK, large error => more IK)"
        ),
        default=True,
    )
    hybrid_min_ik_blend: FloatProperty(
        name="Hybrid Min IK",
        description="Minimum IK blend fraction used by adaptive Hybrid mode",
        default=0.2,
        min=0.0,
        max=1.0,
    )
    hybrid_error_low: FloatProperty(
        name="Hybrid Error Low",
        description="At or below this end-effector error, adaptive IK uses minimum strength",
        default=0.01,
        min=0.0001,
        max=0.2,
        subtype="DISTANCE",
    )
    hybrid_error_high: FloatProperty(
        name="Hybrid Error High",
        description="At or above this end-effector error, adaptive IK uses full strength",
        default=0.08,
        min=0.001,
        max=0.5,
        subtype="DISTANCE",
    )
    hybrid_blend_smoothing: FloatProperty(
        name="Hybrid Blend Smoothing",
        description="Temporal smoothing of adaptive per-chain IK blend (higher = smoother)",
        default=0.8,
        min=0.0,
        max=0.98,
    )
    hybrid_realtime_guard: BoolProperty(
        name="Hybrid Realtime Guard",
        description=(
            "Dynamically reduce IK iterations when retargeting time exceeds "
            "the current frame budget"
        ),
        default=True,
    )
    hybrid_min_iterations: IntProperty(
        name="Hybrid Min Iterations",
        description="Lower bound for IK iterations when realtime guard throttles",
        default=4,
        min=1,
        max=64,
    )
    hybrid_ik_frame_skip: IntProperty(
        name="Hybrid IK Frame Skip",
        description=(
            "Skip N frames between Hybrid IK passes (0 = IK every frame, "
            "1 = every second frame)"
        ),
        default=0,
        min=0,
        max=8,
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
    foot_contact_hysteresis: FloatProperty(
        name="Contact Hysteresis",
        description=(
            "Additional height hysteresis (meters) to avoid rapid foot "
            "contact toggling near the jump threshold"
        ),
        default=0.005,
        min=0.0,
        max=0.05,
    )
    foot_pin_xy_max_step: FloatProperty(
        name="Pin XY Max Step",
        description=(
            "Maximum XY root correction per frame for foot pinning "
            "(smaller values reduce jitter but allow short-lived micro-slip)"
        ),
        default=0.03,
        min=0.001,
        max=0.2,
        subtype="DISTANCE",
    )
    foot_pin_yaw_max_step: FloatProperty(
        name="Pin Yaw Max Step",
        description=(
            "Maximum yaw correction per frame for foot pinning "
            "(limits sudden heading jumps)"
        ),
        default=0.12,
        min=0.005,
        max=1.0,
        subtype="ANGLE",
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
    correction_decay: FloatProperty(
        name="Correction Decay (%)",
        description=(
            "Decay percentage for foot-planting corrections "
            "(reference: 120 FPS). Value is interpreted as a "
            "percentage: 0.05 ≈ 0.5 % per frame at 120 FPS. "
            "Automatically scaled to the scene frame-rate so the "
            "effective decay per second stays constant. "
            "Pulls the URDF root back toward the BVH trajectory "
            "to prevent unbounded drift "
            "(0 = sticky feet / no decay, 1 = full decay / no correction)"
        ),
        default=0.05,
        min=0.0,
        max=1.0,
        step=0.01,
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

    # Export
    target_hz: IntProperty(
        name="Export Hz",
        description="Target frame rate for trajectory export",
        default=50,
        min=1,
        max=240,
    )
    export_from_frame: IntProperty(
        name="Export From Frame",
        description="First frame to include when exporting (scene frame if 0)",
        default=0,
        min=0,
    )
    export_to_frame: IntProperty(
        name="Export To Frame",
        description="Last frame to include when exporting (scene frame if 0)",
        default=0,
        min=0,
    )
    export_blend_in_seconds: FloatProperty(
        name="Blend In (s)",
        description="Duration in seconds to blend from neutral pose into the exported motion",
        default=0.0,
        min=0.0,
    )
    export_blend_out_seconds: FloatProperty(
        name="Blend Out (s)",
        description="Duration in seconds to blend from exported motion back to neutral pose",
        default=0.0,
        min=0.0,
    )
    export_end_pose_hold_seconds: FloatProperty(
        name="End Pose Hold (s)",
        description="Duration in seconds to hold the final exported pose at the end",
        default=0.0,
        min=0.0,
    )
    use_custom_default_pose: BoolProperty(
        name="Use Neutral Pose",
        description="Legacy compatibility flag for the stored neutral pose settings",
        default=True,
        options={"HIDDEN"},
    )
    default_pose_root_rotation: FloatVectorProperty(
        name="Neutral Root Rotation",
        description="Stored neutral root rotation (Euler XYZ)",
        subtype="EULER",
        size=3,
        default=(0.0, 0.0, 0.0),
    )
    default_pose_joints: CollectionProperty(
        type=DefaultPoseJoint,
        description="Editable neutral joint angles used for export blend phases and initial retarget pose",
    )
    default_pose_active_index: IntProperty(
        name="Neutral Pose Joint Index",
        description="Active neutral-pose joint index in UI",
        default=0,
        min=0,
    )

    # Import
    import_use_meta_hz: BoolProperty(
        name="Use Meta Hz",
        description="If a matching _meta.json exists, use its export_hz",
        default=True,
    )
    import_manual_hz: FloatProperty(
        name="Manual Hz",
        description="Override the sample rate (Hz) when not using meta",
        default=0.0,
        min=0.0,
    )
    import_set_scene_fps: BoolProperty(
        name="Set Scene FPS",
        description="Set scene FPS to the import sample rate",
        default=True,
    )
