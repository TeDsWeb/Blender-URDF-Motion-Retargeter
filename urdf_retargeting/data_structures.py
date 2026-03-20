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
    bvh_root_bone_name: StringProperty(
        name="BVH Root",
        description="Optional BVH root bone for chain-length estimation (ancestor of BVH Target)",
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
    solver_orientation_weight: FloatProperty(
        name="Solver Orientation Weight",
        description="How strongly the IK solver targets end-effector orientation (0=position only, 1=full orientation)",
        default=0.55,
        min=0.0,
        max=1.0,
    )
    use_auto_blend_override: BoolProperty(
        name="Auto Blend Override",
        description="Use chain-specific adaptive Hybrid IK parameters",
        default=False,
    )
    auto_blend_min: FloatProperty(
        name="Auto Blend Min",
        description="Chain-specific minimum IK blend in adaptive Hybrid mode",
        default=0.2,
        min=0.0,
        max=1.0,
    )
    auto_blend_error_low: FloatProperty(
        name="Auto Blend Error Low",
        description="Below this error, adaptive Hybrid IK uses minimum strength",
        default=0.01,
        min=0.0001,
        max=0.2,
        subtype="DISTANCE",
    )
    auto_blend_error_high: FloatProperty(
        name="Auto Blend Error High",
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
    precompute_on_apply: BoolProperty(
        name="Precompute on Apply",
        description="Bake the retargeted motion into keyframes when Apply Mapping is pressed",
        default=False,
    )
    precompute_disable_live_after_apply: BoolProperty(
        name="Disable Live After Precompute",
        description="Turn off live retargeting after precompute so playback uses baked keyframes only",
        default=True,
    )
    retargeting_method: EnumProperty(
        name="Retargeting Method",
        description="Retargeting backend",
        items=[
            (
                "FK_ONLY",
                "FK Only",
                "Use FK mapping only without IK correction",
            ),
            (
                "IK_ONLY",
                "IK Only (Debug)",
                "Debug mode: skip FK baseline and solve only from IK chains",
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
    output_joint_smoothing: FloatProperty(
        name="Output Joint Smoothing",
        description="Low-pass EMA filter on joint angles (actuator space)",
        default=0.1,
        min=0.0,
        max=1.0,
    )
    output_smoothing_policy: EnumProperty(
        name="Output Smoothing Policy",
        description=("How output joint smoothing is combined with IK target smoothing"),
        items=[
            (
                "AUTO",
                "Auto",
                "Reduce output smoothing automatically when IK target smoothing is already high",
            ),
            (
                "IK_REDUCED",
                "IK Reduced",
                "Always reduce output smoothing when IK contributes",
            ),
            (
                "ALWAYS",
                "Always Apply",
                "Always use Joint Smoothing unchanged",
            ),
        ],
        default="AUTO",
    )
    ik_output_smoothing_min: FloatProperty(
        name="IK Output Smoothing Min",
        description=(
            "Minimum output smoothing factor when IK is active (higher means less post smoothing)"
        ),
        default=0.7,
        min=0.0,
        max=1.0,
    )
    ik_only_mode: BoolProperty(
        name="IK Only Mode",
        description="Skip FK baseline and solve motion using IK chains only",
        default=False,
    )
    max_jump_threshold: FloatProperty(
        name="Max Jump Threshold",
        description="Maximum allowed angle jump in radians (rejects larger jumps as extraction artifacts)",
        default=0.1745,
        min=0.01,
        max=3.14159,
        subtype="ANGLE",
    )
    solver_iterations: IntProperty(
        name="Solver Iterations",
        description="Maximum FABRIK iterations per kinematic chain and frame",
        default=12,
        min=1,
        max=64,
    )
    bake_watchdog_enabled: BoolProperty(
        name="Bake Watchdog",
        description="Detect very slow bake frames and warn in the status/progress output",
        default=True,
    )
    bake_watchdog_frame_ms: FloatProperty(
        name="Watchdog Frame Limit (ms)",
        description="Warn (or optionally abort) when a single baked frame exceeds this duration",
        default=250.0,
        min=10.0,
        max=10000.0,
    )
    bake_watchdog_abort: BoolProperty(
        name="Abort On Watchdog",
        description="Abort bake when a single frame exceeds the watchdog frame limit",
        default=False,
    )
    stability_debug_metrics: BoolProperty(
        name="Debug Stability Metrics",
        description="Collect and expose per-frame jitter diagnostics in scene properties",
        default=False,
    )
    solver_tolerance: FloatProperty(
        name="Solver Tolerance",
        description="Stop the IK solve when the end-effector is closer than this distance to the target",
        default=0.01,
        min=0.0001,
        max=0.25,
        subtype="DISTANCE",
    )
    solver_step_limit: FloatProperty(
        name="Solver Max Step",
        description="Maximum joint-angle change per CCD update to avoid unstable flips",
        default=0.12,
        min=0.005,
        max=1.0,
        subtype="ANGLE",
    )
    solver_target_scale: FloatProperty(
        name="Solver Target Scale",
        description="Manual amplitude scale for BVH end-effector motion in kinematic IK",
        default=1.0,
        min=0.1,
        max=2.0,
    )
    solver_proportion_blend: FloatProperty(
        name="Solver Proportion Blend",
        description="Blend between absolute target matching (0) and proportion-scaled motion transfer (1)",
        default=0.75,
        min=0.0,
        max=1.0,
    )
    solver_ground_lock_strength: FloatProperty(
        name="Solver Ground Lock",
        description="For configured foot targets: keep vertical end-effector position close to calibrated ground pose while foot is grounded",
        default=0.7,
        min=0.0,
        max=1.0,
    )
    solver_target_smoothing: FloatProperty(
        name="Solver Target Smoothing",
        description="Low-pass filter on end-effector target positions",
        default=0.25,
        min=0.0,
        max=0.98,
    )
    hybrid_master_blend: FloatProperty(
        name="Hybrid Master Blend",
        description="Strength of IK correction on top of FK baseline (0 = FK only, 1 = full IK correction)",
        default=0.5,
        min=0.0,
        max=1.0,
    )
    hybrid_auto_blend: BoolProperty(
        name="Adaptive Hybrid Blend",
        description=(
            "Scale IK correction strength by end-effector error in Hybrid mode "
            "(small error => less IK, large error => more IK)"
        ),
        default=True,
    )
    hybrid_blend_min: FloatProperty(
        name="Hybrid Blend Min",
        description="Minimum IK blend fraction used by adaptive Hybrid mode",
        default=0.2,
        min=0.0,
        max=1.0,
    )
    hybrid_blend_error_low: FloatProperty(
        name="Hybrid Error Low",
        description="At or below this end-effector error, adaptive IK uses minimum strength",
        default=0.01,
        min=0.0001,
        max=0.2,
        subtype="DISTANCE",
    )
    hybrid_blend_error_high: FloatProperty(
        name="Hybrid Error High",
        description="At or above this end-effector error, adaptive IK uses full strength",
        default=0.08,
        min=0.001,
        max=0.5,
        subtype="DISTANCE",
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
    adaptive_foot_pinning: BoolProperty(
        name="Adaptive Foot Pinning",
        description=(
            "Use smooth residual-dependent XY/yaw correction limits "
            "instead of fixed hard per-frame caps"
        ),
        default=True,
    )
    foot_pin_adaptive_gain: FloatProperty(
        name="Pin Adaptive Gain",
        description="Response gain for adaptive foot-pinning step limits",
        default=1.5,
        min=0.1,
        max=6.0,
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
    correction_decay_airborne_only: BoolProperty(
        name="Decay Airborne Only",
        description="Apply correction decay only when no stance foot is grounded",
        default=True,
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
