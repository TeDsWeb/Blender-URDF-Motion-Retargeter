"""
Blender operators for retargeting workflow and URDF import.

Provides operators for mapping generation, retargeting initialization,
pose calibration, and URDF file import.
"""

import bpy
import os
import mathutils
from bpy.types import Operator
from bpy_extras.io_utils import ImportHelper
from bpy_extras.anim_utils import action_get_channelbag_for_slot
from .urdf import parse_urdf
from .armature import create_urdf_armature, bind_meshes
from .retargeting import get_lowest_z_world


class OT_GenerateMappingList(Operator):
    """Generate initial BVH-to-URDF bone mapping list from BVH rig."""

    bl_idname = "object.generate_mapping_list"
    bl_label = "Generate Mapping List"
    bl_description = "Generates BVH bone list from selected BVH rig"

    def execute(self, context):
        """Create a mapping entry for each BVH bone."""
        scene = context.scene
        settings = scene.bvh_mapping_settings
        bvh_rig = scene.bvh_rig_object

        if not bvh_rig:
            return {"CANCELLED"}

        # Clear existing mappings
        settings.mappings.clear()

        # Create mapping for each BVH bone
        for ub in [pb.name for pb in bvh_rig.pose.bones]:
            item = settings.mappings.add()
            item.bvh_bone_name = ub

        settings.active_mapping_index = 0
        settings.active_urdf_index = 0
        return {"FINISHED"}


class OT_ApplyBVHMapping(Operator):
    """Reset URDF rig and enable live retargeting with optional smoothing."""

    bl_idname = "object.apply_bvh_mapping"
    bl_label = "Apply Mapping"
    bl_description = "Resets rig to neutral pose at frame 0 and starts live retargeting"

    def check_bvh_rotation_mode(self, bvh_obj: bpy.types.Object) -> bool:
        """
        Check if all BVH bones use QUATERNION rotation mode.

        Args:
            bvh_obj: The BVH armature object.

        Returns:
            True if all pose bones use quaternion rotation.
        """
        if bvh_obj.type != "ARMATURE":
            return False

        for pbone in bvh_obj.pose.bones:
            if pbone.rotation_mode != "QUATERNION":
                return False
        return True

    def apply_zero_lag_smoothing(
        self, context, bvh_obj: bpy.types.Object, smoothing_factor: float
    ) -> bpy.types.Object:
        """
        Create a copy of BVH object and apply Zero-Lag F-Curve smoothing.

        Performs bidirectional filtering (forward + backward pass) on quaternion
        rotations and position vectors to reduce noise while preserving motion.

        Args:
            context: The Blender context.
            bvh_obj: The original BVH armature.
            smoothing_factor: Smoothing strength (0.0=no smoothing, 1.0=max).

        Returns:
            The new smoothed BVH armature object.
        """
        # Remove existing smoothed copy if present
        smoothed_name = f"{bvh_obj.name}_smoothed"
        if smoothed_name in bpy.data.objects:
            bvh_obj.hide_viewport = False
            context.view_layer.objects.active = bvh_obj
            bpy.data.objects.remove(bpy.data.objects[smoothed_name], do_unlink=True)

        # Create a duplicate armature with animation data
        new_obj = bvh_obj.copy()
        new_obj.data = bvh_obj.data.copy()
        new_obj.name = smoothed_name
        if bvh_obj.animation_data and bvh_obj.animation_data.action:
            new_obj.animation_data.action = bvh_obj.animation_data.action.copy()

        context.collection.objects.link(new_obj)

        # Hide original and activate smoothed copy
        bvh_obj.hide_viewport = True
        context.view_layer.objects.active = new_obj

        # Ensure quaternion mode
        bpy.ops.object.mode_set(mode="POSE")
        for pbone in new_obj.pose.bones:
            pbone.rotation_mode = "QUATERNION"
        bpy.ops.object.mode_set(mode="OBJECT")

        # Get animation data
        action = new_obj.animation_data.action
        if not action:
            return new_obj

        slot = new_obj.animation_data.action_slot
        channelbag = action_get_channelbag_for_slot(action, slot)
        if not channelbag:
            return new_obj

        # Group F-Curves by data path
        curves_by_group = {}
        for fc in channelbag.fcurves:
            key = fc.data_path
            if key not in curves_by_group:
                curves_by_group[key] = []
            curves_by_group[key].append(fc)

        # Apply Zero-Lag smoothing to each group
        alpha = 1.0 - smoothing_factor
        for path, curves in curves_by_group.items():
            curves.sort(key=lambda x: x.array_index)
            num_keys = len(curves[0].keyframe_points)
            if num_keys < 2:
                continue

            is_quat = "rotation_quaternion" in path
            is_loc = "location" in path

            # Only process quaternions and locations
            if not (is_quat or is_loc):
                continue

            # Load keyframe data
            all_data = []
            for fc in curves:
                coords = [0.0] * (num_keys * 2)
                fc.keyframe_points.foreach_get("co", coords)
                all_data.append(coords)

            # Convert to mathutils types
            points = []
            for i in range(num_keys):
                idx = i * 2 + 1
                if is_quat and len(curves) == 4:
                    points.append(
                        mathutils.Quaternion(
                            (
                                all_data[0][idx],
                                all_data[1][idx],
                                all_data[2][idx],
                                all_data[3][idx],
                            )
                        )
                    )
                else:
                    points.append(
                        mathutils.Vector(
                            (all_data[0][idx], all_data[1][idx], all_data[2][idx])
                        )
                    )

            # Zero-Lag smoothing: forward and backward passes
            smoothed = [p.copy() for p in points]

            # Forward pass
            for i in range(1, num_keys):
                if is_quat:
                    smoothed[i] = smoothed[i - 1].slerp(points[i], alpha)
                else:
                    smoothed[i] = smoothed[i - 1].lerp(points[i], alpha)

            # Backward pass (Zero-Lag)
            for i in range(num_keys - 2, -1, -1):
                if is_quat:
                    smoothed[i] = smoothed[i + 1].slerp(smoothed[i], alpha)
                else:
                    smoothed[i] = smoothed[i + 1].lerp(smoothed[i], alpha)

            # Write back smoothed values
            for i, fc in enumerate(curves):
                for k in range(num_keys):
                    all_data[i][k * 2 + 1] = smoothed[k][i]
                fc.keyframe_points.foreach_set("co", all_data[i])
                fc.update()

        return new_obj

    def execute(self, context):
        """Reset and initialize retargeting."""
        scene = context.scene
        settings = scene.bvh_mapping_settings
        bvh_obj = scene.bvh_rig_object
        urdf_obj = scene.urdf_rig_object

        if not urdf_obj or not bvh_obj:
            self.report({"ERROR"}, "URDF and BVH rigs must be selected!")
            return {"CANCELLED"}

        # Disable retargeting during setup
        scene.bvh_mapping_settings.live_retarget = False
        scene.frame_set(0)

        # Reset URDF bones to neutral pose
        for pb in urdf_obj.pose.bones:
            pb.rotation_mode = "QUATERNION"

            l_min = pb.get("limit_lower", -3.14)
            l_max = pb.get("limit_upper", 3.14)
            neutral = max(min(0.0, l_max), l_min)

            pb.rotation_quaternion = mathutils.Quaternion((0, 1, 0), neutral)

            # Clear caches
            for key in ("offset", "_joint_angle", "_last_urdf_angle"):
                if key in pb:
                    del pb[key]

        # Clear global smoothing cache
        if "_bvh_smooth_cache" in scene:
            scene["_bvh_smooth_cache"] = {}

        # Apply zero-lag smoothing if enabled
        if settings.bvh_smoothing > 0.0:
            if not self.check_bvh_rotation_mode(bvh_obj):
                self.report(
                    {"ERROR"},
                    "Smoothing requires BVH to be imported with Quaternion rotations!",
                )
                return {"CANCELLED"}

            self.report(
                {"INFO"},
                f"Zero-Lag Smoothing BVH Animation (Factor: {settings.bvh_smoothing})...",
            )
            scene.smoothed_bvh_rig_object = self.apply_zero_lag_smoothing(
                context, bvh_obj, settings.bvh_smoothing
            )
        else:
            scene.smoothed_bvh_rig_object = bvh_obj

        # Calibrate reference poses
        bpy.ops.object.calibrate_rest_pose()

        # Calculate foot height offset
        urdf_foot_names = []
        for item in settings.mappings:
            if item.bvh_bone_name in [settings.foot_l_name, settings.foot_r_name]:
                urdf_foot_names = [b.urdf_bone_name for b in item.urdf_bones]
                break

        min_foot_z = float("inf")
        for u_f_name in urdf_foot_names:
            u_foot = urdf_obj.pose.bones.get(u_f_name)
            if u_foot:
                z = (urdf_obj.matrix_world @ u_foot.matrix).to_translation().z
                if z < min_foot_z:
                    min_foot_z = z

        scene["urdf_foot_height_offset"] = min_foot_z

        # Enable retargeting
        scene.bvh_mapping_settings.live_retarget = True

        # Burn in frame 0
        for _ in range(100):
            scene.frame_set(0)
            context.view_layer.update()

        self.report({"INFO"}, "Rig reset to frame 0. Retargeting active.")
        return {"FINISHED"}


class OT_CalibrateRestPose(Operator):
    """Calibrate mapping offsets based on current BVH pose."""

    bl_idname = "object.calibrate_rest_pose"
    bl_label = "Calibrate Rest Pose"
    bl_description = "Saves the current BVH rotation as the neutral reference."

    def execute(self, context):
        """Store current poses as reference."""
        scene = context.scene
        settings = context.scene.bvh_mapping_settings
        bvh = context.scene.smoothed_bvh_rig_object
        urdf = context.scene.urdf_rig_object

        if not bvh or not urdf:
            self.report({"ERROR"}, "BVH and URDF rigs required!")
            return {"CANCELLED"}

        # Store root pose
        bvh_root_mat = bvh.matrix_world @ bvh.pose.bones[0].matrix
        scene["ref_root_pos"] = bvh_root_mat.to_translation().copy()
        scene["ref_root_rot"] = bvh_root_mat.to_quaternion().copy()

        # Store reference rotation for each mapping
        count = 0
        for item in settings.mappings:
            bvh_bone = bvh.pose.bones.get(item.bvh_bone_name)
            if bvh_bone:
                item.ref_rot = bvh_bone.matrix_basis.to_quaternion()
                count += 1

        # Calculate BVH and URDF floor offsets
        if (
            settings.foot_l_name in bvh.pose.bones
            and settings.foot_r_name in bvh.pose.bones
        ):
            lowest_bvh_z = min(
                (bvh.matrix_world @ bvh.pose.bones.get(settings.foot_l_name).matrix)
                .to_translation()
                .z,
                (bvh.matrix_world @ bvh.pose.bones.get(settings.foot_r_name).matrix)
                .to_translation()
                .z,
            )
            if scene.get("bvh_floor_offset", None) is None:
                scene["bvh_floor_offset"] = lowest_bvh_z

            if scene.get("urdf_height_offset", None) is None:
                scene["urdf_height_offset"] = abs(get_lowest_z_world(urdf))

        self.report({"INFO"}, f"Calibrated {count} bones. Reference set.")
        return {"FINISHED"}


class OT_AddBVHBone(Operator):
    """Add a new URDF bone entry to a BVH mapping."""

    bl_idname = "object.add_urdf_bone"
    bl_label = "Add URDF Bone"
    bl_description = "Add URDF Bone to Mapping"
    bvh_bone_name: bpy.props.StringProperty()

    def execute(self, context):
        """Add a new URDF bone to the mapping."""
        m = next(
            i
            for i in context.scene.bvh_mapping_settings.mappings
            if i.bvh_bone_name == self.bvh_bone_name
        )
        m.urdf_bones.add()
        return {"FINISHED"}


class OT_RemoveBVHBone(Operator):
    """Remove a URDF bone entry from a BVH mapping."""

    bl_idname = "object.remove_urdf_bone"
    bl_label = "Remove URDF Bone"
    bl_description = "Remove URDF Bone from Mapping"
    bvh_bone_name: bpy.props.StringProperty()

    def execute(self, context):
        """Remove the selected URDF bone from the mapping."""
        m = next(
            i
            for i in context.scene.bvh_mapping_settings.mappings
            if i.bvh_bone_name == self.bvh_bone_name
        )
        m.urdf_bones.remove(context.scene.bvh_mapping_settings.active_urdf_index)
        return {"FINISHED"}


class IMPORT_OT_urdf_humanoid(Operator, ImportHelper):
    """Import and rig a URDF humanoid robot for retargeting."""

    bl_idname = "import_scene.urdf_humanoid"
    bl_label = "Import URDF"
    bl_description = (
        "Imports a URDF humanoid robot and creates an armature with bound meshes."
    )
    filename_ext = ".urdf"

    def execute(self, context):
        """Parse URDF and create rig."""
        robot = parse_urdf(self.filepath)
        arm, link_mats = create_urdf_armature(robot)
        bind_meshes(robot, arm, link_mats, os.path.dirname(self.filepath))

        context.scene.urdf_rig_object = arm

        # Store movable joints for export
        arm["urdf_joint_order"] = [
            j.name
            for j in robot.joints.values()
            if j.type in {"revolute", "continuous", "prismatic"}
        ]

        return {"FINISHED"}


def menu_func_import(self, context):
    """Add import option to File > Import menu."""
    self.layout.operator(
        IMPORT_OT_urdf_humanoid.bl_idname, text="URDF Humanoid (.urdf)"
    )
