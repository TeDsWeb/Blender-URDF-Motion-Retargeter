"""
Blender operators for retargeting workflow and URDF import.

Provides operators for mapping generation, retargeting initialization,
pose calibration, and URDF file import.
"""

import bpy
import os
import time
import mathutils
from bpy.types import Operator
from bpy_extras.io_utils import ImportHelper, ExportHelper
from bpy_extras.anim_utils import action_get_channelbag_for_slot
from .urdf import parse_urdf
from .armature import create_urdf_armature, bind_meshes
from .retargeting import get_lowest_z_world, apply_custom_default_pose_to_urdf
from .mapping_io import (
    export_mapping_to_json,
    import_mapping_from_json,
    count_non_quaternion_bones,
    get_preset_library_dir,
    ensure_preset_library_dir,
    sanitize_preset_filename,
)


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

    def _validate_required_fields(self, settings, bvh_obj: bpy.types.Object) -> bool:
        """Validate mandatory fields before enabling live retargeting."""
        missing_fields = []

        if not settings.foot_l_name:
            missing_fields.append("Foot Configuration -> Left Foot")
        if not settings.foot_r_name:
            missing_fields.append("Foot Configuration -> Right Foot")

        if missing_fields:
            self.report(
                {"WARNING"},
                "Apply Mapping cancelled: required fields are missing: "
                + ", ".join(missing_fields),
            )
            return False

        invalid_bones = []
        if bvh_obj.pose.bones.get(settings.foot_l_name) is None:
            invalid_bones.append(
                f"Left Foot '{settings.foot_l_name}' was not found in the BVH rig"
            )
        if bvh_obj.pose.bones.get(settings.foot_r_name) is None:
            invalid_bones.append(
                f"Right Foot '{settings.foot_r_name}' was not found in the BVH rig"
            )

        if invalid_bones:
            self.report(
                {"WARNING"},
                "Apply Mapping cancelled: " + "; ".join(invalid_bones),
            )
            return False

        if settings.foot_l_name == settings.foot_r_name:
            self.report(
                {"WARNING"},
                "Apply Mapping cancelled: Left Foot and Right Foot must be different bones.",
            )
            return False

        return True

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

    def _set_action_linear_interpolation(self, action: bpy.types.Action) -> None:
        """Set all baked keyframes to linear interpolation for deterministic playback."""
        if not action:
            return

        slot = None
        channelbag = None
        if bpy.app.version >= (4, 0, 0):
            slot = getattr(action, "slots", None)
            slot = slot[0] if slot else None
            channelbag = action_get_channelbag_for_slot(action, slot) if slot else None

        if channelbag and getattr(channelbag, "fcurves", None):
            fcurves = channelbag.fcurves
        else:
            fcurves = action.fcurves

        for fcurve in fcurves:
            for key in fcurve.keyframe_points:
                key.interpolation = "LINEAR"

    def _cleanup_previous_bakes(self, urdf_obj: bpy.types.Object) -> None:
        """Remove old bake actions from this URDF rig to avoid buildup."""
        bake_prefix = f"{urdf_obj.name}_RetargetBake"

        current_action = None
        if urdf_obj.animation_data:
            current_action = urdf_obj.animation_data.action
            if current_action and current_action.name.startswith(bake_prefix):
                urdf_obj.animation_data.action = None

        for act in list(bpy.data.actions):
            if not act.name.startswith(bake_prefix):
                continue
            if current_action and act.name == current_action.name:
                continue
            if act.users == 0:
                bpy.data.actions.remove(act, do_unlink=True)

    def _bake_retargeted_motion(
        self,
        context,
        scene: bpy.types.Scene,
        settings,
        urdf_obj: bpy.types.Object,
        eval_start: int,
        eval_end: int,
        key_start: int,
    ) -> tuple[int, int]:
        """Bake retargeted transforms into a new URDF action."""
        eval_start = int(eval_start)
        eval_end = int(eval_end)
        key_start = int(key_start)

        if eval_end < eval_start:
            return (0, 0)

        self._cleanup_previous_bakes(urdf_obj)

        urdf_obj.animation_data_create()
        action_name = f"{urdf_obj.name}_RetargetBake"
        baked_action = bpy.data.actions.new(name=action_name)
        urdf_obj.animation_data.action = baked_action

        previous_quality_flag = scene.get("_export_full_quality", None)
        previous_ik_iterations = int(getattr(settings, "ik_iterations", 12))
        offline_ik_iterations = int(
            getattr(settings, "quality_ik_iterations", previous_ik_iterations)
        )

        scene["_export_full_quality"] = True
        settings.ik_iterations = offline_ik_iterations

        total_eval_frames = max(0, eval_end - eval_start + 1)
        wm = context.window_manager
        wm.progress_begin(0, max(1, total_eval_frames))

        keyed_frames = 0
        keyed_bones = 0
        try:
            for frame in range(eval_start, eval_end + 1):
                progress = frame - eval_start
                wm.progress_update(progress)
                context.workspace.status_text_set(
                    f"Bake Retarget: Frame {frame}/{eval_end}"
                )
                if progress % 5 == 0:
                    try:
                        bpy.ops.wm.redraw_timer(type="DRAW_WIN_SWAP", iterations=1)
                    except Exception:
                        pass
                scene.frame_set(frame)
                context.view_layer.update()

                if frame < key_start:
                    continue

                urdf_obj.keyframe_insert(data_path="location", frame=frame)
                urdf_obj.keyframe_insert(data_path="rotation_quaternion", frame=frame)

                for pb in urdf_obj.pose.bones:
                    pb.rotation_mode = "QUATERNION"
                    pb.keyframe_insert(data_path="rotation_quaternion", frame=frame)
                    if "_joint_angle" in pb:
                        pb.keyframe_insert(data_path='["_joint_angle"]', frame=frame)

                keyed_frames += 1
                keyed_bones = len(urdf_obj.pose.bones)
        finally:
            settings.ik_iterations = previous_ik_iterations
            wm.progress_end()
            context.workspace.status_text_set(None)

        self._set_action_linear_interpolation(baked_action)

        if previous_quality_flag is None:
            if "_export_full_quality" in scene:
                del scene["_export_full_quality"]
        else:
            scene["_export_full_quality"] = previous_quality_flag

        if settings.precompute_disable_live_after_apply:
            settings.live_retarget = False

        return keyed_frames, keyed_bones

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

        if not self._validate_required_fields(settings, bvh_obj):
            return {"CANCELLED"}

        # Disable retargeting during setup
        scene.bvh_mapping_settings.live_retarget = False
        scene.frame_set(0)

        # Clear only per-frame transient state keys (not calibration keys)
        for _k in (
            "_persistent_foot_correction",
            "_persistent_foot_rot_correction",
            "_active_anchor_name",
            "_anchor_world_pos_xy",
            "_anchor_world_yaw",
            "_anchor_bvh_yaw",
            "_foot_positions",
            "_bvh_smooth_cache",
            "_kinematic_target_cache",
            "_ik_custom_default_ref_cache",
        ):
            if _k in scene:
                del scene[_k]

        # Reset URDF bones to neutral pose
        for pb in urdf_obj.pose.bones:
            pb.rotation_mode = "QUATERNION"

            l_min = pb.get("limit_lower", -3.14)
            l_max = pb.get("limit_upper", 3.14)
            neutral = max(min(0.0, l_max), l_min)

            pb.rotation_quaternion = mathutils.Quaternion((0, 1, 0), neutral)

            # Clear per-frame caches (not joint limits or joint type which are static)
            for key in (
                "offset",
                "_joint_angle",
                "_last_urdf_angle",
                "_ik_last_angle",
                "is_velocity_limited",
            ):
                if key in pb:
                    del pb[key]

        # Reinitialize global smoothing cache
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

        # Ensure retargeting starts from the same pose definition used by export
        # to avoid mismatched IK references when custom defaults are enabled.
        apply_custom_default_pose_to_urdf(urdf_obj, settings)

        # Initialize export frame range defaults (only in operator context)
        try:
            if (
                getattr(settings, "export_from_frame", 0) == 0
                and getattr(settings, "export_to_frame", 0) == 0
                and bvh_obj is not None
            ):
                settings.export_from_frame = scene.frame_start
                settings.export_to_frame = scene.frame_end

        except Exception:
            # Defensive: if properties are not present or writing fails, ignore
            pass

        # Calibrate reference poses (sets ref_root_pos, ref_root_rot, bvh_floor_offset, urdf_height_offset)
        bpy.ops.object.calibrate_rest_pose()

        # Enable retargeting
        scene.bvh_mapping_settings.live_retarget = True

        # Burn in frame 0
        for _ in range(100):
            scene.frame_set(0)
            context.view_layer.update()

        # Calculate foot height offset from current URDF foot position
        # Find foot bone names from mappings
        urdf_foot_names = []
        for item in settings.mappings:
            if item.bvh_bone_name in [settings.foot_l_name, settings.foot_r_name]:
                urdf_foot_names.extend([b.urdf_bone_name for b in item.urdf_bones])

        # Compute minimum Z of all foot bones
        min_foot_z = float("inf")
        if urdf_foot_names:
            bpy.context.view_layer.update()
            for u_f_name in urdf_foot_names:
                u_foot = urdf_obj.pose.bones.get(u_f_name)
                if u_foot:
                    z = (urdf_obj.matrix_world @ u_foot.matrix).to_translation().z
                    if z < min_foot_z:
                        min_foot_z = z

        # Set foot height offset, with fallback to 0 if feet are not found
        if min_foot_z == float("inf"):
            min_foot_z = 0.0
            self.report(
                {"WARNING"}, "No foot bones found; using foot height offset 0.0"
            )

        scene["urdf_foot_height_offset"] = min_foot_z

        skip_apply_precompute = bool(scene.get("_skip_apply_precompute", False))
        if settings.precompute_on_apply and not skip_apply_precompute:
            scene["_bake_apply_request"] = True
            result = bpy.ops.object.bake_bvh_mapping("INVOKE_DEFAULT")
            if result != {"RUNNING_MODAL"} and "_bake_apply_request" in scene:
                del scene["_bake_apply_request"]
            return {"FINISHED"}

        self.report({"INFO"}, "Rig reset to frame 0. Retargeting active.")
        return {"FINISHED"}


class OT_BakeRetargetedMotion(Operator):
    """Bake the currently configured retargeting to a deterministic action."""

    bl_idname = "object.bake_bvh_mapping"
    bl_label = "Bake Retarget"
    bl_description = (
        "Apply mapping and bake motion to URDF keyframes for optimized playback"
    )

    _timer = None
    _orig_frame = 0
    _frame = 0
    _eval_end = 0
    _key_start = 0
    _keyed_frames = 0
    _keyed_bones = 0
    _previous_ik_iterations = 0
    _previous_quality_flag = None
    _disable_live_before = True
    _settings = None
    _scene = None
    _urdf_obj = None
    _baked_action = None
    _watchdog_enabled = False
    _watchdog_frame_ms = 250.0
    _watchdog_abort = False
    _slow_frames = 0

    def _set_action_linear_interpolation(self, action: bpy.types.Action) -> None:
        if not action:
            return

        slot = None
        channelbag = None
        if bpy.app.version >= (4, 0, 0):
            slot = getattr(action, "slots", None)
            slot = slot[0] if slot else None
            channelbag = action_get_channelbag_for_slot(action, slot) if slot else None

        fcurves = (
            channelbag.fcurves
            if channelbag and getattr(channelbag, "fcurves", None)
            else action.fcurves
        )
        for fcurve in fcurves:
            for key in fcurve.keyframe_points:
                key.interpolation = "LINEAR"

    def _cleanup_previous_bakes(self, urdf_obj: bpy.types.Object) -> None:
        bake_prefix = f"{urdf_obj.name}_RetargetBake"

        current_action = None
        if urdf_obj.animation_data:
            current_action = urdf_obj.animation_data.action
            if current_action and current_action.name.startswith(bake_prefix):
                urdf_obj.animation_data.action = None

        for act in list(bpy.data.actions):
            if not act.name.startswith(bake_prefix):
                continue
            if current_action and act.name == current_action.name:
                continue
            if act.users == 0:
                bpy.data.actions.remove(act, do_unlink=True)

    def _finalize(self, context, cancelled: bool = False):
        settings = self._settings
        scene = self._scene

        if settings is not None:
            settings.ik_iterations = self._previous_ik_iterations
            if cancelled:
                settings.live_retarget = self._disable_live_before
            elif settings.precompute_disable_live_after_apply:
                settings.live_retarget = False

        if scene is not None:
            if self._previous_quality_flag is None:
                if "_export_full_quality" in scene:
                    del scene["_export_full_quality"]
            else:
                scene["_export_full_quality"] = self._previous_quality_flag

            if "_bake_apply_request" in scene:
                del scene["_bake_apply_request"]

            scene.frame_set(self._orig_frame)

        if self._baked_action and not cancelled:
            self._set_action_linear_interpolation(self._baked_action)

        wm = context.window_manager
        if self._timer is not None:
            wm.event_timer_remove(self._timer)
            self._timer = None
        wm.progress_end()
        context.workspace.status_text_set(None)

    def modal(self, context, event):
        if event.type == "ESC":
            self._finalize(context, cancelled=True)
            self.report({"WARNING"}, "Bake abgebrochen")
            return {"CANCELLED"}

        if event.type != "TIMER":
            return {"PASS_THROUGH"}

        if self._frame > self._eval_end:
            self._finalize(context, cancelled=False)
            self.report(
                {"INFO"},
                (
                    "Rig reset and precomputed to keyframes "
                    f"({self._keyed_frames} frames, {self._keyed_bones} bones)."
                ),
            )
            return {"FINISHED"}

        progress = self._frame - self._scene.frame_start
        context.window_manager.progress_update(progress)
        context.workspace.status_text_set(
            f"Bake Retarget: Frame {self._frame}/{self._eval_end} | ESC to Cancel"
        )

        t_frame_start = time.perf_counter()
        self._scene.frame_set(self._frame)
        context.view_layer.update()

        if self._frame >= self._key_start:
            self._urdf_obj.keyframe_insert(data_path="location", frame=self._frame)
            self._urdf_obj.keyframe_insert(
                data_path="rotation_quaternion", frame=self._frame
            )

            for pb in self._urdf_obj.pose.bones:
                pb.rotation_mode = "QUATERNION"
                pb.keyframe_insert(data_path="rotation_quaternion", frame=self._frame)
                if "_joint_angle" in pb:
                    pb.keyframe_insert(data_path='["_joint_angle"]', frame=self._frame)

            self._keyed_frames += 1
            self._keyed_bones = len(self._urdf_obj.pose.bones)

        frame_ms = (time.perf_counter() - t_frame_start) * 1000.0
        if self._watchdog_enabled and frame_ms > self._watchdog_frame_ms:
            self._slow_frames += 1
            if self._slow_frames <= 3 or (self._slow_frames % 25) == 0:
                self.report(
                    {"WARNING"},
                    (
                        f"Bake watchdog: slow frame {self._frame} "
                        f"({frame_ms:.1f} ms > {self._watchdog_frame_ms:.1f} ms)"
                    ),
                )

            if self._watchdog_abort:
                self._finalize(context, cancelled=True)
                self.report(
                    {"ERROR"},
                    (
                        f"Bake watchdog aborted at frame {self._frame}: "
                        f"{frame_ms:.1f} ms exceeded limit {self._watchdog_frame_ms:.1f} ms"
                    ),
                )
                return {"CANCELLED"}

        self._frame += 1
        return {"RUNNING_MODAL"}

    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings

        use_current_setup = bool(scene.get("_bake_apply_request", False))

        if not use_current_setup:
            scene["_skip_apply_precompute"] = True
            try:
                result = bpy.ops.object.apply_bvh_mapping()
            finally:
                if "_skip_apply_precompute" in scene:
                    del scene["_skip_apply_precompute"]

            if result != {"FINISHED"}:
                return result

        urdf_obj = scene.urdf_rig_object
        if not urdf_obj:
            if "_bake_apply_request" in scene:
                del scene["_bake_apply_request"]
            self.report({"ERROR"}, "Kein URDF Rig zum Backen gefunden")
            return {"CANCELLED"}

        key_from = int(getattr(settings, "export_from_frame", scene.frame_start))
        key_to = int(getattr(settings, "export_to_frame", scene.frame_end))
        key_from = max(scene.frame_start, min(key_from, scene.frame_end))
        key_to = max(scene.frame_start, min(key_to, scene.frame_end))
        if key_from > key_to:
            key_from, key_to = key_to, key_from

        self._scene = scene
        self._settings = settings
        self._urdf_obj = urdf_obj
        self._orig_frame = scene.frame_current
        self._frame = scene.frame_start
        self._eval_end = key_to
        self._key_start = key_from
        self._keyed_frames = 0
        self._keyed_bones = 0
        self._slow_frames = 0
        self._disable_live_before = bool(settings.live_retarget)
        self._watchdog_enabled = bool(getattr(settings, "bake_watchdog_enabled", True))
        self._watchdog_frame_ms = float(
            getattr(settings, "bake_watchdog_frame_ms", 250.0)
        )
        self._watchdog_abort = bool(getattr(settings, "bake_watchdog_abort", False))

        self._cleanup_previous_bakes(urdf_obj)
        urdf_obj.animation_data_create()
        self._baked_action = bpy.data.actions.new(name=f"{urdf_obj.name}_RetargetBake")
        urdf_obj.animation_data.action = self._baked_action

        self._previous_quality_flag = scene.get("_export_full_quality", None)
        self._previous_ik_iterations = int(getattr(settings, "ik_iterations", 12))
        scene["_export_full_quality"] = True
        settings.ik_iterations = int(
            getattr(settings, "quality_ik_iterations", self._previous_ik_iterations)
        )

        total_eval_frames = max(1, self._eval_end - scene.frame_start + 1)
        wm = context.window_manager
        wm.progress_begin(0, total_eval_frames)
        self._timer = wm.event_timer_add(0.001, window=context.window)
        wm.modal_handler_add(self)
        return {"RUNNING_MODAL"}


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

        # Calculate BVH and URDF floor offsets (always recalculate, don't check if None)
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
        else:
            # Fallback without foot anchors so operator never crashes on missing setup.
            if scene.get("bvh_floor_offset", None) is None:
                lowest_bvh_z = min(
                    (bvh.matrix_world @ pb.matrix).to_translation().z
                    for pb in bvh.pose.bones
                )
                scene["bvh_floor_offset"] = lowest_bvh_z
            if scene.get("urdf_height_offset", None) is None:
                scene["urdf_height_offset"] = abs(get_lowest_z_world(urdf))

            self.report(
                {"WARNING"},
                "Foot Configuration is incomplete or invalid. "
                "Using generic floor offsets; configure Left/Right Foot bones for stable foot contact.",
            )

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


class OT_AddKinematicChain(Operator):
    """Add a new kinematic IK chain definition."""

    bl_idname = "object.add_kinematic_chain"
    bl_label = "Add Kinematic Chain"
    bl_description = "Add a BVH target to URDF chain mapping for IK retargeting"

    def execute(self, context):
        settings = context.scene.bvh_mapping_settings
        chain = settings.kinematic_chains.add()
        chain.label = f"Chain {len(settings.kinematic_chains)}"
        settings.active_kinematic_chain_index = len(settings.kinematic_chains) - 1
        return {"FINISHED"}


class OT_RemoveKinematicChain(Operator):
    """Remove the selected kinematic IK chain definition."""

    bl_idname = "object.remove_kinematic_chain"
    bl_label = "Remove Kinematic Chain"
    bl_description = "Remove the selected BVH target to URDF chain mapping"

    def execute(self, context):
        settings = context.scene.bvh_mapping_settings
        if not settings.kinematic_chains:
            return {"CANCELLED"}

        index = settings.active_kinematic_chain_index
        index = max(0, min(index, len(settings.kinematic_chains) - 1))
        settings.kinematic_chains.remove(index)
        settings.active_kinematic_chain_index = max(0, index - 1)
        return {"FINISHED"}


class OT_ExportMappingJSON(Operator, ExportHelper):
    """Export current mapping and kinematic chains as JSON preset."""

    bl_idname = "object.export_mapping_json"
    bl_label = "Export Mapping JSON"
    bl_description = "Save BVH/URDF mapping and kinematic chains as a JSON preset"

    filename_ext = ".json"
    filter_glob: bpy.props.StringProperty(default="*.json", options={"HIDDEN"})

    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings

        urdf_name = scene.urdf_rig_object.name if scene.urdf_rig_object else ""
        bvh_name = scene.bvh_rig_object.name if scene.bvh_rig_object else ""

        metadata = {
            "preset_name": settings.mapping_preset_name,
            "robot_profile": settings.mapping_robot_profile,
            "mocap_profile": settings.mapping_mocap_profile,
            "urdf_rig_name": urdf_name,
            "bvh_rig_name": bvh_name,
        }
        export_mapping_to_json(self.filepath, settings, metadata=metadata)

        self.report(
            {"INFO"},
            f"Mapping preset exported: {os.path.basename(self.filepath)}",
        )
        return {"FINISHED"}


class OT_ImportMappingJSON(Operator, ImportHelper):
    """Import mapping and kinematic chains from JSON preset."""

    bl_idname = "object.import_mapping_json"
    bl_label = "Import Mapping JSON"
    bl_description = "Load BVH/URDF mapping and kinematic chains from a JSON preset"

    filename_ext = ".json"
    filter_glob: bpy.props.StringProperty(default="*.json", options={"HIDDEN"})

    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings

        metadata = import_mapping_from_json(self.filepath, settings)

        # Keep metadata in UI so users can maintain human-readable presets.
        settings.mapping_preset_name = metadata.get("preset_name", "")
        settings.mapping_robot_profile = metadata.get("robot_profile", "")
        settings.mapping_mocap_profile = metadata.get("mocap_profile", "")

        # Soft compatibility warning: preset target/source profile vs. current rigs.
        current_urdf = scene.urdf_rig_object.name if scene.urdf_rig_object else ""
        current_bvh = scene.bvh_rig_object.name if scene.bvh_rig_object else ""
        preset_urdf = metadata.get("urdf_rig_name", "")
        preset_bvh = metadata.get("bvh_rig_name", "")

        mismatch = []
        if preset_urdf and current_urdf and preset_urdf != current_urdf:
            mismatch.append(f"URDF preset={preset_urdf}, scene={current_urdf}")
        if preset_bvh and current_bvh and preset_bvh != current_bvh:
            mismatch.append(f"BVH preset={preset_bvh}, scene={current_bvh}")

        if mismatch:
            self.report(
                {"WARNING"},
                "Preset may not match current rig pair: " + " | ".join(mismatch),
            )

        # Quaternion warning only (no conversion), as requested.
        non_quat_bvh = count_non_quaternion_bones(scene.bvh_rig_object)
        non_quat_urdf = count_non_quaternion_bones(scene.urdf_rig_object)
        if non_quat_bvh > 0 or non_quat_urdf > 0:
            self.report(
                {"WARNING"},
                (
                    "Detected non-quaternion rotation mode bones "
                    f"(BVH={non_quat_bvh}, URDF={non_quat_urdf}). "
                    "Please convert in Blender if needed."
                ),
            )

        self.report(
            {"INFO"},
            f"Mapping preset imported: {os.path.basename(self.filepath)}",
        )
        return {"FINISHED"}


class OT_LoadMappingPresetLibrary(Operator):
    """Load selected preset from bundled preset library folder."""

    bl_idname = "object.load_mapping_preset_library"
    bl_label = "Load Library Preset"
    bl_description = "Load selected preset from urdf_retargeting/presets/mappings"

    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings
        preset_file = settings.mapping_library_preset

        if not preset_file:
            self.report({"ERROR"}, "No library preset selected.")
            return {"CANCELLED"}

        path = os.path.join(get_preset_library_dir(), preset_file)
        if not os.path.isfile(path):
            self.report({"ERROR"}, f"Preset not found: {preset_file}")
            return {"CANCELLED"}

        metadata = import_mapping_from_json(path, settings)
        settings.mapping_preset_name = metadata.get("preset_name", "")
        settings.mapping_robot_profile = metadata.get("robot_profile", "")
        settings.mapping_mocap_profile = metadata.get("mocap_profile", "")

        self.report({"INFO"}, f"Loaded library preset: {preset_file}")
        return {"FINISHED"}


class OT_SaveMappingPresetLibrary(Operator):
    """Save current mapping to bundled preset library folder."""

    bl_idname = "object.save_mapping_preset_library"
    bl_label = "Save To Library"
    bl_description = "Save current mapping preset to urdf_retargeting/presets/mappings"

    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings

        if not settings.mapping_preset_name.strip():
            self.report(
                {"ERROR"},
                "Preset Name is required before saving to library.",
            )
            return {"CANCELLED"}

        directory = ensure_preset_library_dir()
        filename = sanitize_preset_filename(settings.mapping_preset_name)
        path = os.path.join(directory, filename)

        metadata = {
            "preset_name": settings.mapping_preset_name,
            "robot_profile": settings.mapping_robot_profile,
            "mocap_profile": settings.mapping_mocap_profile,
            "urdf_rig_name": (
                scene.urdf_rig_object.name if scene.urdf_rig_object else ""
            ),
            "bvh_rig_name": scene.bvh_rig_object.name if scene.bvh_rig_object else "",
        }
        export_mapping_to_json(path, settings, metadata=metadata)
        settings.mapping_library_preset = filename

        self.report({"INFO"}, f"Saved library preset: {filename}")
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


class OT_ClearScene(Operator):
    """Delete addon-created rigs and clear runtime caches (confirm required)."""

    bl_idname = "object.clear_retarget_scene"
    bl_label = "Clear Scene (Delete Addon Objects & Caches)"
    bl_description = "Remove only objects created by this addon and clear related caches (irreversible)"

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        scene = context.scene

        # Disable live retargeting if present
        try:
            scene.bvh_mapping_settings.live_retarget = False
        except Exception:
            pass

        # Helper: check if object is safe to delete (addon-owned)
        def is_addon_object(obj):
            """
            Return True only if object has both:
            - An addon-specific property marker (_link_to_bone, urdf_joint_order)
            - AND an addon naming pattern (_smoothed, _mesh_, _Rig)
            """
            has_addon_prop = (
                obj.get("_link_to_bone") is not None
                or obj.get("urdf_joint_order") is not None
            )
            has_addon_name = (
                obj.name.endswith("_smoothed")
                or "_mesh_" in obj.name
                or obj.name.endswith("_Rig")
            )
            return has_addon_prop and has_addon_name

        # Collect scene-referenced rigs (these are always addon-owned)
        scene_ref_objs = set()
        for key in ("smoothed_bvh_rig_object", "bvh_rig_object", "urdf_rig_object"):
            obj = scene.get(key)
            if obj and isinstance(obj, bpy.types.Object):
                scene_ref_objs.add(obj.name)

        to_delete = set()

        # Add scene-referenced rigs
        to_delete.update(scene_ref_objs)

        # Add objects that pass the strict addon check
        for obj in bpy.data.objects:
            if is_addon_object(obj):
                to_delete.add(obj.name)

        # Include children of addon objects
        for obj in list(bpy.data.objects):
            if obj.parent and obj.parent.name in to_delete:
                to_delete.add(obj.name)

        removed = []
        # Delete collected addon objects only
        for name in list(to_delete):
            if name in bpy.data.objects:
                obj = bpy.data.objects[name]
                # Remove action if strictly linked to this object
                if obj.animation_data and obj.animation_data.action:
                    act = obj.animation_data.action
                    try:
                        bpy.data.actions.remove(act, do_unlink=True)
                    except Exception:
                        pass
                try:
                    bpy.data.objects.remove(obj, do_unlink=True)
                    removed.append(name)
                except Exception:
                    pass

        # Clean orphan datablocks that may have been created by addon
        for mesh in list(bpy.data.meshes):
            if mesh.users == 0:
                try:
                    bpy.data.meshes.remove(mesh, do_unlink=True)
                except Exception:
                    pass
        for arm in list(bpy.data.armatures):
            if arm.users == 0:
                try:
                    bpy.data.armatures.remove(arm, do_unlink=True)
                except Exception:
                    pass
        for mat in list(bpy.data.materials):
            if mat.users == 0:
                try:
                    bpy.data.materials.remove(mat, do_unlink=True)
                except Exception:
                    pass
        for act in list(bpy.data.actions):
            if act.users == 0:
                try:
                    bpy.data.actions.remove(act, do_unlink=True)
                except Exception:
                    pass

        # Clear transient scene keys (calibration and runtime)
        for k in (
            "_persistent_foot_correction",
            "_active_anchor_name",
            "_anchor_world_pos_xy",
            "_foot_positions",
            "_bvh_smooth_cache",
            "urdf_foot_height_offset",
            "ref_root_pos",
            "ref_root_rot",
            "bvh_floor_offset",
            "urdf_height_offset",
        ):
            if k in scene:
                try:
                    del scene[k]
                except Exception:
                    pass

        # Clear addon-specific object properties from remaining objects
        for obj in bpy.data.objects:
            if obj.name in removed:
                continue
            for prop in ("_link_to_bone", "urdf_joint_order"):
                if prop in obj:
                    try:
                        del obj[prop]
                    except Exception:
                        pass

        # Clear scene pointers
        for key in ("smoothed_bvh_rig_object", "bvh_rig_object", "urdf_rig_object"):
            if key in scene:
                try:
                    del scene[key]
                except Exception:
                    pass

        self.report({"INFO"}, f"Cleared addon objects: {len(removed)}")
        return {"FINISHED"}
