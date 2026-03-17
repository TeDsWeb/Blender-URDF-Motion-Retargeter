"""
Export functionality for Beyond Mimic trajectory format.

Handles exporting retargeted robot trajectories to CSV and JSON formats
with resampling to a specified target Hz.
"""

import bpy
import os
import csv
import json
import mathutils
from bpy.types import Operator


def _sync_default_pose_joints(settings, joint_names):
    """Ensure default-pose joint collection matches URDF joint order."""
    prev = {item.joint_name: item.angle for item in settings.default_pose_joints}
    settings.default_pose_joints.clear()

    for jn in joint_names:
        item = settings.default_pose_joints.add()
        item.joint_name = jn
        item.angle = prev.get(jn, 0.0)

    if len(settings.default_pose_joints) == 0:
        settings.default_pose_active_index = 0
    else:
        settings.default_pose_active_index = min(
            settings.default_pose_active_index,
            len(settings.default_pose_joints) - 1,
        )


class OT_CaptureDefaultPoseFromCurrent(Operator):
    """Capture the current URDF pose as editable custom default pose."""

    bl_idname = "object.capture_default_pose_from_current"
    bl_label = "Capture Neutral Pose"
    bl_description = "Capture current URDF root rotation and joint angles into the stored neutral pose"

    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings
        urdf = scene.urdf_rig_object

        if not urdf:
            self.report({"ERROR"}, "No URDF rig selected")
            return {"CANCELLED"}

        joint_names = list(urdf.get("urdf_joint_order", []))

        settings.use_custom_default_pose = True

        q = urdf.rotation_quaternion
        e = q.to_euler("XYZ")
        # Keep only roll/pitch editable by user; yaw is taken from motion during export.
        settings.default_pose_root_rotation = (e.x, e.y, 0.0)

        _sync_default_pose_joints(settings, joint_names)

        for item in settings.default_pose_joints:
            pb = urdf.pose.bones.get(item.joint_name)
            if pb is None:
                continue

            l_min = pb.get("limit_lower", -3.14)
            l_max = pb.get("limit_upper", 3.14)
            captured = pb.get("_joint_angle", 0.0)
            item.angle = max(min(captured, l_max), l_min)

        self.report(
            {"INFO"},
            f"Neutral pose captured ({len(settings.default_pose_joints)} joints)",
        )
        return {"FINISHED"}


class OT_ResetDefaultPose(Operator):
    """Reset and disable custom default pose settings."""

    bl_idname = "object.reset_default_pose"
    bl_label = "Reset Neutral Pose"
    bl_description = (
        "Clear the stored neutral pose values and reset them to neutral defaults"
    )

    def execute(self, context):
        settings = context.scene.bvh_mapping_settings
        settings.use_custom_default_pose = True
        settings.default_pose_root_rotation = (0.0, 0.0, 0.0)
        settings.default_pose_joints.clear()
        settings.default_pose_active_index = 0
        self.report({"INFO"}, "Neutral pose reset")
        return {"FINISHED"}


class OT_SyncDefaultPoseJoints(Operator):
    """Sync editable default-pose joint list from URDF joint order."""

    bl_idname = "object.sync_default_pose_joints"
    bl_label = "Sync Neutral Pose Joints"
    bl_description = "Refresh neutral pose joint list from URDF joint order while preserving existing edited values"

    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings
        urdf = scene.urdf_rig_object

        if not urdf:
            self.report({"ERROR"}, "No URDF rig selected")
            return {"CANCELLED"}

        joint_names = list(urdf.get("urdf_joint_order", []))
        _sync_default_pose_joints(settings, joint_names)
        settings.use_custom_default_pose = True

        self.report(
            {"INFO"},
            f"Neutral pose joint list synced ({len(settings.default_pose_joints)} joints)",
        )
        return {"FINISHED"}


class OT_ExportBeyondMimic(Operator):
    """Exports retargeted trajectories to CSV and JSON files with time-based resampling."""

    bl_idname = "object.export_beyond_mimic"
    bl_label = "Export CSV & JSON"
    bl_description = "Select directory; files will be named after the Source BVH rig and resampled to Target Hz"

    directory: bpy.props.StringProperty(name="Export Directory", subtype="DIR_PATH")
    export_from_frame: bpy.props.IntProperty(
        name="Export From Frame",
        description="First Blender frame to include in the exported data (inclusive)",
        default=0,
        min=0,
    )
    export_to_frame: bpy.props.IntProperty(
        name="Export To Frame",
        description="Last Blender frame to include in the exported data (inclusive)",
        default=0,
        min=0,
    )
    default_pose_blend_in_seconds: bpy.props.FloatProperty(
        name="Blend In (s)",
        description="Duration in seconds to blend from neutral pose into motion",
        default=0.0,
        min=0.0,
    )
    default_pose_blend_out_seconds: bpy.props.FloatProperty(
        name="Blend Out (s)",
        description="Duration in seconds to blend from motion back to neutral pose",
        default=0.0,
        min=0.0,
    )
    end_pose_hold_seconds: bpy.props.FloatProperty(
        name="End Pose Hold (s)",
        description="Duration in seconds to hold the final exported pose",
        default=0.0,
        min=0.0,
    )

    # Internal Modal State
    _timer = None
    _duration = 0
    _total_frames = 0
    _raw_frames = []  # per-integer-frame data (Pass 1)
    _data_rows = []  # resampled output   (Pass 2)
    _meta_joints = {}
    _csv_path = ""
    _json_path = ""
    _orig_frame = 0
    _joints = []
    _base_name = ""
    _eval_frame = 0
    _blend_in_seconds = 0.0
    _blend_out_seconds = 0.0
    _end_pose_hold_seconds = 0.0
    _phase = 1

    def modal(self, context, event):
        """Modal handler — Pass 1: evaluate every frame sequentially.

        Walking through every integer frame guarantees that the
        retarget_frame handler accumulates its persistent state
        (foot corrections, smooth caches) in the correct order.
        Resampling to target_hz happens in finish() (Pass 2).
        """
        scene = context.scene
        urdf = scene.urdf_rig_object

        if event.type == "ESC":
            return self.cancel(context)

        if event.type == "TIMER":
            if self._phase == 1 and self._eval_frame <= scene.frame_end:
                # Evaluate this frame (triggers retarget_frame handler)
                scene.frame_set(self._eval_frame)

                # Collect root data (translation + quaternion)
                row = [
                    urdf.location.x,
                    urdf.location.y,
                    urdf.location.z,
                    urdf.rotation_quaternion.x,
                    urdf.rotation_quaternion.y,
                    urdf.rotation_quaternion.z,
                    urdf.rotation_quaternion.w,
                ]

                # Collect joint angles
                for j in self._joints:
                    pb = urdf.pose.bones[j]
                    row.append(pb.get("_joint_angle", 0.0))

                    # Store metadata once per joint
                    if j not in self._meta_joints:
                        self._meta_joints[j] = {
                            "lower": pb.get("limit_lower", -3.14),
                            "upper": pb.get("limit_upper", 3.14),
                        }

                self._raw_frames.append(row)

                # UI update
                progress = self._eval_frame - scene.frame_start
                context.workspace.status_text_set(
                    f"Export Pass 1/4: Frame {self._eval_frame}/{scene.frame_end} | ESC to Cancel"
                )
                context.window_manager.progress_update(progress)
                self._eval_frame += 1
            elif self._phase == 1:
                self._phase = 2

            elif self._phase == 2:
                context.workspace.status_text_set("Export Pass 2/4: Resampling…")
                self._data_rows = self._resample_to_target_hz(scene)
                context.window_manager.progress_update(self._total_frames + 1)
                self._phase = 3

            elif self._phase == 3:
                context.workspace.status_text_set(
                    "Export Pass 3/4: Blending neutral pose…"
                )
                self._data_rows = self._apply_default_pose_blends(
                    scene, urdf, self._data_rows
                )
                context.window_manager.progress_update(self._total_frames + 2)
                self._phase = 4

            elif self._phase == 4:
                context.workspace.status_text_set("Export Pass 4/4: Holding end pose…")
                self._data_rows = self._apply_end_pose_hold(scene, self._data_rows)
                context.window_manager.progress_update(self._total_frames + 3)
                return self.finish(context)

        return {"RUNNING_MODAL"}

    def execute(self, context):
        """Initialize export and start sequential frame evaluation."""
        scene = context.scene
        settings = scene.bvh_mapping_settings
        urdf, bvh = scene.urdf_rig_object, scene.bvh_rig_object

        if not urdf or not bvh or not self.directory:
            self.report({"ERROR"}, "Robot, BVH Rig or Directory missing!")
            return {"CANCELLED"}

        # Initialize paths
        self._base_name = bpy.path.clean_name(bvh.name)
        self._csv_path = os.path.join(self.directory, f"{self._base_name}.csv")
        self._json_path = os.path.join(self.directory, f"{self._base_name}_meta.json")

        # Timing info (used during resampling in finish())
        fps = scene.render.fps
        self._total_frames = scene.frame_end - scene.frame_start
        self._duration = self._total_frames / fps

        # Resolve export frame range (prefer operator props, then UI settings, then scene range)
        if self.export_from_frame > 0:
            exp_from = self.export_from_frame
        elif getattr(settings, "export_from_frame", 0) > 0:
            exp_from = settings.export_from_frame
        else:
            exp_from = scene.frame_start

        if self.export_to_frame > 0:
            exp_to = self.export_to_frame
        elif getattr(settings, "export_to_frame", 0) > 0:
            exp_to = settings.export_to_frame
        else:
            exp_to = scene.frame_end
        # Clamp to scene bounds
        exp_from = max(scene.frame_start, min(exp_from, scene.frame_end))
        exp_to = max(scene.frame_start, min(exp_to, scene.frame_end))
        if exp_from > exp_to:
            exp_from, exp_to = exp_to, exp_from
        self._export_from_frame = exp_from
        self._export_to_frame = exp_to
        self._blend_in_seconds = max(0.0, self.default_pose_blend_in_seconds)
        self._blend_out_seconds = max(0.0, self.default_pose_blend_out_seconds)
        self._end_pose_hold_seconds = max(0.0, self.end_pose_hold_seconds)

        self._joints = list(urdf.get("urdf_joint_order", []))
        self._eval_frame = scene.frame_start
        self._raw_frames = []
        self._data_rows = []
        self._meta_joints = {}
        self._orig_frame = scene.frame_current
        self._phase = 1

        # Export should always evaluate the full-quality retarget path.
        scene["_export_full_quality"] = True

        # Start modal loop — progress tracks frame evaluation
        wm = context.window_manager
        context.workspace.status_text_set(
            f"Exporting frames {self._export_from_frame}..{self._export_to_frame} → {self._base_name} ({settings.target_hz}Hz, in={self._blend_in_seconds:.2f}s, out={self._blend_out_seconds:.2f}s, hold={self._end_pose_hold_seconds:.2f}s)"
        )
        wm.progress_begin(0, self._total_frames + 3)
        self._timer = wm.event_timer_add(0.001, window=context.window)
        wm.modal_handler_add(self)

        return {"RUNNING_MODAL"}

    def _resample_to_target_hz(self, scene):
        """Resample raw per-frame data to target_hz with interpolation.

        Uses linear interpolation for positions and joint angles, and
        quaternion SLERP for root orientation.  Only samples that fall
        within [export_from_frame, export_to_frame] are emitted.

        Returns:
            List of resampled rows.
        """
        settings = scene.bvh_mapping_settings
        fps = scene.render.fps
        target_hz = settings.target_hz

        if not self._raw_frames:
            return []

        total_src = len(self._raw_frames)  # one entry per integer frame
        duration_secs = (total_src - 1) / fps
        num_samples = max(int(duration_secs * target_hz) + 1, 1)

        resampled = []
        for step in range(num_samples):
            t_secs = step / target_hz

            # Fractional source-frame index (relative to frame_start)
            src_idx_f = t_secs * fps

            # Corresponding Blender frame for range check
            blender_frame = scene.frame_start + src_idx_f
            if blender_frame < self._export_from_frame:
                continue
            if blender_frame > self._export_to_frame:
                break

            # Clamp to collected range
            src_idx_f = max(0.0, min(src_idx_f, total_src - 1))
            idx_lo = int(src_idx_f)
            idx_hi = min(idx_lo + 1, total_src - 1)
            frac = src_idx_f - idx_lo

            if idx_lo == idx_hi or frac < 1e-6:
                resampled.append(list(self._raw_frames[idx_lo]))
                continue

            row_a = self._raw_frames[idx_lo]
            row_b = self._raw_frames[idx_hi]
            row = []

            # Root position (indices 0-2): linear interpolation
            for i in range(3):
                row.append(row_a[i] + (row_b[i] - row_a[i]) * frac)

            # Root quaternion (indices 3-6, stored x,y,z,w): SLERP
            qa = mathutils.Quaternion(
                (row_a[6], row_a[3], row_a[4], row_a[5])  # w,x,y,z
            )
            qb = mathutils.Quaternion((row_b[6], row_b[3], row_b[4], row_b[5]))
            qi = qa.slerp(qb, frac)
            row.extend([qi.x, qi.y, qi.z, qi.w])

            # Joint angles (indices 7+): linear interpolation
            for i in range(7, len(row_a)):
                row.append(row_a[i] + (row_b[i] - row_a[i]) * frac)

            resampled.append(row)

        return resampled

    def _build_default_pose_row(
        self,
        scene,
        urdf,
        root_pos_override=None,
        root_motion_quat_override=None,
    ):
        """Build a default pose row (root + joints) for blend phases."""
        settings = scene.bvh_mapping_settings

        if root_pos_override is not None:
            ref_pos = mathutils.Vector(root_pos_override)
        else:
            ref_pos = mathutils.Vector(scene.get("ref_root_pos", urdf.location))

        # Neutral root yaw is intentionally not user-editable.
        base_r = settings.default_pose_root_rotation
        ref_rot = mathutils.Euler((base_r[0], base_r[1], 0.0), "XYZ").to_quaternion()
        custom_joint_angles = {
            item.joint_name: item.angle for item in settings.default_pose_joints
        }

        if root_motion_quat_override is not None:
            motion_q = mathutils.Quaternion(
                (
                    root_motion_quat_override[3],
                    root_motion_quat_override[0],
                    root_motion_quat_override[1],
                    root_motion_quat_override[2],
                )
            )
            user_e = ref_rot.to_euler("XYZ")
            motion_e = motion_q.to_euler("XYZ")
            ref_rot = mathutils.Euler(
                (user_e.x, user_e.y, motion_e.z), "XYZ"
            ).to_quaternion()

        row = [
            ref_pos.x,
            ref_pos.y,
            ref_pos.z,
            ref_rot.x,
            ref_rot.y,
            ref_rot.z,
            ref_rot.w,
        ]

        for j in self._joints:
            pb = urdf.pose.bones.get(j)
            if pb is None:
                row.append(0.0)
                continue

            l_min = pb.get("limit_lower", -3.14)
            l_max = pb.get("limit_upper", 3.14)
            if j in custom_joint_angles:
                angle = custom_joint_angles[j]
            else:
                angle = 0.0
            row.append(max(min(angle, l_max), l_min))

        return row

    def _interpolate_row(self, row_a, row_b, frac):
        """Interpolate full pose row with SLERP for root quaternion."""
        row = []

        for i in range(3):
            row.append(row_a[i] + (row_b[i] - row_a[i]) * frac)

        qa = mathutils.Quaternion((row_a[6], row_a[3], row_a[4], row_a[5]))
        qb = mathutils.Quaternion((row_b[6], row_b[3], row_b[4], row_b[5]))
        qi = qa.slerp(qb, frac)
        row.extend([qi.x, qi.y, qi.z, qi.w])

        for i in range(7, len(row_a)):
            row.append(row_a[i] + (row_b[i] - row_a[i]) * frac)

        return row

    def _apply_default_pose_blends(self, scene, urdf, rows):
        """Prepend/append interpolation phases between default pose and motion."""
        settings = scene.bvh_mapping_settings
        target_hz = settings.target_hz

        if not rows:
            return rows

        blend_in_samples = max(int(round(self._blend_in_seconds * target_hz)), 0)
        blend_out_samples = max(int(round(self._blend_out_seconds * target_hz)), 0)

        if blend_in_samples == 0 and blend_out_samples == 0:
            return rows

        first_motion = rows[0]
        last_motion = rows[-1]
        default_row_in = self._build_default_pose_row(
            scene,
            urdf,
            first_motion[:3],
            first_motion[3:7],
        )
        default_row_out = self._build_default_pose_row(
            scene,
            urdf,
            last_motion[:3],
            last_motion[3:7],
        )
        result = []

        if blend_in_samples > 0:
            for i in range(blend_in_samples):
                frac = i / blend_in_samples
                result.append(self._interpolate_row(default_row_in, first_motion, frac))

        result.extend(rows)

        if blend_out_samples > 0:
            for i in range(blend_out_samples):
                frac = (i + 1) / blend_out_samples
                result.append(self._interpolate_row(last_motion, default_row_out, frac))

        return result

    def _apply_end_pose_hold(self, scene, rows):
        """Append a constant hold of the final pose for the configured duration."""
        if not rows:
            return rows

        target_hz = scene.bvh_mapping_settings.target_hz
        hold_samples = max(int(round(self._end_pose_hold_seconds * target_hz)), 0)

        if hold_samples == 0:
            return rows

        held_row = list(rows[-1])
        rows.extend([held_row.copy() for _ in range(hold_samples)])
        return rows

    def finish(self, context):
        """Write final CSV/JSON after all modal export phases are completed."""
        scene = context.scene
        settings = scene.bvh_mapping_settings
        urdf = scene.urdf_rig_object

        # Create CSV header
        header = [
            "root_tx",
            "root_ty",
            "root_tz",
            "root_qx",
            "root_qy",
            "root_qz",
            "root_qw",
        ] + self._joints

        # Write files
        try:
            with open(self._csv_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(self._data_rows)

            with open(self._json_path, "w") as f:
                json.dump(
                    {
                        # Origin Info
                        "source_bvh": self._base_name,
                        "target_urdf": bpy.path.clean_name(urdf.name),
                        # Timing
                        "source_fps": scene.render.fps,
                        "source_total_frames": self._total_frames,
                        "export_hz": settings.target_hz,
                        "export_frame_range": {
                            "from": self._export_from_frame,
                            "to": self._export_to_frame,
                        },
                        "default_pose_blend": {
                            "enabled": self._blend_in_seconds > 0.0
                            or self._blend_out_seconds > 0.0,
                            "blend_in_seconds": self._blend_in_seconds,
                            "blend_out_seconds": self._blend_out_seconds,
                            "end_pose_hold_seconds": self._end_pose_hold_seconds,
                            "use_custom_default_pose": settings.use_custom_default_pose,
                            "root_position_mode": "match_motion_start_end",
                            "root_yaw_mode": "match_motion_start_end",
                            "root_rotation_user_editable": ["roll", "pitch"],
                        },
                        "source_duration_secs": self._duration,
                        "duration_secs": len(self._data_rows) / settings.target_hz,
                        "total_samples": len(self._data_rows),
                        # Export Settings
                        "bvh_smoothing": settings.bvh_smoothing,
                        "joint_smoothing": settings.joint_smoothing,
                        "retarget_quality": {
                            "mode": "full_quality",
                            "hybrid_ik_blend": settings.hybrid_ik_blend,
                            "realtime_guard_ui_enabled": settings.hybrid_realtime_guard,
                            "frame_skip_ui_value": settings.hybrid_ik_frame_skip,
                            "realtime_guard_applied_during_export": False,
                            "frame_skip_applied_during_export": False,
                            "note": "Export forces full IK quality; realtime guard and frame skip are ignored during frame evaluation.",
                        },
                        # Units
                        "measurement_unit": "meters",
                        "angle_unit": "radians",
                        # Root Definition
                        "root": {
                            "type": "free_floating_base",
                            "frame": "blender_world",
                            "dofs": ["x", "y", "z", "qx", "qy", "qz", "qw"],
                            "representation": "quaternion",
                            "note": "No semantic root bone; base pose is encoded as world-space transform of the URDF armature object",
                        },
                        # Coordinate System
                        "coordinate_system": {
                            "space": "blender_world",
                            "up_axis": "Z",
                            "forward_axis": None,
                            "right_axis": "X",
                            "note": "URDF does not define a semantic forward axis; heading is encoded in the root orientation quaternion",
                        },
                        # Joint Definition
                        "joints": {
                            "order": self._joints,
                            "type": "hinge",
                            "angle_unit": "radians",
                            "limits": self._meta_joints,
                        },
                    },
                    f,
                    indent=4,
                )

            self.report(
                {"INFO"},
                f"Export finished: {self._base_name}.csv; {self._base_name}_meta.json",
            )
        except Exception as e:
            self.report({"ERROR"}, f"File Error: {str(e)}")

        self.cleanup(context)
        return {"FINISHED"}

    def cancel(self, context):
        """Cancel export operation."""
        self.cleanup(context)
        self.report({"WARNING"}, "Export cancelled")
        return {"CANCELLED"}

    def cleanup(self, context):
        """Clean up modal state and restore scene."""
        if "_export_full_quality" in context.scene:
            del context.scene["_export_full_quality"]
        context.scene.frame_set(self._orig_frame)
        context.window_manager.event_timer_remove(self._timer)
        context.window_manager.progress_end()
        context.workspace.status_text_set(None)
        bpy.ops.object.apply_bvh_mapping()  # Restore retargeting state

    def invoke(self, context, event):
        """Open file browser for directory selection."""
        settings = context.scene.bvh_mapping_settings
        if self.default_pose_blend_in_seconds <= 0.0:
            self.default_pose_blend_in_seconds = getattr(
                settings, "export_blend_in_seconds", 0.0
            )
        if self.default_pose_blend_out_seconds <= 0.0:
            self.default_pose_blend_out_seconds = getattr(
                settings, "export_blend_out_seconds", 0.0
            )
        if self.end_pose_hold_seconds <= 0.0:
            self.end_pose_hold_seconds = getattr(
                settings, "export_end_pose_hold_seconds", 0.0
            )
        context.window_manager.fileselect_add(self)
        bpy.ops.object.apply_bvh_mapping()  # Ensure final retarget before export
        return {"RUNNING_MODAL"}
