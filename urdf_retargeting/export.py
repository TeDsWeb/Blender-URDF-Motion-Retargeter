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
            if self._eval_frame <= scene.frame_end:
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
                    f"Export Pass 1/2: Frame {self._eval_frame}/{scene.frame_end} | ESC to Cancel"
                )
                context.window_manager.progress_update(progress)
                self._eval_frame += 1
            else:
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

        self._joints = list(urdf.get("urdf_joint_order", []))
        self._eval_frame = scene.frame_start
        self._raw_frames = []
        self._data_rows = []
        self._meta_joints = {}
        self._orig_frame = scene.frame_current

        # Start modal loop — progress tracks frame evaluation
        wm = context.window_manager
        context.workspace.status_text_set(
            f"Exporting frames {self._export_from_frame}..{self._export_to_frame} → {self._base_name} ({settings.target_hz}Hz)"
        )
        wm.progress_begin(0, self._total_frames)
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

    def finish(self, context):
        """Resample collected data to target Hz, then write CSV and JSON."""
        scene = context.scene
        settings = scene.bvh_mapping_settings
        urdf = scene.urdf_rig_object

        # Pass 2: resample sequential per-frame data to target Hz
        context.workspace.status_text_set("Export Pass 2/2: Resampling…")
        self._data_rows = self._resample_to_target_hz(scene)

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
                        "duration_secs": self._duration,
                        "total_samples": len(self._data_rows),
                        # Export Settings
                        "bvh_smoothing": settings.bvh_smoothing,
                        "joint_smoothing": settings.joint_smoothing,
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
        context.scene.frame_set(self._orig_frame)
        context.window_manager.event_timer_remove(self._timer)
        context.window_manager.progress_end()
        context.workspace.status_text_set(None)
        bpy.ops.object.apply_bvh_mapping()  # Restore retargeting state

    def invoke(self, context, event):
        """Open file browser for directory selection."""
        context.window_manager.fileselect_add(self)
        bpy.ops.object.apply_bvh_mapping()  # Ensure final retarget before export
        return {"RUNNING_MODAL"}
