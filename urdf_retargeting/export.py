"""
Export functionality for Beyond Mimic trajectory format.

Handles exporting retargeted robot trajectories to CSV and JSON formats
with resampling to a specified target Hz.
"""

import bpy
import os
import csv
import json
from bpy.types import Operator


class OT_ExportBeyondMimic(Operator):
    """Exports retargeted trajectories to CSV and JSON files with time-based resampling."""

    bl_idname = "object.export_beyond_mimic"
    bl_label = "Export CSV & JSON"
    bl_description = "Select directory; files will be named after the Source BVH rig and resampled to Target Hz"

    directory: bpy.props.StringProperty(name="Export Directory", subtype="DIR_PATH")

    # Internal Modal State
    _timer = None
    _num_steps = 0
    _current_step = 0
    _time_per_step = 0
    _duration = 0
    _total_frames = 0
    _data_rows = []
    _meta_joints = {}
    _csv_path = ""
    _json_path = ""
    _orig_frame = 0
    _joints = []
    _base_name = ""

    def modal(self, context, event):
        """Modal handler for progressive export."""
        scene = context.scene
        urdf = scene.urdf_rig_object
        fps = scene.render.fps

        if event.type == "ESC":
            return self.cancel(context)

        if event.type == "TIMER":
            if self._current_step < self._num_steps:
                # Calculate current time and map back to Blender frame
                current_time_secs = self._current_step * self._time_per_step
                blender_frame = scene.frame_start + (current_time_secs * fps)

                # Set frame with subframe for smooth interpolation
                scene.frame_set(int(blender_frame), subframe=blender_frame % 1.0)

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

                    # Store metadata only once
                    if self._current_step == 0:
                        self._meta_joints[j] = {
                            "lower": pb.get("limit_lower", -3.14),
                            "upper": pb.get("limit_upper", 3.14),
                        }

                self._data_rows.append(row)

                # UI update
                context.workspace.status_text_set(
                    f"Export Beyond Mimic: Step {self._current_step}/{self._num_steps} | ESC to Cancel"
                )
                context.window_manager.progress_update(self._current_step)
                self._current_step += 1
            else:
                return self.finish(context)

        return {"RUNNING_MODAL"}

    def execute(self, context):
        """Initialize export parameters and start modal loop."""
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

        # Setup resampling
        fps = scene.render.fps
        self._total_frames = scene.frame_end - scene.frame_start
        self._duration = self._total_frames / fps
        target_hz = settings.target_hz
        self._num_steps = int(self._duration * target_hz) + 1
        self._time_per_step = 1.0 / target_hz

        self._joints = list(urdf.get("urdf_joint_order", []))
        self._current_step = 0
        self._data_rows = []
        self._meta_joints = {}
        self._orig_frame = scene.frame_current

        # Start modal loop
        wm = context.window_manager
        wm.progress_begin(0, self._num_steps)
        self._timer = wm.event_timer_add(0.001, window=context.window)
        wm.modal_handler_add(self)

        return {"RUNNING_MODAL"}

    def finish(self, context):
        """Write CSV and JSON files and finalize export."""
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
