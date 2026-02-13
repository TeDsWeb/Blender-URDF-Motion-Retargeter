"""
Import functionality for Beyond Mimic CSV trajectory format.

Handles importing retargeted robot trajectories from CSV and applying
animation data to the URDF rig with optional FPS handling.
"""

import bpy
import os
import csv
import json
import math
import mathutils
from typing import Optional
from bpy.types import Operator
from bpy_extras.io_utils import ImportHelper


class OT_ImportBeyondMimic(Operator, ImportHelper):
    """Import BeyondMimic CSV and apply it to the URDF rig."""

    bl_idname = "object.import_beyond_mimic"
    bl_label = "Import CSV"
    bl_description = "Import BeyondMimic CSV and apply animation to the URDF rig"

    filename_ext = ".csv"
    filter_glob: bpy.props.StringProperty(default="*.csv", options={"HIDDEN"})
    start_frame: bpy.props.IntProperty(
        name="Start Frame",
        description="Frame to start inserting keyframes",
        default=1,
        min=0,
    )
    use_meta_hz: bpy.props.BoolProperty(
        name="Use Meta Hz",
        description="If a matching _meta.json exists, use its export_hz",
        default=True,
    )
    manual_hz: bpy.props.FloatProperty(
        name="Hz",
        description="Override the sample rate (Hz) when not using meta",
        default=0.0,
        min=0.0,
    )
    set_scene_fps: bpy.props.BoolProperty(
        name="Set Scene FPS",
        description="Set scene FPS to the import sample rate",
        default=False,
    )
    update_scene_range: bpy.props.BoolProperty(
        name="Update Scene Range",
        description="Extend scene end frame to cover the imported animation",
        default=True,
    )

    # Internal Modal State
    _timer = None
    _data_rows = None
    _joint_names = None
    _frame = 0.0
    _frame_step = 1.0
    _row_index = 0
    _missing_joints = None
    _bad_rows = 0
    _sample_hz = 0.0

    def _read_meta_hz(self, csv_path: str) -> Optional[float]:
        meta_path = os.path.splitext(csv_path)[0] + "_meta.json"
        if not os.path.exists(meta_path):
            return None
        try:
            with open(meta_path, "r") as f:
                meta = json.load(f)
            hz = meta.get("export_hz", None)
            return float(hz) if hz is not None else None
        except Exception:
            return None

    def _to_float(self, value) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    def modal(self, context, event):
        scene = context.scene
        urdf = scene.urdf_rig_object

        if event.type == "ESC":
            return self.cancel(context)

        if event.type == "TIMER":
            if self._row_index >= len(self._data_rows):
                return self.finish(context)

            row = self._data_rows[self._row_index]
            if len(row) < 7:
                self._bad_rows += 1
            else:
                root_tx = self._to_float(row[0])
                root_ty = self._to_float(row[1])
                root_tz = self._to_float(row[2])
                root_qx = self._to_float(row[3])
                root_qy = self._to_float(row[4])
                root_qz = self._to_float(row[5])
                root_qw = self._to_float(row[6])

                urdf.location = mathutils.Vector((root_tx, root_ty, root_tz))
                urdf.rotation_mode = "QUATERNION"
                urdf.rotation_quaternion = mathutils.Quaternion(
                    (root_qw, root_qx, root_qy, root_qz)
                )

                for j_idx, j_name in enumerate(self._joint_names):
                    pb = urdf.pose.bones.get(j_name)
                    if not pb:
                        self._missing_joints.add(j_name)
                        continue

                    col_idx = 7 + j_idx
                    angle = self._to_float(row[col_idx]) if col_idx < len(row) else 0.0
                    pb.rotation_mode = "QUATERNION"
                    pb.rotation_quaternion = mathutils.Quaternion((0, 1, 0), angle)
                    pb["_joint_angle"] = angle
                    pb.keyframe_insert(
                        data_path="rotation_quaternion", frame=self._frame
                    )

                urdf.keyframe_insert(data_path="location", frame=self._frame)
                urdf.keyframe_insert(data_path="rotation_quaternion", frame=self._frame)

                self._frame += self._frame_step

            self._row_index += 1

            context.workspace.status_text_set(
                f"Import: Sample {self._row_index}/{len(self._data_rows)} | ESC to Cancel"
            )
            context.window_manager.progress_update(self._row_index)

        return {"RUNNING_MODAL"}

    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings
        urdf = scene.urdf_rig_object

        if not urdf:
            self.report({"ERROR"}, "URDF rig is missing")
            return {"CANCELLED"}
        if not self.filepath:
            self.report({"ERROR"}, "CSV file path is missing")
            return {"CANCELLED"}

        settings.live_retarget = False

        if settings is not None:
            settings.import_use_meta_hz = self.use_meta_hz
            settings.import_manual_hz = self.manual_hz
            settings.import_set_scene_fps = self.set_scene_fps

        sample_hz = None
        if not self.use_meta_hz and self.manual_hz > 0:
            sample_hz = self.manual_hz
        if self.use_meta_hz:
            sample_hz = self._read_meta_hz(self.filepath)
        if not sample_hz or sample_hz <= 0:
            sample_hz = scene.render.fps if scene.render.fps > 0 else 60.0
        if self.set_scene_fps and sample_hz > 0:
            scene.render.fps = int(round(sample_hz))

        try:
            with open(self.filepath, "r", newline="") as f:
                reader = csv.reader(f)
                rows = [row for row in reader if row]
        except Exception as e:
            self.report({"ERROR"}, f"CSV read error: {str(e)}")
            return {"CANCELLED"}

        if not rows:
            self.report({"ERROR"}, "CSV file is empty")
            return {"CANCELLED"}

        header = [h.strip().lower() for h in rows[0]]
        has_header = len(header) >= 7 and header[0] == "root_tx"

        if has_header:
            joint_names = rows[0][7:]
            data_rows = rows[1:]
        else:
            joint_names = list(urdf.get("urdf_joint_order", []))
            data_rows = rows

        if not data_rows:
            self.report({"ERROR"}, "No data rows found in CSV")
            return {"CANCELLED"}

        action_name = f"{bpy.path.clean_name(urdf.name)}_csv"
        action = bpy.data.actions.new(name=action_name)
        if not urdf.animation_data:
            urdf.animation_data_create()
        urdf.animation_data.action = action

        self._sample_hz = sample_hz
        self._data_rows = data_rows
        self._joint_names = joint_names
        self._frame_step = (scene.render.fps / sample_hz) if sample_hz > 0 else 1.0
        self._frame = float(self.start_frame)
        self._row_index = 0
        self._missing_joints = set()
        self._bad_rows = 0

        wm = context.window_manager
        context.workspace.status_text_set(
            f"Importing {len(self._data_rows)} samples → {bpy.path.clean_name(urdf.name)}"
        )
        wm.progress_begin(0, len(self._data_rows))
        self._timer = wm.event_timer_add(0.001, window=context.window)
        wm.modal_handler_add(self)

        return {"RUNNING_MODAL"}

    def finish(self, context):
        scene = context.scene

        if self.update_scene_range:
            end_frame = int(math.ceil(self._frame - self._frame_step))
            if end_frame > scene.frame_end:
                scene.frame_end = end_frame

        if self._bad_rows:
            self.report({"WARNING"}, f"Skipped {self._bad_rows} malformed rows")
        if self._missing_joints:
            self.report(
                {"WARNING"},
                f"Missing {len(self._missing_joints)} joints in URDF rig",
            )

        self.cleanup(context)
        self.report({"INFO"}, f"Import finished: {os.path.basename(self.filepath)}")
        return {"FINISHED"}

    def cancel(self, context):
        self.cleanup(context)
        self.report({"WARNING"}, "Import cancelled")
        return {"CANCELLED"}

    def cleanup(self, context):
        if self._timer is not None:
            context.window_manager.event_timer_remove(self._timer)
        context.window_manager.progress_end()
        context.workspace.status_text_set(None)
