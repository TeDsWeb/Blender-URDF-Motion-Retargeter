"""
User interface panels and lists for BVH-to-URDF retargeting.

Provides multiple organized panels and lists for bone mapping configuration,
motion options, and export parameters.
"""

from bpy.types import Panel, UIList


def _draw_advanced_toggle(layout, data, prop_name, label="Advanced"):
    """Draw a compact disclosure-style toggle for optional advanced settings."""
    is_open = getattr(data, prop_name)
    row = layout.row(align=True)
    row.prop(
        data,
        prop_name,
        text=label,
        emboss=False,
        icon="TRIA_DOWN" if is_open else "TRIA_RIGHT",
    )
    return is_open


class UL_BVHMappingList(UIList):
    """List of BVH-to-URDF bone mappings."""

    def draw_item(
        self, context, layout, data, item, icon, active_data, active_propname, index
    ):
        """Draw a single mapping item in the list."""
        row = layout.row(align=True)
        row.label(text=f"BVH: {item.bvh_bone_name}")
        count = len(item.urdf_bones)
        if count > 0:
            row.label(text=f"URDF: {count}")
        else:
            row.enabled = False
            row.label(text=f"URDF: {count}")


class UL_URDFBoneList(UIList):
    """List of URDF bones controlled by a BVH bone."""

    def draw_item(
        self, context, layout, data, item, icon, active_data, active_propname, index
    ):
        """Draw a single URDF bone mapping item."""
        urdf_obj = context.scene.urdf_rig_object
        row = layout.row(align=True)
        if urdf_obj and item.urdf_bone_name in urdf_obj.pose.bones:
            ub = urdf_obj.pose.bones[item.urdf_bone_name]
            if ub.get("is_velocity_limited"):
                row.label(text="", icon="ERROR")
                row.alert = True
        if urdf_obj:
            row.prop_search(item, "urdf_bone_name", urdf_obj.pose, "bones", text="")
        row.prop(item, "source_axis", text="")
        row.prop(item, "sign", text="")
        row.prop(item, "invert_alignment", text="")
        row.prop(item, "neutral_offset", text="")


class UL_DefaultPoseJointList(UIList):
    """List of editable default-pose joint angles."""

    def draw_item(
        self, context, layout, data, item, icon, active_data, active_propname, index
    ):
        """Draw a single default-pose joint angle row."""
        row = layout.row(align=True)
        row.label(text=item.joint_name or "<joint>")
        row.prop(item, "angle", text="")


class UL_KinematicChainList(UIList):
    """List of configured kinematic target chains."""

    def draw_item(
        self, context, layout, data, item, icon, active_data, active_propname, index
    ):
        """Draw a single kinematic chain row."""
        row = layout.row(align=True)
        label = item.label or item.bvh_target_bone_name or f"Chain {index + 1}"
        row.label(text=label, icon="CON_KINEMATIC")
        if item.urdf_root_bone_name and item.urdf_end_bone_name:
            row.label(text=f"{item.urdf_root_bone_name} -> {item.urdf_end_bone_name}")
        else:
            row.label(text="Incomplete", icon="ERROR")


class PANEL_RigSelection(Panel):
    """Panel for selecting URDF and BVH rigs."""

    bl_label = "Rig Selection"
    bl_idname = "VIEW3D_PT_urdf_rig_selection"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BVH -> URDF Retargeting"

    def draw(self, context):
        """Draw rig selection panel."""
        layout = self.layout
        s = context.scene
        layout.prop(s, "urdf_rig_object", text="URDF Rig")
        layout.prop(s, "bvh_rig_object", text="BVH Rig")


class PANEL_MotionOptions(Panel):
    """Panel for root motion and smoothing options."""

    bl_label = "Motion Options"
    bl_idname = "VIEW3D_PT_urdf_motion_options"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BVH -> URDF Retargeting"

    def draw(self, context):
        """Draw motion options panel."""
        layout = self.layout
        settings = context.scene.bvh_mapping_settings

        # Keep old scene files compatible but expose one production path in UI.
        if settings.retargeting_method != "HYBRID":
            settings.retargeting_method = "HYBRID"
        layout.label(text="Retargeting: Hybrid FK + IK", icon="INFO")

        # Root motion control
        box = layout.box()
        box.label(text="Root Motion", icon="ARMATURE_DATA")
        box.prop(settings, "root_scale", slider=True)
        if _draw_advanced_toggle(box, settings, "ui_show_root_advanced"):
            box.prop(settings, "location_offset")
            box.prop(settings, "rotation_offset")

        # BVH source smoothing (always visible)
        box = layout.box()
        box.label(text="Smoothing", icon="MOD_SMOOTH")
        box.prop(settings, "bvh_smoothing", slider=True)

        # --- HYBRID: FK Baseline ---
        fk_box = layout.box()
        fk_box.label(text="FK Baseline", icon="BONE_DATA")
        fk_box.prop(settings, "joint_smoothing", slider=True)
        if _draw_advanced_toggle(fk_box, settings, "ui_show_fk_advanced"):
            fk_box.prop(settings, "max_jump_threshold", slider=True)

        # --- HYBRID: IK Correction ---
        ik_box = layout.box()
        ik_box.label(text="IK Correction", icon="CON_KINEMATIC")
        ik_box.prop(settings, "hybrid_ik_blend", slider=True)
        ik_box.prop(settings, "ik_target_smoothing", slider=True)
        ik_box.prop(settings, "ik_joint_smoothing", slider=True)
        if _draw_advanced_toggle(ik_box, settings, "ui_show_ik_advanced"):
            ik_box.prop(settings, "hybrid_adaptive_ik")
            if settings.hybrid_adaptive_ik:
                sub = ik_box.column(align=True)
                sub.prop(settings, "hybrid_min_ik_blend", slider=True)
                sub.prop(settings, "hybrid_error_low")
                sub.prop(settings, "hybrid_error_high")
                sub.prop(settings, "hybrid_blend_smoothing", slider=True)
            ik_box.separator()
            ik_box.prop(settings, "ik_micro_deadzone", slider=True)
            ik_box.prop(settings, "ik_iterations")
            ik_box.prop(settings, "ik_tolerance")
            ik_box.prop(settings, "ik_max_step_angle", slider=True)
            ik_box.prop(settings, "ik_target_scale", slider=True)
            ik_box.prop(settings, "ik_proportion_blend", slider=True)
            ik_box.prop(settings, "ik_ground_lock_strength", slider=True)

        # --- HYBRID: Performance ---
        perf_box = layout.box()
        perf_box.label(text="Performance", icon="SORTTIME")
        perf_box.prop(settings, "hybrid_realtime_guard")
        if _draw_advanced_toggle(
            perf_box,
            settings,
            "ui_show_performance_advanced",
        ):
            if settings.hybrid_realtime_guard:
                perf_box.prop(settings, "hybrid_min_iterations")
            perf_box.prop(settings, "hybrid_ik_frame_skip")
            perf_box.prop(settings, "quality_ik_iterations")
            perf_box.separator()
            perf_box.prop(settings, "bake_watchdog_enabled")
            if settings.bake_watchdog_enabled:
                perf_box.prop(settings, "bake_watchdog_frame_ms")
                perf_box.prop(settings, "bake_watchdog_abort")

        # --- Telemetry ---
        avg_ms = context.scene.get("_retarget_ms_ema", None)
        if avg_ms is not None:
            perf_box.separator()
            perf_box.label(text=f"Retarget Avg: {avg_ms:.2f} ms/frame", icon="TIME")
        ik_ms = context.scene.get("_retarget_ms_ik", None)
        if ik_ms is not None:
            perf_box.label(text=f"IK Stage: {ik_ms:.2f} ms", icon="CON_KINEMATIC")
        fk_ms = context.scene.get("_retarget_ms_fk", None)
        if fk_ms is not None:
            perf_box.label(text=f"FK Stage: {fk_ms:.2f} ms", icon="BONE_DATA")
        sink_ms = context.scene.get("_retarget_ms_sink", None)
        if sink_ms is not None:
            perf_box.label(text=f"Anti-Sink: {sink_ms:.2f} ms", icon="EMPTY_ARROWS")


class PANEL_FootConfiguration(Panel):
    """Panel for foot configuration and ground contact parameters."""

    bl_label = "Foot Configuration"
    bl_idname = "VIEW3D_PT_urdf_foot_config"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BVH -> URDF Retargeting"

    def draw(self, context):
        """Draw foot configuration panel."""
        layout = self.layout
        s = context.scene
        settings = s.bvh_mapping_settings

        if not s.bvh_rig_object:
            layout.label(text="Select BVH Rig first", icon="INFO")
            return

        box = layout.box()
        box.label(text="Foot Bones", icon="BONE_DATA")
        box.prop_search(
            settings, "foot_l_name", s.bvh_rig_object.pose, "bones", text="Left Foot"
        )
        box.prop_search(
            settings, "foot_r_name", s.bvh_rig_object.pose, "bones", text="Right Foot"
        )

        box = layout.box()
        box.label(text="Foot Contact", icon="SNAP_FACE_CENTER")
        box.prop(settings, "jump_threshold", slider=True)
        box.prop(settings, "foot_flattening_strength", slider=True)
        if _draw_advanced_toggle(box, settings, "ui_show_foot_advanced"):
            box.prop(settings, "foot_contact_hysteresis", slider=True)
            box.prop(settings, "foot_flattening_height", slider=True)
            box.prop(settings, "foot_pin_xy_max_step", slider=True)
            box.prop(settings, "foot_pin_yaw_max_step", slider=True)
            box.prop(settings, "correction_decay", slider=True)


class PANEL_BoneMapping(Panel):
    """Panel for BVH-to-URDF bone mapping management."""

    bl_label = "Bone Mapping"
    bl_idname = "VIEW3D_PT_urdf_bone_mapping"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BVH -> URDF Retargeting"

    def check_duplicate_mappings(self, context):
        """Check for duplicate URDF bone assignments within bone mappings only."""
        settings = context.scene.bvh_mapping_settings
        used_urdf_bones = {}
        duplicates = []

        def _register_assignment(name, source_name):
            if not name:
                return
            if name in used_urdf_bones:
                used_urdf_bones[name].append(source_name)
                if name not in duplicates:
                    duplicates.append(name)
            else:
                used_urdf_bones[name] = [source_name]

        for item in settings.mappings:
            for u_bone in item.urdf_bones:
                _register_assignment(u_bone.urdf_bone_name, item.bvh_bone_name)

        return duplicates, used_urdf_bones

    def draw(self, context):
        """Draw bone mapping panel."""
        layout = self.layout
        settings = context.scene.bvh_mapping_settings

        layout.operator("object.generate_mapping_list", text="Generate Mapping List")

        if not settings.mappings:
            layout.label(
                text="Click 'Generate Mapping List' to show bones.", icon="INFO"
            )
            return

        layout.template_list(
            "UL_BVHMappingList",
            "",
            settings,
            "mappings",
            settings,
            "active_mapping_index",
        )

        if 0 <= settings.active_mapping_index < len(settings.mappings):
            active = settings.mappings[settings.active_mapping_index]
            box = layout.box()
            row = box.row()
            row.template_list(
                "UL_URDFBoneList",
                "",
                active,
                "urdf_bones",
                settings,
                "active_urdf_index",
            )
            col = row.column(align=True)
            col.operator("object.add_urdf_bone", text="", icon="ADD").bvh_bone_name = (
                active.bvh_bone_name
            )
            col.operator(
                "object.remove_urdf_bone", text="", icon="REMOVE"
            ).bvh_bone_name = active.bvh_bone_name

        duplicates, _ = self.check_duplicate_mappings(context)
        if duplicates:
            box = layout.box()
            box.alert = True
            col = box.column()
            col.label(text="WARNING: Multiple assignments!", icon="ERROR")
            for d in duplicates:
                col.label(
                    text=f"Joint '{d}' is controlled multiple times", icon="BONE_DATA"
                )


class PANEL_KinematicChains(Panel):
    """Panel for configuring BVH target to URDF IK chains."""

    bl_label = "Kinematic Chains"
    bl_idname = "VIEW3D_PT_urdf_kinematic_chains"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BVH -> URDF Retargeting"

    def draw(self, context):
        """Draw kinematic chain configuration panel."""
        layout = self.layout
        scene = context.scene
        settings = scene.bvh_mapping_settings

        row = layout.row()
        row.template_list(
            "UL_KinematicChainList",
            "",
            settings,
            "kinematic_chains",
            settings,
            "active_kinematic_chain_index",
        )
        col = row.column(align=True)
        col.operator("object.add_kinematic_chain", text="", icon="ADD")
        col.operator("object.remove_kinematic_chain", text="", icon="REMOVE")

        if 0 <= settings.active_kinematic_chain_index < len(settings.kinematic_chains):
            chain = settings.kinematic_chains[settings.active_kinematic_chain_index]
            box = layout.box()
            box.prop(chain, "label")

            if scene.bvh_rig_object:
                box.prop_search(
                    chain,
                    "bvh_target_bone_name",
                    scene.bvh_rig_object.pose,
                    "bones",
                    text="BVH Target",
                )
            else:
                box.prop(chain, "bvh_target_bone_name")

            if scene.urdf_rig_object:
                box.prop_search(
                    chain,
                    "urdf_root_bone_name",
                    scene.urdf_rig_object.pose,
                    "bones",
                    text="URDF Root",
                )
                box.prop_search(
                    chain,
                    "urdf_end_bone_name",
                    scene.urdf_rig_object.pose,
                    "bones",
                    text="URDF End",
                )
            else:
                box.prop(chain, "urdf_root_bone_name")
                box.prop(chain, "urdf_end_bone_name")

            box.prop(chain, "influence", slider=True)
            box.separator()
            box.label(text="Hybrid Adaptive Override", icon="CON_SPLINEIK")
            box.prop(chain, "use_hybrid_adaptive_override")
            if chain.use_hybrid_adaptive_override:
                box.prop(chain, "hybrid_min_ik_blend", slider=True)
                box.prop(chain, "hybrid_error_low")
                box.prop(chain, "hybrid_error_high")
            layout.label(
                text="Define chains from a URDF parent joint to an end-effector joint.",
                icon="BONE_DATA",
            )


class PANEL_DefaultPose(Panel):
    """Panel for configuring the stored neutral pose."""

    bl_label = "Neutral Pose"
    bl_idname = "VIEW3D_PT_urdf_default_pose"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BVH -> URDF Retargeting"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        """Draw neutral pose configuration panel."""
        layout = self.layout
        settings = context.scene.bvh_mapping_settings

        box = layout.box()
        box.label(
            text="Used for export blend phases and as the retarget base pose.",
            icon="INFO",
        )
        row = box.row(align=True)
        row.prop(
            settings,
            "default_pose_root_rotation",
            index=0,
            text="Neutral Root Roll",
        )
        row.prop(
            settings,
            "default_pose_root_rotation",
            index=1,
            text="Neutral Root Pitch",
        )
        row = box.row(align=True)
        row.operator(
            "object.capture_default_pose_from_current",
            text="Capture Current Pose",
            icon="IMPORT",
        )
        row.operator(
            "object.sync_default_pose_joints",
            text="Sync Joint List",
            icon="FILE_REFRESH",
        )
        row.operator(
            "object.reset_default_pose",
            text="Reset Neutral Pose",
            icon="LOOP_BACK",
        )
        box.template_list(
            "UL_DefaultPoseJointList",
            "",
            settings,
            "default_pose_joints",
            settings,
            "default_pose_active_index",
            rows=5,
        )


class PANEL_ApplyAndExport(Panel):
    """Panel for applying mappings and exporting data."""

    bl_label = "Actions"
    bl_idname = "VIEW3D_PT_urdf_actions"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BVH -> URDF Retargeting"

    def draw(self, context):
        """Draw actions panel."""
        layout = self.layout
        settings = context.scene.bvh_mapping_settings

        box = layout.box()
        box.label(text="Retargeting", icon="PLAY")
        row = box.row(align=True)
        row.operator("object.apply_bvh_mapping", text="Apply Mapping")
        row.operator("object.bake_bvh_mapping", text="Bake Retarget", icon="REC")
        box.prop(settings, "precompute_on_apply")
        if settings.precompute_on_apply:
            box.prop(settings, "precompute_disable_live_after_apply")

        box = layout.box()
        box.label(text="Export", icon="EXPORT")
        box.prop(settings, "target_hz", text="Export Hz")
        row = box.row(align=True)
        row.prop(settings, "export_from_frame", text="From")
        row.prop(settings, "export_to_frame", text="To")
        row = box.row(align=True)
        row.prop(settings, "export_blend_in_seconds", text="Blend In (s)")
        row.prop(settings, "export_blend_out_seconds", text="Blend Out (s)")
        box.prop(settings, "export_end_pose_hold_seconds", text="End Pose Hold (s)")

        op = box.operator("object.export_beyond_mimic", text="Export BeyondMimic")
        op.export_from_frame = settings.export_from_frame
        op.export_to_frame = settings.export_to_frame
        op.default_pose_blend_in_seconds = settings.export_blend_in_seconds
        op.default_pose_blend_out_seconds = settings.export_blend_out_seconds
        op.end_pose_hold_seconds = settings.export_end_pose_hold_seconds

        box = layout.box()
        box.label(text="Mapping Preset (JSON)", icon="FILE_FOLDER")
        box.prop(settings, "mapping_preset_name")
        box.prop(settings, "mapping_robot_profile")
        box.prop(settings, "mapping_mocap_profile")
        box.prop(settings, "mapping_library_preset")
        row = box.row(align=True)
        row.operator(
            "object.load_mapping_preset_library",
            text="Load Library Preset",
            icon="FILEBROWSER",
        )
        row.operator(
            "object.save_mapping_preset_library",
            text="Save To Library",
            icon="FILE_TICK",
        )
        row = box.row(align=True)
        row.operator(
            "object.export_mapping_json", text="Export Mapping (JSON)", icon="EXPORT"
        )
        row.operator(
            "object.import_mapping_json", text="Import Mapping (JSON)", icon="IMPORT"
        )

        from .mapping_io import count_non_quaternion_bones

        non_quat_bvh = count_non_quaternion_bones(context.scene.bvh_rig_object)
        non_quat_urdf = count_non_quaternion_bones(context.scene.urdf_rig_object)
        if non_quat_bvh > 0 or non_quat_urdf > 0:
            warn = layout.box()
            warn.alert = True
            warn.label(
                text=(
                    "Warning: Non-quaternion bones detected "
                    f"(BVH: {non_quat_bvh}, URDF: {non_quat_urdf})"
                ),
                icon="ERROR",
            )
            warn.label(
                text="Please convert rotation mode in Blender if needed.", icon="INFO"
            )

        box = layout.box()
        box.label(text="Import", icon="IMPORT")
        box.prop(settings, "import_use_meta_hz", text="Use Meta Hz")
        row = box.row(align=True)
        row.prop(settings, "import_manual_hz", text="Hz")
        row.enabled = not settings.import_use_meta_hz
        box.prop(settings, "import_set_scene_fps", text="Set Scene FPS")
        op = box.operator("object.import_beyond_mimic", text="Import BeyondMimic")
        op.use_meta_hz = settings.import_use_meta_hz
        op.manual_hz = settings.import_manual_hz
        op.set_scene_fps = settings.import_set_scene_fps

        box = layout.box()
        box.label(text="Danger Zone", icon="ERROR")
        row = box.row()
        row.alert = True
        row.operator(
            "object.clear_retarget_scene",
            text="Clear Scene (Delete Rigs & Clear Caches)",
            icon="TRASH",
        )
