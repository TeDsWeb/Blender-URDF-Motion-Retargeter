"""
User interface panels and lists for BVH-to-URDF retargeting.

Provides multiple organized panels and lists for bone mapping configuration,
motion options, and export parameters.
"""

from bpy.types import Panel, UIList


class UL_BVHMappingList(UIList):
    """List of BVH-to-URDF bone mappings."""

    def draw_item(
        self,
        context,
        layout,
        data,
        item,
        icon,
        active_data,
        active_propname,
        index,
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
        self,
        context,
        layout,
        data,
        item,
        icon,
        active_data,
        active_propname,
        index,
    ):
        """Draw a single URDF bone mapping item."""
        urdf_obj = context.scene.urdf_rig_object
        row = layout.row(align=True)

        # Show error icon if velocity limit was exceeded
        if urdf_obj and item.urdf_bone_name in urdf_obj.pose.bones:
            ub = urdf_obj.pose.bones[item.urdf_bone_name]
            if ub.get("is_velocity_limited"):
                row.label(text="", icon="ERROR")
                row.alert = True

        # URDF bone search property
        if urdf_obj:
            row.prop_search(
                item,
                "urdf_bone_name",
                urdf_obj.pose,
                "bones",
                text="",
            )

        # Configuration properties
        row.prop(item, "source_axis", text="")
        row.prop(item, "sign", text="")
        row.prop(item, "invert_alignment", text="")
        row.prop(item, "neutral_offset", text="")


class UL_DefaultPoseJointList(UIList):
    """List of editable default-pose joint angles."""

    def draw_item(
        self,
        context,
        layout,
        data,
        item,
        icon,
        active_data,
        active_propname,
        index,
    ):
        """Draw a single default-pose joint angle row."""
        row = layout.row(align=True)
        row.label(text=item.joint_name or "<joint>")
        row.prop(item, "angle", text="")


class PANEL_RigSelection(Panel):
    """Panel for selecting URDF and BVH rigs."""

    bl_label = "Rig Selection"
    bl_idname = "VIEW3D_PT_urdf_rig_selection"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BVH → URDF Retargeting"

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
    bl_category = "BVH → URDF Retargeting"

    def draw(self, context):
        """Draw motion options panel."""
        layout = self.layout
        settings = context.scene.bvh_mapping_settings

        # Root motion control
        box = layout.box()
        box.label(text="Root Motion", icon="ARMATURE_DATA")
        box.prop(settings, "root_scale", slider=True)
        box.prop(settings, "location_offset")
        box.prop(settings, "rotation_offset")

        # Motion smoothing
        box = layout.box()
        box.label(text="Smoothing", icon="MOD_SMOOTH")
        box.prop(settings, "bvh_smoothing", slider=True)
        box.prop(settings, "joint_smoothing", slider=True)
        box.prop(settings, "max_jump_threshold", slider=True)


class PANEL_FootConfiguration(Panel):
    """Panel for foot configuration and ground contact parameters."""

    bl_label = "Foot Configuration"
    bl_idname = "VIEW3D_PT_urdf_foot_config"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BVH → URDF Retargeting"

    def draw(self, context):
        """Draw foot configuration panel."""
        layout = self.layout
        s = context.scene
        settings = s.bvh_mapping_settings

        if not s.bvh_rig_object:
            layout.label(text="Select BVH Rig first", icon="INFO")
            return

        # Foot bone selection
        box = layout.box()
        box.label(text="Foot Bones", icon="BONE_DATA")
        box.prop_search(
            settings,
            "foot_l_name",
            s.bvh_rig_object.pose,
            "bones",
            text="Left Foot",
        )
        box.prop_search(
            settings,
            "foot_r_name",
            s.bvh_rig_object.pose,
            "bones",
            text="Right Foot",
        )

        # Foot contact parameters
        box = layout.box()
        box.label(text="Foot Contact", icon="SNAP_FACE_CENTER")
        box.prop(settings, "jump_threshold", slider=True)
        box.prop(settings, "foot_flattening_height", slider=True)
        box.prop(settings, "foot_flattening_strength", slider=True)
        box.prop(settings, "correction_decay", slider=True)


class PANEL_BoneMapping(Panel):
    """Panel for BVH-to-URDF bone mapping management."""

    bl_label = "Bone Mapping"
    bl_idname = "VIEW3D_PT_urdf_bone_mapping"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BVH → URDF Retargeting"

    def check_duplicate_mappings(self, context):
        """
        Check for duplicate URDF bone assignments.

        Returns:
            Tuple of (duplicate_bones, bones_by_usage) where:
            - duplicate_bones: List of bone names assigned multiple times
            - bones_by_usage: Dict mapping bone names to their assignments
        """
        settings = context.scene.bvh_mapping_settings
        used_urdf_bones = {}
        duplicates = []

        for item in settings.mappings:
            for u_bone in item.urdf_bones:
                name = u_bone.urdf_bone_name
                if name:
                    if name in used_urdf_bones:
                        used_urdf_bones[name].append(item.bvh_bone_name)
                        if name not in duplicates:
                            duplicates.append(name)
                    else:
                        used_urdf_bones[name] = [item.bvh_bone_name]

        return duplicates, used_urdf_bones

    def draw(self, context):
        """Draw bone mapping panel."""
        layout = self.layout
        settings = context.scene.bvh_mapping_settings

        # Generate mappings button
        layout.operator("object.generate_mapping_list", text="Generate Mapping List")

        if not settings.mappings:
            layout.label(
                text="Click 'Generate Mapping List' to show bones.", icon="INFO"
            )
            return

        # Mapping list
        layout.template_list(
            "UL_BVHMappingList",
            "",
            settings,
            "mappings",
            settings,
            "active_mapping_index",
        )

        # URDF bone details
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

        # Duplicate warning
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


class PANEL_ApplyAndExport(Panel):
    """Panel for applying mappings and exporting data."""

    bl_label = "Actions"
    bl_idname = "VIEW3D_PT_urdf_actions"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BVH → URDF Retargeting"

    def draw(self, context):
        """Draw actions panel."""
        layout = self.layout
        settings = context.scene.bvh_mapping_settings

        # Apply mapping
        box = layout.box()
        box.label(text="Retargeting", icon="PLAY")
        box.operator("object.apply_bvh_mapping", text="Apply Mapping")

        # Export section
        box = layout.box()
        box.label(text="Export", icon="EXPORT")
        box.prop(settings, "target_hz", text="Export Hz")

        # Export frame range controls (auto-fill to scene range when BVH rig present)
        row = box.row(align=True)
        row.prop(settings, "export_from_frame", text="From")
        row.prop(settings, "export_to_frame", text="To")
        row = box.row(align=True)
        row.prop(settings, "export_blend_in_seconds", text="Blend In (s)")
        row.prop(settings, "export_blend_out_seconds", text="Blend Out (s)")
        box.prop(settings, "export_end_pose_hold_seconds", text="End Pose Hold (s)")
        box.prop(settings, "use_custom_default_pose", text="Use Custom Default Pose")

        if settings.use_custom_default_pose:
            row = box.row(align=True)
            row.prop(
                settings,
                "default_pose_root_rotation",
                index=0,
                text="Default Root Roll",
            )
            row.prop(
                settings,
                "default_pose_root_rotation",
                index=1,
                text="Default Root Pitch",
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
                text="Reset Default Pose",
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

        op = box.operator("object.export_beyond_mimic", text="Export BeyondMimic")
        # pass current UI settings as defaults to the operator so file-browser shows them
        op.export_from_frame = settings.export_from_frame
        op.export_to_frame = settings.export_to_frame
        op.default_pose_blend_in_seconds = settings.export_blend_in_seconds
        op.default_pose_blend_out_seconds = settings.export_blend_out_seconds
        op.end_pose_hold_seconds = settings.export_end_pose_hold_seconds

        # Import section
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

        # Danger: Clear scene
        box = layout.box()
        box.label(text="Danger Zone", icon="ERROR")
        row = box.row()
        row.alert = True
        row.operator(
            "object.clear_retarget_scene",
            text="Clear Scene (Delete Rigs & Clear Caches)",
            icon="TRASH",
        )
