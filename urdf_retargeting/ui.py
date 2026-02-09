"""
User interface panels and lists for BVH-to-URDF retargeting.

Provides the main settings panel, bone mapping lists, and property drawings
for the Blender addon UI.
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


class PANEL_BVHMapping(Panel):
    """Main panel for BVH-to-URDF retargeting configuration."""

    bl_label = "URDF Robot Retargeting"
    bl_idname = "VIEW3D_PT_urdf_mapping"
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
        """Draw the retargeting panel."""
        layout = self.layout
        s = context.scene
        settings = s.bvh_mapping_settings

        # Rig selection
        layout.prop(s, "urdf_rig_object")
        layout.prop(s, "bvh_rig_object")

        # Retargeting options
        box = layout.box()
        box.label(text="BVH → URDF Options")
        box.prop(settings, "root_scale")
        box.prop(settings, "location_offset")
        box.prop(settings, "rotation_offset")

        # Foot configuration
        if s.bvh_rig_object:
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

        # Foot and movement parameters
        box.prop(settings, "jump_threshold")
        box.prop(settings, "foot_flattening_height")
        box.prop(settings, "foot_flattening_strength")
        box.prop(settings, "bvh_smoothing")
        box.prop(settings, "joint_smoothing")

        # Generate mappings
        layout.operator("object.generate_mapping_list")

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

        # Apply mapping
        layout.operator("object.apply_bvh_mapping")

        # Export section
        layout.separator()
        box = layout.box()
        box.label(text="Export Beyond Mimic", icon="EXPORT")
        box.prop(settings, "target_hz")
        box.operator("object.export_beyond_mimic", text="Export")
