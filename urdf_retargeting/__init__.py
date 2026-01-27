bl_info = {
    "name": "URDF Retargeting",
    "author": "Dominik Brämer",
    "version": (1, 0, 0),
    "blender": (5, 0, 0),
    "location": "File > Import > URDF Humanoid",
    "description": "URDF Import + BVH mapping with joint limits and interactive N-Panel",
    "category": "Import-Export",
    "type": "add-on",
}

import bpy
import os
import math
import mathutils
from bpy_extras.io_utils import ImportHelper
import xml.etree.ElementTree as ET
from bpy.props import (
    StringProperty,
    PointerProperty,
    CollectionProperty,
    IntProperty,
    EnumProperty,
    BoolProperty,
    FloatVectorProperty,
    FloatProperty,
)
from bpy.types import PropertyGroup, Operator, Panel, UIList
from bpy.app.handlers import persistent

# ============================================================
# URDF DATA STRUCTURES & PARSER
# ============================================================


class URDFVisual:
    def __init__(self, xyz, rpy, mesh):
        self.xyz = xyz
        self.rpy = rpy
        self.mesh = mesh


class URDFLink:
    def __init__(self, name):
        self.name = name
        self.visuals = []


class URDFJoint:
    def __init__(
        self,
        name,
        jtype,
        parent,
        child,
        xyz,
        rpy,
        axis,
        limit_lower=None,
        limit_upper=None,
    ):
        self.name = name
        self.type = jtype
        self.parent = parent
        self.child = child
        self.xyz = xyz
        self.rpy = rpy
        self.axis = axis
        self.limit_lower = limit_lower
        self.limit_upper = limit_upper


class URDFRobot:
    def __init__(self, name):
        self.name = name
        self.links = {}
        self.joints = {}


def parse_float_list(s, n):
    """Parses a string of space/comma separated floats."""
    if s is None:
        return [0.0] * n
    parts = s.replace(",", " ").split()
    vals = []
    for p in parts:
        try:
            vals.append(float(p))
        except:
            vals.append(0.0)
    while len(vals) < n:
        vals.append(0.0)
    return vals[:n]


def parse_urdf(path):
    """Parses the URDF XML file into a URDFRobot structure."""
    tree = ET.parse(path)
    root = tree.getroot()
    robot = URDFRobot(root.attrib.get("name", "URDF_Robot"))

    # Parse Links
    for link_el in root.findall("link"):
        link = URDFLink(link_el.attrib["name"])
        for vis_el in link_el.findall("visual"):
            org = vis_el.find("origin")
            xyz = parse_float_list(
                org.attrib.get("xyz") if org is not None else None, 3
            )
            rpy = parse_float_list(
                org.attrib.get("rpy") if org is not None else None, 3
            )
            geom = vis_el.find("geometry")
            mesh_el = geom.find("mesh") if geom is not None else None
            mesh = mesh_el.attrib.get("filename") if mesh_el is not None else None
            if mesh:
                link.visuals.append(URDFVisual(xyz, rpy, mesh))
        robot.links[link.name] = link

    # Parse Joints
    for idx, joint_el in enumerate(root.findall("joint")):
        name = joint_el.attrib.get("name", "joint_%03d" % idx)
        jtype = joint_el.attrib.get("type", "fixed")
        parent = joint_el.find("parent").attrib["link"]
        child = joint_el.find("child").attrib["link"]
        org = joint_el.find("origin")
        xyz = parse_float_list(org.attrib.get("xyz") if org is not None else None, 3)
        rpy = parse_float_list(org.attrib.get("rpy") if org is not None else None, 3)
        axis_el = joint_el.find("axis")
        axis = parse_float_list(
            axis_el.attrib.get("xyz") if axis_el is not None else None, 3
        )

        # New: Extract Joint Limits
        limit_el = joint_el.find("limit")
        lower = (
            float(limit_el.attrib.get("lower", -3.14159))
            if limit_el is not None
            else None
        )
        upper = (
            float(limit_el.attrib.get("upper", 3.14159))
            if limit_el is not None
            else None
        )

        robot.joints[name] = URDFJoint(
            name, jtype, parent, child, xyz, rpy, axis, lower, upper
        )
    return robot


# ============================================================
# UTILITY & BUILDING
# ============================================================


def origin_to_matrix(xyz, rpy):
    """Converts URDF origin (xyz, rpy) to a 4x4 Transformation Matrix."""
    loc = mathutils.Vector(xyz)
    rot = mathutils.Euler(rpy, "XYZ").to_matrix().to_4x4()
    return mathutils.Matrix.Translation(loc) @ rot


def build_link_transforms(robot):
    """Computes world matrices for all links in their default pose."""
    children = {}
    joint_by_child = {}
    for j in robot.joints.values():
        children.setdefault(j.parent, []).append(j)
        joint_by_child[j.child] = j
    all_links = set(l.name for l in robot.links.values())
    roots = list(all_links - set(joint_by_child.keys()))
    root = roots[0] if roots else list(all_links)[0]

    mats = {root: mathutils.Matrix.Identity(4)}
    stack = [root]
    while stack:
        p = stack.pop()
        for j in children.get(p, []):
            mats[j.child] = mats[p] @ origin_to_matrix(j.xyz, j.rpy)
            stack.append(j.child)
    return mats, root


def create_urdf_armature(robot):
    """Creates a Blender armature based on URDF joint structure."""
    link_mats, root = build_link_transforms(robot)
    bpy.ops.object.add(type="ARMATURE", enter_editmode=True)
    arm_obj = bpy.context.object
    arm_obj.name = f"{robot.name}_Rig"
    bones = arm_obj.data.edit_bones

    link_to_bone = {}
    for link_name in link_mats:
        link_to_bone[link_name] = bones.new(link_name)

    for j in robot.joints.values():
        b = link_to_bone.get(j.child)
        if b:
            head = link_mats[j.child].to_translation()
            joint_rot = mathutils.Euler(j.rpy, "XYZ").to_matrix()
            axis_world = (
                link_mats[j.parent].to_3x3() @ joint_rot @ mathutils.Vector(j.axis)
            ).normalized()
            b.head = head
            b.tail = head + axis_world * 0.05
            if j.parent in link_to_bone:
                b.parent = link_to_bone[j.parent]

    bpy.ops.object.mode_set(mode="OBJECT")

    # Store Joint Limits in PoseBones for the engine
    for j_name, j in robot.joints.items():
        if j.child in arm_obj.pose.bones:
            pb = arm_obj.pose.bones[j.child]
            if j.limit_lower is not None:
                pb["limit_lower"] = j.limit_lower
            if j.limit_upper is not None:
                pb["limit_upper"] = j.limit_upper

    return arm_obj, link_mats


def bind_meshes(robot, arm_obj, link_mats, base_path):
    """Imports meshes and parents them to bones with correct offsets."""
    for link_name, link in robot.links.items():
        for idx, vis in enumerate(link.visuals):
            mesh_raw = vis.mesh.replace("package://", "")
            mesh_path = os.path.normpath(os.path.join(base_path, mesh_raw))
            if not os.path.exists(mesh_path):
                continue

            # Import based on file extension
            ext = os.path.splitext(mesh_path)[1].lower()
            if ext == ".stl":
                bpy.ops.wm.stl_import(filepath=mesh_path)
            elif ext == ".obj":
                bpy.ops.wm.obj_import(filepath=mesh_path)
            elif ext == ".dae":
                bpy.ops.wm.collada_import(filepath=mesh_path)

            if bpy.context.selected_objects:
                obj = bpy.context.selected_objects[0]
                obj.name = f"{link_name}_mesh_{idx}"

                # Calculate target world matrix from URDF
                target_mtx = link_mats[link_name] @ origin_to_matrix(vis.xyz, vis.rpy)

                # Parent to Bone (Fixes weird offsets)
                obj.parent = arm_obj
                if link_name in arm_obj.pose.bones:
                    obj.parent_type = "BONE"
                    obj.parent_bone = link_name
                    # Parent inverse ensures the mesh stays in place relative to bone position
                    bone_mtx_world = (
                        arm_obj.matrix_world @ arm_obj.pose.bones[link_name].matrix
                    )
                    obj.matrix_parent_inverse = bone_mtx_world.inverted()

                obj.matrix_world = target_mtx


# ============================================================
# DATA STRUCTURES
# ============================================================


class BVHMappingBone(PropertyGroup):
    bvh_bone_name: StringProperty(name="URDF Bone")
    source_axis: EnumProperty(
        items=[("X", "X", ""), ("Y", "Y", ""), ("Z", "Z", "")], default="X"
    )
    sign: EnumProperty(items=[("POS", "+", ""), ("NEG", "-", "")], default="NEG")
    neutral_offset: FloatProperty(name="Neutral Offset", subtype="ANGLE")


class BVHMappingItem(PropertyGroup):
    urdf_bones: CollectionProperty(type=BVHMappingBone)
    bvh_bone_name: StringProperty()
    ref_rot: FloatVectorProperty(size=4, default=(1.0, 0.0, 0.0, 0.0))


class BVHMappingSettings(PropertyGroup):
    mappings: CollectionProperty(type=BVHMappingItem)
    active_mapping_index: IntProperty()
    active_urdf_index: IntProperty()
    live_retarget: BoolProperty(name="Live Retargeting")
    smoothing: FloatProperty(name="Smoothing", default=0.25, min=0, max=1)
    # Root Motion Settings
    root_scale: FloatProperty(name="Root Scale", default=1.0)
    location_offset: FloatVectorProperty(name="Loc Offset", subtype="TRANSLATION")
    rotation_offset: FloatVectorProperty(name="Rot Offset", subtype="EULER")
    transfer_root_rot: BoolProperty(name="Transfer World Rotation", default=True)


# ============================================================
# ENGINE & UI
# ============================================================


@persistent
def retarget_frame(scene):
    settings = scene.bvh_mapping_settings
    if not settings.live_retarget:
        return
    urdf = scene.urdf_rig_object
    bvh = scene.bvh_rig_object
    if not urdf or not bvh:
        return

    # 1. Root Motion (Object Level)
    if bvh.pose.bones:
        bvh_root = bvh.matrix_world @ bvh.pose.bones[0].matrix
        urdf.location = (
            bvh_root.to_translation() * settings.root_scale
        ) + mathutils.Vector(settings.location_offset)
        if settings.transfer_root_rot:
            off_q = mathutils.Euler(settings.rotation_offset).to_quaternion()
            urdf.rotation_mode = "QUATERNION"
            urdf.rotation_quaternion = bvh_root.to_quaternion() @ off_q

    # 2. Bone Rotations & Neutral Pose Fix
    if "_prev_euler_cache" not in scene:
        scene["_prev_euler_cache"] = {}
    cache = scene["_prev_euler_cache"]

    for item in settings.mappings:
        bvh_b = bvh.pose.bones.get(item.bvh_bone_name)
        if not bvh_b:
            continue

        ref_q = mathutils.Quaternion(item.ref_rot)
        rot_rel = ref_q.inverted() @ bvh_b.matrix_basis.to_quaternion()
        e = rot_rel.to_euler("XYZ")

        key = f"{bvh.name}:{bvh_b.name}"
        if key in cache:
            e.make_compatible(cache[key])
        cache[key] = e.copy()

        for b in item.urdf_bones:
            urdf_b = urdf.pose.bones.get(b.bvh_bone_name)
            if not urdf_b:
                continue

            val = getattr(e, b.source_axis.lower())
            if b.sign == "NEG":
                val = -val

            # --- NEUTRAL POSE FIX ---
            # Store the first frame's value as a base offset
            if "offset" not in urdf_b:
                urdf_b["offset"] = val

            # Resulting angle is the difference from start + manual user correction
            final_angle = val - urdf_b["offset"] + b.neutral_offset

            # --- JOINT LIMITS CLAMPING ---
            l_min = urdf_b.get("limit_lower", -3.14159)
            l_max = urdf_b.get("limit_upper", 3.14159)
            final_angle = max(min(final_angle, l_max), l_min)

            target_q = mathutils.Euler((0, final_angle, 0)).to_quaternion()
            urdf_b.rotation_mode = "QUATERNION"
            urdf_b.rotation_quaternion = urdf_b.rotation_quaternion.slerp(
                target_q, settings.smoothing
            )


# UI Implementation
class UL_BVHMappingList(UIList):
    def draw_item(
        self, context, layout, data, item, icon, active_data, active_propname, index
    ):
        layout.label(text=item.bvh_bone_name)


class UL_URDFBoneList(UIList):
    def draw_item(
        self, context, layout, data, item, icon, active_data, active_propname, index
    ):
        row = layout.row()
        if context.scene.urdf_rig_object:
            row.prop_search(
                item,
                "bvh_bone_name",
                context.scene.urdf_rig_object.pose,
                "bones",
                text="",
            )
        row.prop(item, "source_axis", text="")
        row.prop(item, "sign", text="")
        row.prop(item, "neutral_offset", text="")


class PANEL_BVHMapping(Panel):
    bl_label = "URDF Robot Retargeting"
    bl_idname = "VIEW3D_PT_urdf_mapping"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BVH -> URDF Retargeting"

    def draw(self, context):
        layout = self.layout
        s = context.scene
        settings = s.bvh_mapping_settings
        layout.prop(s, "urdf_rig_object")
        layout.prop(s, "bvh_rig_object")

        box = layout.box()
        box.label(text="BVH -> URDF Root Options")
        box.prop(settings, "root_scale")
        box.prop(settings, "transfer_root_rot")
        box.prop(settings, "location_offset")
        box.prop(settings, "rotation_offset")

        layout.prop(settings, "smoothing")
        layout.prop(settings, "live_retarget", toggle=True)
        layout.operator("object.generate_mapping_list")
        layout.operator("object.apply_bvh_mapping")

        if settings.mappings:
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
                col.operator(
                    "object.add_urdf_bone", text="", icon="ADD"
                ).bvh_bone_name = active.bvh_bone_name
                col.operator(
                    "object.remove_urdf_bone", text="", icon="REMOVE"
                ).bvh_bone_name = active.bvh_bone_name


# Helper Operators
class OT_GenerateMappingList(Operator):
    bl_idname = "object.generate_mapping_list"
    bl_label = "Generate Mapping List"

    def execute(self, context):
        bvh = context.scene.bvh_rig_object
        if not bvh:
            return {"CANCELLED"}
        context.scene.bvh_mapping_settings.mappings.clear()
        for pb in bvh.pose.bones:
            context.scene.bvh_mapping_settings.mappings.add().bvh_bone_name = pb.name
        return {"FINISHED"}


class OT_ApplyBVHMapping(Operator):
    bl_idname = "object.apply_bvh_mapping"
    bl_label = "Apply Mapping"

    def execute(self, context):
        # Reset Calibration Offsets
        if context.scene.urdf_rig_object:
            for pb in context.scene.urdf_rig_object.pose.bones:
                if "offset" in pb:
                    del pb["offset"]
        context.scene.bvh_mapping_settings.live_retarget = True
        return {"FINISHED"}


class OT_AddBVHBone(Operator):
    bl_idname = "object.add_urdf_bone"
    bl_label = "Add"
    bvh_bone_name: StringProperty()

    def execute(self, context):
        m = next(
            i
            for i in context.scene.bvh_mapping_settings.mappings
            if i.bvh_bone_name == self.bvh_bone_name
        )
        m.urdf_bones.add()
        return {"FINISHED"}


class OT_RemoveBVHBone(Operator):
    bl_idname = "object.remove_urdf_bone"
    bl_label = "Rem"
    bvh_bone_name: StringProperty()

    def execute(self, context):
        m = next(
            i
            for i in context.scene.bvh_mapping_settings.mappings
            if i.bvh_bone_name == self.bvh_bone_name
        )
        m.urdf_bones.remove(context.scene.bvh_mapping_settings.active_urdf_index)
        return {"FINISHED"}


class IMPORT_OT_urdf_humanoid(Operator, ImportHelper):
    bl_idname = "import_scene.urdf_humanoid"
    bl_label = "Import URDF"
    filename_ext = ".urdf"

    def execute(self, context):
        robot = parse_urdf(self.filepath)
        arm, link_mats = create_urdf_armature(robot)
        bind_meshes(robot, arm, link_mats, os.path.dirname(self.filepath))
        context.scene.urdf_rig_object = arm
        return {"FINISHED"}


def menu_func_import(self, context):
    self.layout.operator(
        IMPORT_OT_urdf_humanoid.bl_idname, text="URDF Humanoid (.urdf)"
    )


classes = [
    BVHMappingBone,
    BVHMappingItem,
    BVHMappingSettings,
    UL_BVHMappingList,
    UL_URDFBoneList,
    PANEL_BVHMapping,
    OT_GenerateMappingList,
    OT_ApplyBVHMapping,
    OT_AddBVHBone,
    OT_RemoveBVHBone,
    IMPORT_OT_urdf_humanoid,
]


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.bvh_mapping_settings = PointerProperty(type=BVHMappingSettings)
    bpy.types.Scene.urdf_rig_object = PointerProperty(type=bpy.types.Object)
    bpy.types.Scene.bvh_rig_object = PointerProperty(type=bpy.types.Object)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)
    bpy.app.handlers.frame_change_post.append(retarget_frame)


def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    bpy.app.handlers.frame_change_post.remove(retarget_frame)
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
