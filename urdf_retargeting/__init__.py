bl_info = {
    "name": "URDF Retargeting",
    "author": "Dominik Brämer",
    "version": (1, 0, 0),
    "blender": (5, 0, 0),
    "location": "File > Import > URDF Humanoid",
    "description": "URDF Import + BVH mapping with beyond_mimic export",
    "category": "Import-Export",
    "type": "add-on",
}

import bpy
import os
import csv
import json
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
# BUILDING & BINDING
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
    link_to_bone = {link_name: bones.new(link_name) for link_name in link_mats}
    for j in robot.joints.values():
        b = link_to_bone.get(j.child)
        if b:
            head = link_mats[j.child].to_translation()
            joint_rot = mathutils.Euler(j.rpy, "XYZ").to_matrix()
            axis_world = (
                link_mats[j.parent].to_3x3() @ joint_rot @ mathutils.Vector(j.axis)
            ).normalized()
            b.head, b.tail = head, head + axis_world * 0.05
            if j.parent in link_to_bone:
                b.parent = link_to_bone[j.parent]
    bpy.ops.object.mode_set(mode="OBJECT")
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
                target_mtx = link_mats[link_name] @ origin_to_matrix(vis.xyz, vis.rpy)
                obj.parent = arm_obj
                if link_name in arm_obj.pose.bones:
                    obj.parent_type, obj.parent_bone = "BONE", link_name
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
    joint_smoothing: FloatProperty(
        name="Joint Smoothing",
        description="Low-pass filter on joint angles (actuator space)",
        default=0.75,
        min=0.0,
        max=1.0,
    )
    root_scale: FloatProperty(name="Root Scale", default=1.0)
    location_offset: FloatVectorProperty(name="Loc Offset", subtype="TRANSLATION")
    rotation_offset: FloatVectorProperty(name="Rot Offset", subtype="EULER")
    transfer_root_rot: BoolProperty(name="Transfer World Rotation", default=True)
    target_hz: IntProperty(name="Export Hz", default=50, min=1, max=240)
    auto_grounding: BoolProperty(name="Auto Grounding", default=True)


# ============================================================
# RETARGETING ENGINE
# ============================================================


@persistent
def retarget_frame(scene):
    settings = scene.bvh_mapping_settings
    if not settings.live_retarget:
        return
    urdf, bvh = scene.urdf_rig_object, scene.bvh_rig_object
    if not urdf or not bvh:
        return

    if bvh.pose.bones:
        bvh_root = bvh.matrix_world @ bvh.pose.bones[0].matrix
        urdf.location = (
            bvh_root.to_translation() * settings.root_scale
        ) + mathutils.Vector(settings.location_offset)
        if settings.transfer_root_rot:
            off_q = mathutils.Euler(settings.rotation_offset).to_quaternion()
            urdf.rotation_mode, urdf.rotation_quaternion = (
                "QUATERNION",
                bvh_root.to_quaternion() @ off_q,
            )

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
            if "offset" not in urdf_b:
                urdf_b["offset"] = val
            final_angle = val - urdf_b["offset"] + b.neutral_offset
            l_min, l_max = urdf_b.get("limit_lower", -3.14), urdf_b.get(
                "limit_upper", 3.14
            )
            final_angle = max(min(final_angle, l_max), l_min)
            # Store joint angle explicitly for export (avoid Euler later)
            urdf_b["_joint_angle"] = final_angle
            target_q = mathutils.Euler((0, final_angle, 0)).to_quaternion()
            urdf_b.rotation_mode = "QUATERNION"
            # URDF joint smoothing
            urdf_b.rotation_quaternion = urdf_b.rotation_quaternion.slerp(
                target_q, 1 - settings.joint_smoothing
            )


# ============================================================
# EXPORT OPERATOR (DIR Selection + Dual File Output)
# ============================================================


class OT_ExportBeyondMimic(Operator):
    """Exports both CSV trajectory and JSON metadata with time-based resampling."""

    bl_idname = "object.export_beyond_mimic"
    bl_label = "Export CSV & JSON"
    bl_description = "Select directory; files will be named after the Source BVH rig and resampled to Target Hz"

    directory: StringProperty(name="Export Directory", subtype="DIR_PATH")

    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings
        urdf, bvh = scene.urdf_rig_object, scene.bvh_rig_object

        if not urdf or not bvh or not self.directory:
            self.report({"ERROR"}, "Robot, BVH Rig or Directory missing!")
            return {"CANCELLED"}

        # 1. Prepare Paths
        base_name = bpy.path.clean_name(bvh.name)
        csv_path = os.path.join(self.directory, f"{base_name}.csv")
        json_path = os.path.join(self.directory, f"{base_name}_meta.json")

        # 2. Resampling Setup
        # Calculate total duration in seconds
        fps = scene.render.fps
        total_frames = scene.frame_end - scene.frame_start
        duration = total_frames / fps

        # Calculate total steps based on target Hz
        target_hz = settings.target_hz
        num_steps = int(duration * target_hz) + 1
        time_per_step = 1.0 / target_hz

        joints = list(urdf.get("urdf_joint_order", []))
        header = [
            "root_tx",
            "root_ty",
            "root_tz",
            "root_qx",
            "root_qy",
            "root_qz",
            "root_qw",
        ] + joints

        data_rows, meta_joints = [], {}
        orig_f = scene.frame_current

        # 3. Sampling Loop (Time-based Resampling)
        for step in range(num_steps):
            # Calculate current time in seconds
            current_time_secs = step * time_per_step

            # Map time back to Blender frames (including subframes for precision)
            blender_frame = scene.frame_start + (current_time_secs * fps)

            # Set frame and subframe (crucial for smooth interpolation)
            scene.frame_set(int(blender_frame), subframe=blender_frame % 1.0)

            if settings.auto_grounding:
                min_z = min((urdf.matrix_world @ pb.head).z for pb in urdf.pose.bones)
                urdf.location.z -= min_z

            # Collect Root Data
            row = [
                urdf.location.x,
                urdf.location.y,
                urdf.location.z,
                urdf.rotation_quaternion.x,
                urdf.rotation_quaternion.y,
                urdf.rotation_quaternion.z,
                urdf.rotation_quaternion.w,
            ]

            # Collect Joint Data
            for j in joints:
                pb = urdf.pose.bones[j]
                row.append(pb.get("_joint_angle", 0.0))

                # Metadata (limits) only needed once
                if step == 0:
                    meta_joints[j] = {
                        "lower": pb.get("limit_lower", -3.14),
                        "upper": pb.get("limit_upper", 3.14),
                    }

            data_rows.append(row)

        # 4. Write Files
        try:
            with open(csv_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_rows)

            with open(json_path, "w") as f:
                json.dump(
                    {
                        # Origin Info
                        "source_bvh": base_name,
                        "target_urdf": bpy.path.clean_name(urdf.name),
                        # Timing
                        "source_fps": fps,
                        "source_total_frames": total_frames,
                        "export_hz": target_hz,
                        "duration_secs": duration,
                        "total_samples": len(data_rows),
                        # Export Settings
                        "bvh_prefilter": settings.bvh_prefilter,
                        "joint_smoothing": settings.joint_smoothing,
                        # Units
                        "measurement_unit": "meters",
                        "angle_unit": "radians",
                        # Root / Base definition
                        "root": {
                            "type": "free_floating_base",
                            "frame": "blender_world",
                            "dofs": ["x", "y", "z", "qx", "qy", "qz", "qw"],
                            "representation": "quaternion",
                            "note": "No semantic root bone; base pose is encoded as world-space transform of the URDF armature object",
                        },
                        # Coordinate system (explicit but honest)
                        "coordinate_system": {
                            "space": "blender_world",
                            "up_axis": "Z",
                            "forward_axis": None,
                            "right_axis": "X",
                            "note": "URDF does not define a semantic forward axis; heading is encoded in the root orientation quaternion",
                        },
                        # Joint definition
                        "joints": {
                            "order": joints,
                            "type": "hinge",
                            "angle_unit": "radians",
                            "limits": meta_joints,
                        },
                    },
                    f,
                    indent=4,
                )

            self.report(
                {"INFO"},
                f"Resampled to {target_hz}Hz: {base_name}.csv and {base_name}_meta.json",
            )

        except Exception as e:
            self.report({"ERROR"}, f"File Error: {str(e)}")
            return {"CANCELLED"}

        scene.frame_set(orig_f)
        return {"FINISHED"}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}


# ============================================================
# UI IMPLEMENTATION
# ============================================================


class UL_BVHMappingList(UIList):
    def draw_item(
        self, context, layout, data, item, icon, active_data, active_propname, index
    ):
        row = layout.row(align=True)
        row.label(text=f"BVH: {item.bvh_bone_name}")
        count = len(item.urdf_bones)
        if count > 0:
            row.label(text=f"URDF: {count}")
        else:
            row.enabled = False
            row.label(text=f"URDF: {count}")


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
    bl_category = "BVH → URDF Retargeting"

    def draw(self, context):
        layout = self.layout
        s = context.scene
        settings = s.bvh_mapping_settings
        layout.prop(s, "urdf_rig_object")
        layout.prop(s, "bvh_rig_object")

        box = layout.box()
        box.label(text="BVH → URDF Options")
        box.prop(settings, "root_scale")
        box.prop(settings, "transfer_root_rot")
        box.prop(settings, "location_offset")
        box.prop(settings, "rotation_offset")
        box.prop(settings, "joint_smoothing")

        layout.prop(settings, "live_retarget", toggle=True)
        layout.operator("object.generate_mapping_list")
        if not settings.mappings:
            layout.label(
                text="Click 'Generate Mapping List' to show bones.", icon="INFO"
            )
            return
        else:
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
        layout.operator("object.apply_bvh_mapping")

        layout.separator()
        box = layout.box()
        box.label(text="Export Beyond Mimic", icon="EXPORT")
        box.prop(settings, "target_hz")
        box.prop(settings, "auto_grounding")
        box.operator("object.export_beyond_mimic", text="Export")


# ============================================================
# OPERATORS & REGISTRATION
# ============================================================


class OT_GenerateMappingList(Operator):
    bl_idname = "object.generate_mapping_list"
    bl_label = "Generate Mapping List"
    bl_description = "Generates BVH bone list from selected BVH rig"

    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings
        bvh_rig = scene.bvh_rig_object
        if not bvh_rig:
            return {"CANCELLED"}

        settings.mappings.clear()
        for ub in [pb.name for pb in bvh_rig.pose.bones]:
            item = settings.mappings.add()
            item.bvh_bone_name = ub

        settings.active_mapping_index = 0
        settings.active_urdf_index = 0
        return {"FINISHED"}


class OT_ApplyBVHMapping(Operator):
    bl_idname = "object.apply_bvh_mapping"
    bl_label = "Apply Mapping"
    bl_description = "Applies the current BVH → URDF mapping once"

    def execute(self, context):
        if context.scene.urdf_rig_object:
            for pb in context.scene.urdf_rig_object.pose.bones:
                for key in ("offset", "_joint_angle"):
                    pb.pop(key, None)

        for key in ("_prev_euler_cache", "_bvh_prefilter_cache"):
            if key in context.scene:
                context.scene.pop(key, None)

        context.scene.bvh_mapping_settings.live_retarget = True
        return {"FINISHED"}


class OT_AddBVHBone(Operator):
    bl_idname = "object.add_urdf_bone"
    bl_label = "Add"
    bl_description = "Add URDF Bone to Mapping"
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
    bl_description = "Remove URDF Bone from Mapping"
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
    bl_description = (
        "Imports a URDF humanoid robot and creates an armature with bound meshes."
    )
    filename_ext = ".urdf"

    def execute(self, context):
        robot = parse_urdf(self.filepath)
        arm, link_mats = create_urdf_armature(robot)
        bind_meshes(robot, arm, link_mats, os.path.dirname(self.filepath))
        context.scene.urdf_rig_object = arm
        arm["urdf_joint_order"] = [j.child for j in robot.joints.values()]
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
    OT_ExportBeyondMimic,
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
