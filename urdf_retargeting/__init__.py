bl_info = {
    "name": "URDF Retargeting",
    "author": "Dominik Brämer",
    "version": (1, 0, 0),
    "blender": (5, 0, 0),
    "location": "File > Import > URDF Humanoid",
    "description": "URDF Import + BVH mapping for retargeting to URDF rigs",
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
    def __init__(self, name, jtype, parent, child, xyz, rpy, axis):
        self.name = name
        self.type = jtype
        self.parent = parent
        self.child = child
        self.xyz = xyz
        self.rpy = rpy
        self.axis = axis

class URDFRobot:
    def __init__(self, name):
        self.name = name
        self.links = {}
        self.joints = {}

def parse_float_list(s, n):
    if s is None: return [0.0] * n
    parts = s.replace(',', ' ').split()
    vals = []
    for p in parts:
        try: vals.append(float(p))
        except: vals.append(0.0)
    while len(vals) < n: vals.append(0.0)
    return vals[:n]

def parse_urdf(path):
    tree = ET.parse(path)
    root = tree.getroot()
    robot = URDFRobot(root.attrib.get("name", "URDF_Robot"))
    for link_el in root.findall("link"):
        link = URDFLink(link_el.attrib["name"])
        for vis_el in link_el.findall("visual"):
            org = vis_el.find("origin")
            xyz = parse_float_list(org.attrib.get("xyz") if org is not None else None, 3)
            rpy = parse_float_list(org.attrib.get("rpy") if org is not None else None, 3)
            geom = vis_el.find("geometry")
            mesh_el = geom.find("mesh") if geom is not None else None
            mesh = mesh_el.attrib.get("filename") if mesh_el is not None else None
            if mesh: link.visuals.append(URDFVisual(xyz, rpy, mesh))
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
        axis = parse_float_list(axis_el.attrib.get("xyz") if axis_el is not None else None, 3)
        robot.joints[name] = URDFJoint(name, jtype, parent, child, xyz, rpy, axis)
    return robot

# ============================================================
# UTILITY
# ============================================================
def origin_to_matrix(xyz, rpy):
    loc = mathutils.Vector(xyz)
    rot = mathutils.Euler(rpy, 'XYZ').to_matrix().to_4x4()
    return mathutils.Matrix.Translation(loc) @ rot

def build_link_transforms(robot):
    children = {}
    joint_by_child = {}
    for j in robot.joints.values():
        children.setdefault(j.parent, []).append(j)
        joint_by_child[j.child] = j
    all_links = set(l.name for l in robot.links.values())
    child_links = set(joint_by_child.keys())
    roots = list(all_links - child_links)
    root = roots[0] if roots else list(all_links)[0]
    mats = {root: mathutils.Matrix.Identity(4)}
    stack = [root]
    while stack:
        parent = stack.pop()
        parent_mat = mats[parent]
        for j in children.get(parent, []):
            mats[j.child] = parent_mat @ origin_to_matrix(j.xyz, j.rpy)
            stack.append(j.child)
    return mats, root

def create_urdf_armature(robot):
    link_mats, root = build_link_transforms(robot)
    bpy.ops.object.add(type='ARMATURE', enter_editmode=True)
    arm_obj = bpy.context.object
    arm_obj.name = f"{robot.name}_URDF_Rig"
    bones = arm_obj.data.edit_bones
    link_to_bone = {}
    for j in robot.joints.values(): link_to_bone[j.child] = bones.new(j.child)
    for j in robot.joints.values():
        b = link_to_bone[j.child]
        head = link_mats[j.child].to_translation()
        joint_rot = mathutils.Euler(j.rpy, 'XYZ').to_matrix()
        axis_world = (link_mats[j.parent].to_3x3() @ joint_rot @ mathutils.Vector(j.axis)).normalized()
        b.head = head
        b.tail = head + axis_world * 0.05
        if j.parent in link_to_bone: b.parent = link_to_bone[j.parent]
    bpy.ops.object.mode_set(mode='POSE')
    for pb in arm_obj.pose.bones: pb.rotation_mode = 'XYZ'
    bpy.ops.object.mode_set(mode='OBJECT')
    return arm_obj, link_mats

# ============================================================
# DATA STRUCTURES
# ============================================================
axis_items = [('X', "X", ""), ('Y', "Y", ""), ('Z', "Z", "")]
sign_items = [('POS', "+", ""), ('NEG', "−", "")]

class BVHMappingBone(PropertyGroup):
    bvh_bone_name: StringProperty(name="URDF Bone")
    source_axis: EnumProperty(name="BVH Axis", items=axis_items, default='X')
    sign: EnumProperty(name="Sign", items=sign_items, default='NEG')
    neutral_offset: FloatProperty(name="Neutral Offset", default=0.0, subtype='ANGLE')

class BVHMappingItem(PropertyGroup):
    urdf_bones: CollectionProperty(type=BVHMappingBone)
    bvh_bone_name: StringProperty(name="BVH Bone")
    ref_rot: FloatVectorProperty(size=4, default=(1.0, 0.0, 0.0, 0.0))

class BVHMappingSettings(PropertyGroup):
    mappings: CollectionProperty(type=BVHMappingItem)
    active_mapping_index: IntProperty(default=0)
    active_urdf_index: IntProperty(default=0)
    live_retarget: BoolProperty(name="Live Retargeting", default=False)
    smoothing: FloatProperty(name="Smoothing", default=0.25, min=0.0, max=1.0)
    
    # Root Motion & Global Offsets
    root_scale: FloatProperty(name="Root Scale", default=1.0, min=0.01)
    location_offset: FloatVectorProperty(name="Location Offset", description="Global X, Y, Z offset", default=(0.0, 0.0, 0.0), subtype='TRANSLATION')
    rotation_offset: FloatVectorProperty(name="Rotation Offset", description="Global Euler rotation offset", default=(0.0, 0.0, 0.0), subtype='EULER')
    transfer_root_rot: BoolProperty(name="Transfer World Rotation", default=True)

# ============================================================
# ENGINE
# ============================================================
@persistent
def retarget_frame(scene):
    settings = scene.bvh_mapping_settings
    if not settings.live_retarget: return
    urdf_obj = scene.urdf_rig_object
    bvh_rig = scene.bvh_rig_object
    if not urdf_obj or not bvh_rig: return

    # 1. ROOT MOTION & GLOBAL OFFSETS
    if bvh_rig.pose.bones:
        bvh_hips = bvh_rig.pose.bones[0]
        bvh_world_m = bvh_rig.matrix_world @ bvh_hips.matrix
        
        # Translation (BVH Pos * Scale + User Offset)
        scaled_loc = bvh_world_m.to_translation() * settings.root_scale
        urdf_obj.location = scaled_loc + mathutils.Vector(settings.location_offset)
        
        # Rotation (BVH Rot * User Offset)
        if settings.transfer_root_rot:
            urdf_obj.rotation_mode = 'QUATERNION'
            off_q = mathutils.Euler(settings.rotation_offset).to_quaternion()
            urdf_obj.rotation_quaternion = bvh_world_m.to_quaternion() @ off_q

    # 2. BONE ROTATIONS
    if "_prev_euler_cache" not in scene: scene["_prev_euler_cache"] = {}
    cache = scene["_prev_euler_cache"]

    for item in settings.mappings:
        bvh_b = bvh_rig.pose.bones.get(item.bvh_bone_name)
        if not bvh_b: continue
        ref_q = mathutils.Quaternion(item.ref_rot)
        cur_q = bvh_b.matrix_basis.to_quaternion()
        rot_rel = ref_q.inverted() @ cur_q
        e = rot_rel.to_euler('XYZ')
        bone_key = f"{bvh_rig.name}:{bvh_b.name}"
        if bone_key in cache: e.make_compatible(cache[bone_key])
        cache[bone_key] = e.copy()

        for b in item.urdf_bones:
            urdf_b = urdf_obj.pose.bones.get(b.bvh_bone_name)
            if not urdf_b: continue
            val = getattr(e, b.source_axis.lower())
            if b.sign == 'NEG': val = -val
            if "offset" not in urdf_b: urdf_b["offset"] = val
            target_q = mathutils.Euler((0, val - urdf_b["offset"] + b.neutral_offset, 0)).to_quaternion()
            urdf_b.rotation_mode = 'QUATERNION'
            urdf_b.rotation_quaternion = urdf_b.rotation_quaternion.slerp(target_q, settings.smoothing)

# ============================================================
# UI & REGISTRATION
# ============================================================
class UL_BVHMappingList(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        row = layout.row(); row.label(text=item.bvh_bone_name); row.label(text=f"{len(item.urdf_bones)} URDF")

class UL_URDFBoneList(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        row = layout.row()
        if context.scene.urdf_rig_object: row.prop_search(item, "bvh_bone_name", context.scene.urdf_rig_object.pose, "bones", text="")
        row.prop(item, "source_axis", text=""); row.prop(item, "sign", text="")

class PANEL_BVHMapping(Panel):
    bl_label = "BVH -> URDF Mapping"
    bl_idname = "VIEW3D_PT_bvh_mapping"; bl_space_type = 'VIEW_3D'; bl_region_type = 'UI'; bl_category = 'URDF Retarget'
    def draw(self, context):
        layout = self.layout; scene = context.scene; settings = scene.bvh_mapping_settings
        col = layout.column(align=True); col.prop(scene, "urdf_rig_object"); col.prop(scene, "bvh_rig_object")
        
        box = layout.box(); box.label(text="Root Motion & Global Offsets", icon='TRANSFORM_ORIGINS')
        box.prop(settings, "root_scale")
        box.prop(settings, "location_offset", text="Loc Offset")
        box.prop(settings, "rotation_offset", text="Rot Offset")
        box.prop(settings, "transfer_root_rot")

        layout.prop(settings, "live_retarget", toggle=True, icon='PLAY' if not settings.live_retarget else 'PAUSE')
        layout.prop(settings, "smoothing")
        row = layout.row(align=True); row.operator("object.generate_mapping_list"); row.operator("object.apply_bvh_mapping", icon='CHECKMARK')
        if settings.mappings:
            layout.template_list("UL_BVHMappingList", "", settings, "mappings", settings, "active_mapping_index")
            if 0 <= settings.active_mapping_index < len(settings.mappings):
                active = settings.mappings[settings.active_mapping_index]; box = layout.box()
                row = box.row(); row.template_list("UL_URDFBoneList", "", active, "urdf_bones", settings, "active_urdf_index")
                col = row.column(align=True); col.operator("object.add_urdf_bone", text="", icon='ADD').bvh_bone_name = active.bvh_bone_name
                col.operator("object.remove_urdf_bone", text="", icon='REMOVE').bvh_bone_name = active.bvh_bone_name

class OT_AddBVHBone(Operator):
    bl_idname = "object.add_urdf_bone"; bl_label = "Add"; bvh_bone_name: StringProperty()
    def execute(self, context):
        next(m for m in context.scene.bvh_mapping_settings.mappings if m.bvh_bone_name == self.bvh_bone_name).urdf_bones.add(); return {'FINISHED'}

class OT_RemoveBVHBone(Operator):
    bl_idname = "object.remove_urdf_bone"; bl_label = "Remove"; bvh_bone_name: StringProperty(); index: IntProperty()
    def execute(self, context):
        next(m for m in context.scene.bvh_mapping_settings.mappings if m.bvh_bone_name == self.bvh_bone_name).urdf_bones.remove(self.index); return {'FINISHED'}

class OT_ApplyBVHMapping(Operator):
    bl_idname = "object.apply_bvh_mapping"; bl_label = "Apply Mapping"
    def execute(self, context):
        bvh = context.scene.bvh_rig_object
        if not bvh: return {'CANCELLED'}
        for item in context.scene.bvh_mapping_settings.mappings:
            bvh_b = bvh.pose.bones.get(item.bvh_bone_name)
            if bvh_b:
                name = bvh_b.name.lower(); e = mathutils.Euler((0,0,0))
                if any(k in name for k in ("hand", "wrist")):
                    if "left" in name or name.endswith(".l"): e.z = math.radians(90)
                    elif "right" in name or name.endswith(".r"): e.z = math.radians(-90)
                q = e.to_quaternion(); item.ref_rot = (q.w, q.x, q.y, q.z)
        context.scene.bvh_mapping_settings.live_retarget = True; return {'FINISHED'}

class OT_GenerateMappingList(Operator):
    bl_idname = "object.generate_mapping_list"; bl_label = "Generate Mapping List"
    def execute(self, context):
        settings = context.scene.bvh_mapping_settings; bvh = context.scene.bvh_rig_object
        if not bvh: return {'CANCELLED'}
        settings.mappings.clear()
        for pb in bvh.pose.bones: settings.mappings.add().bvh_bone_name = pb.name
        return {'FINISHED'}

class IMPORT_OT_urdf_humanoid(Operator, ImportHelper):
    bl_idname = "import_scene.urdf_humanoid"; bl_label = "Import URDF"; filename_ext = ".urdf"
    def execute(self, context):
        robot = parse_urdf(self.filepath); arm, _ = create_urdf_armature(robot)
        context.scene.urdf_rig_object = arm; return {'FINISHED'}

def menu_func_import(self, context): self.layout.operator(IMPORT_OT_urdf_humanoid.bl_idname, text="URDF Humanoid (.urdf)")

classes = [BVHMappingBone, BVHMappingItem, BVHMappingSettings, UL_BVHMappingList, UL_URDFBoneList,
           OT_GenerateMappingList, OT_AddBVHBone, OT_RemoveBVHBone, OT_ApplyBVHMapping, PANEL_BVHMapping, IMPORT_OT_urdf_humanoid]

def register():
    for cls in classes: bpy.utils.register_class(cls)
    bpy.types.Scene.bvh_mapping_settings = PointerProperty(type=BVHMappingSettings)
    bpy.types.Scene.urdf_rig_object = PointerProperty(type=bpy.types.Object)
    bpy.types.Scene.bvh_rig_object = PointerProperty(type=bpy.types.Object)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)
    bpy.app.handlers.frame_change_post.append(retarget_frame)

def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    bpy.app.handlers.frame_change_post.remove(retarget_frame)
    for cls in reversed(classes): bpy.utils.unregister_class(cls)

if __name__ == "__main__": 
    register()