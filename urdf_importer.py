bl_info = {
    "name": "URDF Humanoid Multi-BVH Retarget",
    "author": "Dominik + ChatGPT",
    "version": (1, 0, 2),
    "blender": (3, 0, 0),
    "location": "File > Import > URDF Humanoid",
    "description": "URDF Import + Multi-BVH mapping with interactive N-Panel (BVH → URDF)",
    "category": "Import-Export",
}

import bpy
import os
import mathutils
from bpy_extras.io_utils import ImportHelper
import xml.etree.ElementTree as ET
from bpy.props import StringProperty, PointerProperty, CollectionProperty
from bpy.types import PropertyGroup, Operator, Panel

# ============================================================
# URDF DATA STRUCTURES
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

# ============================================================
# URDF PARSER
# ============================================================
def parse_float_list(s, n):
    if s is None:
        return [0.0] * n
    parts = s.replace(',', ' ').split()
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
        axis = parse_float_list(axis_el.attrib.get("xyz") if axis_el is not None else None, 3)
        joint = URDFJoint(name, jtype, parent, child, xyz, rpy, axis)
        robot.joints[name] = joint
    return robot

# ============================================================
# UTILITY
# ============================================================
def origin_to_matrix(xyz, rpy):
    loc = mathutils.Vector(xyz)
    rot = mathutils.Euler(rpy, 'XYZ').to_matrix().to_4x4()
    return mathutils.Matrix.Translation(loc) @ rot

def axis_to_vec(axis):
    v = mathutils.Vector(axis)
    return v.normalized() if v.length > 0 else mathutils.Vector((0, 0, 1))

def resolve_mesh_path(mesh, base_path):
    if mesh.startswith("package://"):
        mesh = mesh.replace("package://", "")
        parts = mesh.split("/", 1)
        if len(parts) == 2:
            mesh = parts[1]
    return os.path.normpath(os.path.join(base_path, mesh))

# ============================================================
# URDF ARMATURE + MESHES
# ============================================================
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
            T = origin_to_matrix(j.xyz, j.rpy)
            mats[j.child] = parent_mat @ T
            stack.append(j.child)
    return mats, root

def create_urdf_armature(robot):
    link_mats, root = build_link_transforms(robot)
    bpy.ops.object.add(type='ARMATURE', enter_editmode=True)
    arm_obj = bpy.context.object
    arm_obj.name = f"{robot.name}_URDF_Rig"
    arm = arm_obj.data
    bones = arm.edit_bones
    link_to_bone = {}
    bone_len = 0.05
    for j in robot.joints.values():
        b = bones.new(j.child)
        link_to_bone[j.child] = b
    for j in robot.joints.values():
        b = link_to_bone[j.child]
        child_mat = link_mats[j.child]
        head = child_mat.to_translation()
        axis = axis_to_vec(j.axis)
        tail = head + axis * bone_len
        b.head = head
        b.tail = tail
        if j.parent in link_to_bone:
            b.parent = link_to_bone[j.parent]
    bpy.ops.object.mode_set(mode='POSE')
    for pb in arm_obj.pose.bones:
        pb.rotation_mode = 'XYZ'
    bpy.ops.object.mode_set(mode='OBJECT')
    return arm_obj, link_mats

def import_mesh(mesh_path):
    ext = os.path.splitext(mesh_path)[1].lower()
    mesh_path = os.path.normpath(mesh_path)
    if ext == ".stl":
        bpy.ops.wm.stl_import(filepath=mesh_path)
    elif ext == ".obj":
        bpy.ops.import_scene.obj(filepath=mesh_path)
    elif ext == ".dae":
        bpy.ops.wm.collada_import(filepath=mesh_path)
    else:
        return None
    return bpy.context.selected_objects[0]

def bind_meshes(robot, arm_obj, link_mats, base_path):
    for link in robot.links.values():
        link_mat = link_mats.get(link.name, mathutils.Matrix.Identity(4))
        for idx, vis in enumerate(link.visuals):
            mesh_path = resolve_mesh_path(vis.mesh, base_path)
            if not os.path.isfile(mesh_path):
                continue
            obj = import_mesh(mesh_path)
            if not obj:
                continue
            obj.name = f"{link.name}_{idx}"
            obj.matrix_world = link_mat @ origin_to_matrix(vis.xyz, vis.rpy)
            obj.parent = arm_obj
            obj.parent_type = 'OBJECT'

# ============================================================
# Multi-BVH Mapping
# ============================================================
class BVHMappingBone(PropertyGroup):
    bvh_bone_name: StringProperty(name="BVH Bone")

class BVHMappingItem(PropertyGroup):
    urdf_bones: CollectionProperty(type=BVHMappingBone)
    bvh_bone_name: StringProperty(name="BVH Bone")

class BVHMappingSettings(PropertyGroup):
    mappings: CollectionProperty(type=BVHMappingItem)

# OT_GenerateMappingList Operator
class OT_GenerateMappingList(bpy.types.Operator):
    bl_idname = "object.generate_mapping_list"
    bl_label = "Generate Mapping List"
    
    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings
        bvh_rig = scene.bvh_rig_object
        if not bvh_rig:
            return {'CANCELLED'}
        
        settings.mappings.clear()
        for ub in [pb.name for pb in bvh_rig.pose.bones]:
            item = settings.mappings.add()
            item.bvh_bone_name = ub
        return {'FINISHED'}

# OT_ApplyBVHMapping Operator
class OT_ApplyBVHMapping(bpy.types.Operator):
    bl_idname = "object.apply_bvh_mapping"
    bl_label = "Apply Mapping"
    
    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings
        urdf_rig = scene.urdf_rig_object
        bvh_rig = scene.bvh_rig_object
        if not urdf_rig or not bvh_rig:
            return {'CANCELLED'}
        
        for item in settings.mappings:
            for b in item.urdf_bones:
                urdf_b = urdf_rig.pose.bones.get(b.name)
                bvh_b = bvh_rig.pose.bones.get(item.bvh_bone_name)
                if urdf_b and bvh_b:
                    # Apply Child-of constraint for URDF-Bone → BVH-Bone
                    c = urdf_b.constraints.new("CHILD_OF")
                    c.target = bvh_rig
                    c.subtarget = item.bvh_bone_name
                    c.inverse_matrix = urdf_b.matrix.inverted()
        
        bpy.context.view_layer.update()
        return {'FINISHED'}

# OT_AddBVHBone Operator
class OT_AddBVHBone(bpy.types.Operator):
    bl_idname = "object.add_urdf_bone"
    bl_label = "Add URDF Bone"
    
    bvh_bone_name: StringProperty()

    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings
        for item in settings.mappings:
            if item.bvh_bone_name == self.bvh_bone_name:
                new_bone = item.urdf_bones.add()
                new_bone.bvh_bone_name = self.bvh_bone_name  # Link URDF bone to the selected BVH bone
        return {'FINISHED'}

# Operator für das Entfernen von BVH-Bones
class OT_RemoveBVHBone(bpy.types.Operator):
    bl_idname = "object.remove_urdf_bone"
    bl_label = "Remove URDF Bone"
    
    bvh_bone_name: StringProperty()
    index: bpy.props.IntProperty()
    
    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings
        for item in settings.mappings:
            if item.bvh_bone_name == self.bvh_bone_name:
                item.urdf_bones.remove(self.index)
        return {'FINISHED'}
    
# Operator für das Zurücksetzen der Mapping-Liste
class OT_ResetBVHMapping(bpy.types.Operator):
    bl_idname = "object.reset_bvh_mapping"
    bl_label = "Reset Mapping List"
    
    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings
        settings.mappings.clear()  # Alle Mappings zurücksetzen
        return {'FINISHED'}

# ============================================================
# N-Panel UI
# ============================================================
class PANEL_BVHMapping(bpy.types.Panel):
    bl_label = "BVH -> URDF Mapping"
    bl_idname = "VIEW3D_PT_bvh_mapping"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'URDF Retarget'
    
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        settings = scene.bvh_mapping_settings

        layout.prop(scene, "urdf_rig_object")
        layout.prop(scene, "bvh_rig_object")
        layout.operator("object.generate_mapping_list", text="Generate Mapping List")
        layout.operator("object.apply_bvh_mapping", text="Apply Mapping")
        layout.operator("object.reset_bvh_mapping", text="Reset Mapping List")

        if not settings.mappings:
            layout.label(text="Click 'Generate Mapping List' to show bones.")
            return

        for item in settings.mappings:
            box = layout.box()
            box.label(text=f"BVH Bone: {item.bvh_bone_name}")

            for idx, b in enumerate(item.urdf_bones):
                row = box.row()
                if scene.urdf_rig_object:  # Use URDF bones for selection
                    row.prop_search(b, "bvh_bone_name", scene.urdf_rig_object.pose, "bones", text="")
                op = row.operator("object.remove_urdf_bone", text="-")
                op.bvh_bone_name = item.bvh_bone_name
                op.index = idx

            add_op = box.operator("object.add_urdf_bone", text="+ Add URDF Bone")
            add_op.bvh_bone_name = item.bvh_bone_name

# ============================================================
# Import URDF Operator
# ============================================================
class IMPORT_OT_urdf_humanoid(bpy.types.Operator, ImportHelper):
    bl_idname = "import_scene.urdf_humanoid"
    bl_label = "Import URDF Humanoid"
    filename_ext = ".urdf"
    
    def execute(self, context):
        urdf_path = os.path.normpath(self.filepath)
        base_path = os.path.dirname(urdf_path)
        robot = parse_urdf(urdf_path)
        urdf_arm, link_mats = create_urdf_armature(robot)
        bind_meshes(robot, urdf_arm, link_mats, base_path)
        context.scene.urdf_rig_object = urdf_arm
        return {'FINISHED'}

# ============================================================
# REGISTER
# ============================================================
classes = [
    BVHMappingBone,
    BVHMappingItem,
    BVHMappingSettings,
    OT_GenerateMappingList,
    OT_AddBVHBone,
    OT_RemoveBVHBone,
    OT_ApplyBVHMapping,
    OT_ResetBVHMapping,
    PANEL_BVHMapping,
    IMPORT_OT_urdf_humanoid
]

def menu_func_import(self, context):
    self.layout.operator(IMPORT_OT_urdf_humanoid.bl_idname, text="URDF Humanoid (.urdf)")

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.bvh_mapping_settings = PointerProperty(type=BVHMappingSettings)
    bpy.types.Scene.urdf_rig_object = PointerProperty(name="URDF Rig", type=bpy.types.Object)
    bpy.types.Scene.bvh_rig_object = PointerProperty(name="BVH Rig", type=bpy.types.Object)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.bvh_mapping_settings
    del bpy.types.Scene.urdf_rig_object
    del bpy.types.Scene.bvh_rig_object
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)

if __name__ == "__main__":
    register()
