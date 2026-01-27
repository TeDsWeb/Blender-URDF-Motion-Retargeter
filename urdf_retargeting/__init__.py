bl_info = {
    "name": "URDF Retargeting",
    "author": "Dominik Brämer",
    "version": (1, 0, 0),
    "blender": (5, 0, 0),
    "location": "File > Import > URDF Humanoid",
    "description": "URDF Import + BVH mapping with interactive N-Panel (BVH → URDF) using UILists",
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
)
from bpy.types import PropertyGroup, Operator, Panel, UIList
from bpy.app.handlers import persistent


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
        joint_rot = mathutils.Euler(j.rpy, 'XYZ').to_matrix()
        parent_rot = link_mats[j.parent].to_3x3()
        axis_local = mathutils.Vector(j.axis)
        axis_world = (parent_rot @ joint_rot @ axis_local).normalized()
        tail = head + axis_world * bone_len
        b.head = head
        b.tail = tail
        b.align_roll(mathutils.Vector((0, 0, 1)))
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
# Multi-BVH Mapping (mit UILists)
# ============================================================
axis_items = [
    ('X', "X", ""),
    ('Y', "Y", ""),
    ('Z', "Z", ""),
]

sign_items = [
    ('POS', "+", ""),
    ('NEG', "−", ""),
]

class BVHMappingBone(PropertyGroup):
    bvh_bone_name: StringProperty(name="URDF Bone")
    source_axis: EnumProperty(
        name="BVH Axis",
        items=axis_items,
        default='X',
    )
    sign: EnumProperty(
        name="Sign",
        items=sign_items,
        default='NEG',
    )
    neutral_offset: bpy.props.FloatProperty(
        name="Neutral Offset",
        description="Correction of neutral pose in degrees",
        default=0.0,
        subtype='ANGLE'
    )

class BVHMappingItem(PropertyGroup):
    urdf_bones: CollectionProperty(type=BVHMappingBone)
    bvh_bone_name: StringProperty(name="BVH Bone")
    # WICHTIG: Identitäts-Quaternion als Default, um Null-Quats zu vermeiden
    ref_rot: FloatVectorProperty(size=4, default=(1.0, 0.0, 0.0, 0.0))

class BVHMappingSettings(PropertyGroup):
    mappings: CollectionProperty(type=BVHMappingItem)
    active_mapping_index: IntProperty(default=0)
    active_urdf_index: IntProperty(default=0)
    live_retarget: BoolProperty(
        name="Live Retargeting",
        default=False
    )
    smoothing: bpy.props.FloatProperty(
        name="Smoothing",
        description="Smoothing factor for retargeting",
        default=0.25,
        min=0.0,
        max=1.0
    )

# UILists
class UL_BVHMappingList(UIList):
    bl_idname = "UL_BVHMappingList"

    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        row = layout.row()
        row.label(text=item.bvh_bone_name)
        row.label(text=f"{len(item.urdf_bones)} URDF")

class UL_URDFBoneList(UIList):
    bl_idname = "UL_URDFBoneList"

    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        scene = context.scene
        row = layout.row()
        if scene.urdf_rig_object:
            row.prop_search(item, "bvh_bone_name", scene.urdf_rig_object.pose, "bones", text="")
        else:
            row.label(text=item.bvh_bone_name or "<URDF Bone>")
        row.prop(item, "source_axis", text="")
        row.prop(item, "sign", text="")
        row.prop(item, "neutral_offset", text="")

# Operators
class OT_GenerateMappingList(Operator):
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

        settings.active_mapping_index = 0
        settings.active_urdf_index = 0
        return {'FINISHED'}
    
def store_pose(pose_bones):
    return {
        pb.name: pb.matrix_basis.copy()
        for pb in pose_bones
    }

def restore_pose(pose_bones, stored):
    for pb in pose_bones:
        if pb.name in stored:
            pb.matrix_basis = stored[pb.name]

# ============================================================
# T-POSE LOGIK
# ============================================================
def get_tpose_quaternion_for_bone(pb):
    name = pb.name.lower()
    e = mathutils.Euler((0.0, 0.0, 0.0), 'XYZ')

    # --- ARMS ---
    if any(k in name for k in ("upperarm", "humerus", "shoulder")):
        if "left" in name or name.endswith(".l"):
            e.z = math.radians(0)
        elif "right" in name or name.endswith(".r"):
            e.z = math.radians(0)

    elif any(k in name for k in ("forearm", "lowerarm", "elbow")):
        if "left" in name or name.endswith(".l"):
            e.z = math.radians(0)
        elif "right" in name or name.endswith(".r"):
            e.z = math.radians(0)

    elif any(k in name for k in ("hand", "wrist")):
        if "left" in name or name.endswith(".l"):
            e.z = math.radians(90)
        elif "right" in name or name.endswith(".r"):
            e.z = math.radians(-90)

    # --- LEGS ---
    elif any(k in name for k in ("thigh", "upperleg", "upleg")):
        e = mathutils.Euler((0.0, 0.0, 0.0), 'XYZ')

    elif any(k in name for k in ("calf", "lowerleg", "shin")):
        e = mathutils.Euler((0.0, 0.0, 0.0), 'XYZ')

    elif "foot" in name or "toe" in name:
        e = mathutils.Euler((0.0, 0.0, 0.0), 'XYZ')

    return e.to_quaternion()

def capture_reference_offsets(scene):
    """Speichert pro BVH-Bone die virtuelle T-Pose-Quaternion als Referenz."""
    settings = scene.bvh_mapping_settings
    bvh_rig = scene.bvh_rig_object

    if not bvh_rig:
        return

    for item in settings.mappings:
        bvh_b = bvh_rig.pose.bones.get(item.bvh_bone_name)
        if not bvh_b:
            continue

        q_tpose = get_tpose_quaternion_for_bone(bvh_b)
        item.ref_rot = (q_tpose.w, q_tpose.x, q_tpose.y, q_tpose.z)

class OT_ApplyBVHMapping(bpy.types.Operator):
    bl_idname = "object.apply_bvh_mapping"
    bl_label = "Apply Mapping (Setup)"
    
    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings
        urdf_rig = scene.urdf_rig_object
        bvh_rig = scene.bvh_rig_object

        current_frame = scene.frame_current
        scene.frame_set(current_frame)

        if not urdf_rig or not bvh_rig:
            self.report({'ERROR'}, "URDF or BVH rig missing")
            return {'CANCELLED'}

        # Nur virtuelle T-Pose-Referenz berechnen, BVH-Rig bleibt unverändert
        capture_reference_offsets(scene)

        # 1) Alle alten Constraints entfernen
        for pb in urdf_rig.pose.bones:
            for c in list(pb.constraints):
                pb.constraints.remove(c)

        # 2) Alle Offsets zurücksetzen
        for pb in urdf_rig.pose.bones:
            if "offset" in pb:
                del pb["offset"]

        # 3) Live Retarget aktivieren
        settings.live_retarget = True

        # 4) Einmaliges Retargeting ausführen
        retarget_frame(scene)

        self.report({'INFO'}, "Mapping applied and offsets reset")
        return {'FINISHED'}

class OT_AddBVHBone(Operator):
    bl_idname = "object.add_urdf_bone"
    bl_label = "Add URDF Bone"
    
    bvh_bone_name: StringProperty()

    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings

        for item in settings.mappings:
            if item.bvh_bone_name == self.bvh_bone_name:
                new_bone = item.urdf_bones.add()
                new_bone.bvh_bone_name = ""
                new_bone.source_axis = 'X'
                new_bone.sign = 'NEG'

        return {'FINISHED'}

class OT_RemoveBVHBone(Operator):
    bl_idname = "object.remove_urdf_bone"
    bl_label = "Remove URDF Bone"
    
    bvh_bone_name: StringProperty()
    index: IntProperty()
    
    def execute(self, context):
        scene = context.scene
        settings = scene.bvh_mapping_settings

        for item in settings.mappings:
            if item.bvh_bone_name == self.bvh_bone_name:
                if 0 <= self.index < len(item.urdf_bones):
                    item.urdf_bones.remove(self.index)

        return {'FINISHED'}

class OT_ResetBVHMapping(Operator):
    bl_idname = "object.reset_bvh_mapping"
    bl_label = "Reset Mapping List"
    
    def execute(self, context):
        context.scene.bvh_mapping_settings.mappings.clear()
        context.scene.bvh_mapping_settings.active_mapping_index = 0
        context.scene.bvh_mapping_settings.active_urdf_index = 0
        return {'FINISHED'}
    
class OT_DebugTPose(Operator):
    bl_idname = "object.debug_bvh_tpose"
    bl_label = "Toggle BVH T-Pose Preview"
    bl_description = "Show or hide a temporary T-pose preview without modifying the real BVH animation"

    def execute(self, context):
        scene = context.scene
        bvh_rig = scene.bvh_rig_object

        if not bvh_rig:
            self.report({'ERROR'}, "No BVH rig selected")
            return {'CANCELLED'}

        preview_name = bvh_rig.name + "_TPOSE_PREVIEW"
        existing = bpy.data.objects.get(preview_name)

        # ----------------------------------------------------
        # CASE 1: Preview exists → remove it + unhide original
        # ----------------------------------------------------
        if existing:
            bpy.data.objects.remove(existing, do_unlink=True)

            bvh_rig.hide_set(False)
            bvh_rig.hide_viewport = False
            bvh_rig.hide_render = False

            context.view_layer.update()

            self.report({'INFO'}, "T-pose preview removed")
            return {'FINISHED'}

        # ----------------------------------------------------
        # CASE 2: Preview does not exist → create it
        # ----------------------------------------------------

        # 1) Duplikat erzeugen
        collection = bvh_rig.users_collection[0] if bvh_rig.users_collection else context.scene.collection

        tpose_rig = bvh_rig.copy()
        tpose_rig.data = bvh_rig.data.copy()
        tpose_rig.animation_data_clear()
        tpose_rig.name = preview_name
        collection.objects.link(tpose_rig)

        # 2) Preview aktiv machen (entscheidend!)
        bpy.ops.object.select_all(action='DESELECT')
        tpose_rig.select_set(True)
        context.view_layer.objects.active = tpose_rig

        # 3) View-Layer refresh (Pose-Bones initialisieren)
        context.view_layer.update()

        # 4) Original ausblenden (erst jetzt!)
        bvh_rig.hide_set(True)
        bvh_rig.hide_viewport = True
        bvh_rig.hide_render = True

        # Preview-Rig global um 180° drehen (Z-Achse)
        tpose_rig.rotation_euler.z += math.radians(180)

        # 5) T-Pose anwenden
        for pb in tpose_rig.pose.bones:
            q = get_tpose_quaternion_for_bone(pb)
            pb.rotation_mode = 'XYZ'
            pb.rotation_euler = q.to_euler('XYZ')

        # 6) Pose-Modus erzwingen
        tpose_rig.data.pose_position = 'POSE'

        # 7) Finaler Refresh
        context.view_layer.update()

        self.report({'INFO'}, "T-pose preview created")
        return {'FINISHED'}

# ============================================================
# N-Panel UI
# ============================================================
class PANEL_BVHMapping(Panel):
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
        layout.prop(settings, "live_retarget")
        layout.prop(settings, "smoothing")

        row = layout.row()
        row.operator("object.generate_mapping_list", text="Generate Mapping List")
        row.operator("object.reset_bvh_mapping", text="Reset")

        layout.operator("object.apply_bvh_mapping", text="Apply Mapping")

        if scene.bvh_rig_object:
            preview_name = scene.bvh_rig_object.name + "_TPOSE_PREVIEW"
            exists = bpy.data.objects.get(preview_name) is not None
            label = "Hide BVH T-Pose" if exists else "Show BVH T-Pose"
        else:
            label = "Show BVH T-Pose"

        layout.operator(
            "object.debug_bvh_tpose",
            text=label,
            icon="ARMATURE_DATA"
        )

        if not settings.mappings:
            layout.label(text="Click 'Generate Mapping List' to show bones.")
            return

        box = layout.box()
        box.label(text="BVH Bones")
        box.template_list(
            "UL_BVHMappingList",
            "",
            settings,
            "mappings",
            settings,
            "active_mapping_index",
        )

        if 0 <= settings.active_mapping_index < len(settings.mappings):
            active_item = settings.mappings[settings.active_mapping_index]

            box2 = layout.box()
            box2.label(text=f"URDF Bones for: {active_item.bvh_bone_name}")

            row = box2.row()
            row.template_list(
                "UL_URDFBoneList",
                "",
                active_item,
                "urdf_bones",
                settings,
                "active_urdf_index",
            )

            col = row.column(align=True)
            add = col.operator("object.add_urdf_bone", text="", icon="ADD")
            add.bvh_bone_name = active_item.bvh_bone_name

            rem = col.operator("object.remove_urdf_bone", text="", icon="REMOVE")
            rem.bvh_bone_name = active_item.bvh_bone_name
            rem.index = settings.active_urdf_index

# ============================================================
# Import URDF Operator
# ============================================================
class IMPORT_OT_urdf_humanoid(Operator, ImportHelper):
    bl_idname = "import_scene.urdf_humanoid"
    bl_label = "Import URDF Humanoid"
    filename_ext = ".urdf"
    
    def execute(self, context):
        urdf_path = os.path.normpath(self.filepath)
        base_path = os.path.dirname(urdf_path)
        robot = parse_urdf(urdf_path)
        urdf_arm, link_mats = create_urdf_armature(robot)
        #bind_meshes(robot, urdf_arm, link_mats, base_path)
        context.scene.urdf_rig_object = urdf_arm
        return {'FINISHED'}

# ============================================================
# RETARGET
# ============================================================
@persistent
def retarget_frame(scene):
    settings = scene.bvh_mapping_settings
    if not getattr(settings, "live_retarget", False):
        return

    urdf_rig = scene.urdf_rig_object
    bvh_rig = scene.bvh_rig_object
    if not urdf_rig or not bvh_rig:
        return

    # Globaler Euler-Cache für Stabilisierung
    if "_prev_euler_cache" not in scene:
        scene["_prev_euler_cache"] = {}
    cache = scene["_prev_euler_cache"]

    for item in settings.mappings:
        bvh_b = bvh_rig.pose.bones.get(item.bvh_bone_name)
        if not bvh_b:
            continue

        # T-Pose Referenzrotation (Quaternion)
        ref_q = mathutils.Quaternion(item.ref_rot)
        if ref_q == mathutils.Quaternion((0,0,0,0)):
            continue

        # Aktuelle BVH-Rotation (Quaternion)
        cur_q = bvh_b.matrix_basis.to_quaternion()

        # Bewegung relativ zur T-Pose
        rot_bvh = ref_q.inverted() @ cur_q

        # Stabilisierte Euler-Winkel NUR zur Achsenextraktion
        e = rot_bvh.to_euler('XYZ')

        bone_key = f"{bvh_rig.name}:{bvh_b.name}"
        if bone_key in cache:
            e.make_compatible(cache[bone_key])
        cache[bone_key] = e.copy()

        # Achsenextraktion
        def get_axis(euler, axis_char):
            if axis_char == 'X':
                return euler.x
            if axis_char == 'Y':
                return euler.y
            if axis_char == 'Z':
                return euler.z
            return 0.0

        for b in item.urdf_bones:
            urdf_b = urdf_rig.pose.bones.get(b.bvh_bone_name)
            if not urdf_b:
                continue

            # UI-Achse auswählen
            value = get_axis(e, b.source_axis)

            # Vorzeichen aus UI
            if b.sign == 'NEG':
                value = -value

            # URDF-Offset anwenden
            key = "offset"
            if key not in urdf_b:
                urdf_b[key] = value
            value -= urdf_b[key]

            # Neutral-Offset aus UI (in Radiant)
            neutral = b.neutral_offset
            value += neutral

            # Zielrotation NUR um Y-Achse (Quaternion)
            target_q = mathutils.Euler((0.0, value, 0.0), 'XYZ').to_quaternion()

            # Glättung (Slerp)
            alpha = settings.smoothing
            old_q = urdf_b.rotation_quaternion.copy()
            smoothed_q = old_q.slerp(target_q, alpha)

            # Rotation setzen (Quaternion-only)
            urdf_b.rotation_mode = 'QUATERNION'
            urdf_b.rotation_quaternion = smoothed_q

# ============================================================
# REGISTER
# ============================================================
classes = [
    URDFVisual,
    URDFLink,
    URDFJoint,
    URDFRobot,
    BVHMappingBone,
    BVHMappingItem,
    BVHMappingSettings,
    UL_BVHMappingList,
    UL_URDFBoneList,
    OT_GenerateMappingList,
    OT_AddBVHBone,
    OT_RemoveBVHBone,
    OT_ApplyBVHMapping,
    OT_ResetBVHMapping,
    OT_DebugTPose,
    PANEL_BVHMapping,
    IMPORT_OT_urdf_humanoid,
]

def menu_func_import(self, context):
    self.layout.operator(IMPORT_OT_urdf_humanoid.bl_idname, text="URDF Humanoid (.urdf)")

def register():
    for attr in ("bvh_mapping_settings", "urdf_rig_object", "bvh_rig_object"):
        if hasattr(bpy.types.Scene, attr):
            try:
                delattr(bpy.types.Scene, attr)
            except Exception:
                pass
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except RuntimeError:
            pass
    bpy.types.Scene.bvh_mapping_settings = PointerProperty(type=BVHMappingSettings)
    bpy.types.Scene.urdf_rig_object = PointerProperty(name="URDF Rig", type=bpy.types.Object)
    bpy.types.Scene.bvh_rig_object = PointerProperty(name="BVH Rig", type=bpy.types.Object)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)
    if retarget_frame not in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.append(retarget_frame)

def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    if retarget_frame in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.remove(retarget_frame)

    for attr in ("bvh_mapping_settings", "urdf_rig_object", "bvh_rig_object"):
        if hasattr(bpy.types.Scene, attr):
            try:
                delattr(bpy.types.Scene, attr)
            except Exception:
                pass

    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except RuntimeError:
            pass

if __name__ == "__main__":
    register()