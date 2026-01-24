# ============================================================
#  BLOCK 1 — HEADER, IMPORTS, UTILITIES, URDF PARSER
# ============================================================

bl_info = {
    "name": "URDF Robot Importer (Humanoid Meta-Rig v3.1.0)",
    "author": "Dominik + Copilot",
    "version": (3, 1, 0),
    "blender": (3, 0, 0),
    "location": "File > Import > URDF Robot (Humanoid v3.1.0)",
    "description": "URDF importer with humanoid Meta-Rig and user-defined mapping",
    "category": "Import-Export",
}

import bpy
import os
import re
import unicodedata
import mathutils
from bpy_extras.io_utils import ImportHelper
import xml.etree.ElementTree as ET
from pathlib import Path

# ---------- safe_str ----------
def safe_str(s):
    if s is None:
        return ""
    if isinstance(s, str):
        try:
            s.encode("utf-8")
            return s
        except Exception:
            pass
    try:
        return s.decode("utf-8", errors="replace")
    except Exception:
        return str(s)

# ---------- sanitize_path ----------
def sanitize_path(path):
    path = safe_str(path)
    path = path.replace("\\", "/")
    try:
        path = unicodedata.normalize("NFKD", path)
    except Exception:
        pass
    path = path.encode("utf-8", errors="replace").decode("utf-8")
    path = re.sub(r"[^\w\-/\.\s:]", "_", path)
    return os.path.normpath(str(Path(path)))

# ---------- resolve mesh path ----------
def resolve_mesh_path(mesh, base_path):
    mesh = safe_str(mesh)
    if mesh.startswith("package://"):
        mesh = mesh.replace("package://", "")
        parts = mesh.split("/", 1)
        if len(parts) == 2:
            mesh = parts[1]
        return sanitize_path(os.path.join(base_path, mesh))
    return sanitize_path(os.path.join(base_path, mesh))

# ---------- URDF data structures ----------
class URDFVisual:
    def __init__(self, xyz, rpy, mesh):
        self.xyz = xyz
        self.rpy = rpy
        self.mesh = mesh

class URDFLink:
    def __init__(self, name):
        self.name = safe_str(name)
        self.generic_name = None
        self.visuals = []

class URDFJoint:
    def __init__(self, name, jtype, parent, child, xyz, rpy, axis, lower, upper):
        self.name = safe_str(name)
        self.generic_name = None
        self.type = jtype
        self.parent = safe_str(parent)
        self.child = safe_str(child)
        self.xyz = xyz
        self.rpy = rpy
        self.axis = axis
        self.lower = lower
        self.upper = upper

class URDFRobot:
    def __init__(self, name):
        self.name = safe_str(name)
        self.links = {}
        self.joints = {}

# ---------- parse float list ----------
def parse_float_list(s, n):
    if s is None:
        return [0.0] * n
    parts = safe_str(s).replace(",", " ").split()
    vals = []
    for p in parts:
        try:
            vals.append(float(p))
        except Exception:
            vals.append(0.0)
    while len(vals) < n:
        vals.append(0.0)
    return vals[:n]

# ---------- parse URDF ----------
def parse_urdf(path):
    tree = ET.parse(path)
    root = tree.getroot()

    robot = URDFRobot(root.attrib.get("name", "URDF_Robot"))

    # Links
    for link_el in root.findall("link"):
        link = URDFLink(link_el.attrib["name"])
        for vis_el in link_el.findall("visual"):
            org = vis_el.find("origin")
            xyz = parse_float_list(org.attrib.get("xyz"), 3) if org is not None else [0, 0, 0]
            rpy = parse_float_list(org.attrib.get("rpy"), 3) if org is not None else [0, 0, 0]
            geom = vis_el.find("geometry")
            mesh_el = geom.find("mesh") if geom is not None else None
            mesh = safe_str(mesh_el.attrib.get("filename")) if mesh_el is not None else None
            if mesh:
                link.visuals.append(URDFVisual(xyz, rpy, mesh))
        robot.links[link.name] = link

    # Joints
    for idx, joint_el in enumerate(root.findall("joint")):
        name = joint_el.attrib.get("name", "")
        jtype = joint_el.attrib.get("type", "fixed")
        parent = joint_el.find("parent").attrib["link"]
        child = joint_el.find("child").attrib["link"]
        org = joint_el.find("origin")
        xyz = parse_float_list(org.attrib.get("xyz"), 3) if org is not None else [0, 0, 0]
        rpy = parse_float_list(org.attrib.get("rpy"), 3) if org is not None else [0, 0, 0]
        axis_el = joint_el.find("axis")
        axis = parse_float_list(axis_el.attrib.get("xyz"), 3) if axis_el is not None else [0, 0, 1]
        limit_el = joint_el.find("limit")
        lower = float(limit_el.attrib.get("lower", "0")) if limit_el is not None else 0.0
        upper = float(limit_el.attrib.get("upper", "0")) if limit_el is not None else 0.0

        j = URDFJoint(name, jtype, parent, child, xyz, rpy, axis, lower, upper)
        j.generic_name = f"joint_{idx:03d}"
        robot.joints[name] = j

    # Assign generic link names
    for idx, (orig, link) in enumerate(robot.links.items()):
        link.generic_name = f"link_{idx:03d}"

    # Remap parent/child to generic link names
    link_map = {orig: link.generic_name for orig, link in robot.links.items()}
    for j in robot.joints.values():
        j.parent = link_map[j.parent]
        j.child = link_map[j.child]

    return robot

# ============================================================
#  BLOCK 2 — URDF ARMATURE + MESH BINDING
# ============================================================

def origin_to_matrix(xyz, rpy):
    loc = mathutils.Vector(xyz)
    rot = mathutils.Euler((rpy[0], rpy[1], rpy[2]), 'XYZ').to_matrix().to_4x4()
    return mathutils.Matrix.Translation(loc) @ rot

def axis_to_vec(axis):
    v = mathutils.Vector(axis)
    return v.normalized() if v.length > 0 else mathutils.Vector((0, 0, 1))

def build_link_transforms(robot):
    children = {}
    joint_by_child = {}
    for j in robot.joints.values():
        children.setdefault(j.parent, []).append(j)
        joint_by_child[j.child] = j

    all_links = set(l.generic_name for l in robot.links.values())
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

    # Create bones
    for j in robot.joints.values():
        b = bones.new(j.generic_name)
        link_to_bone[j.child] = b

    # Position + parenting
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

    # Store original URDF names
    for j in robot.joints.values():
        pb = arm_obj.pose.bones.get(j.generic_name)
        if pb:
            pb["urdf_joint_name"] = j.name

    # ------------------------------------------------------------
    # PATCH: URDF-Rig Restpose einfrieren
    # ------------------------------------------------------------
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode='POSE')

    for pb in arm_obj.pose.bones:
        pb.rotation_mode = 'XYZ'
        pb.rotation_euler = (0.0, 0.0, 0.0)
        pb.location = (0.0, 0.0, 0.0)
        pb.scale = (1.0, 1.0, 1.0)

    bpy.ops.pose.armature_apply()
    bpy.ops.object.mode_set(mode='OBJECT')

    return arm_obj, link_mats

def import_mesh(mesh_path):
    mesh_path = sanitize_path(mesh_path)
    ext = os.path.splitext(mesh_path)[1].lower()
    if ext == ".stl":
        bpy.ops.wm.stl_import(filepath=mesh_path)
    elif ext == ".obj":
        bpy.ops.import_scene.obj(filepath=mesh_path)
    elif ext == ".dae":
        bpy.ops.wm.collada_import(filepath=mesh_path)
    else:
        print(f"[URDF Import] Unsupported mesh: {mesh_path}")
        return None
    return bpy.context.selected_objects[0]

def bind_meshes(robot, arm_obj, link_mats, base_path):
    bpy.ops.object.mode_set(mode='OBJECT')

    for orig_name, link in robot.links.items():
        gname = link.generic_name
        link_mat = link_mats.get(gname, mathutils.Matrix.Identity(4))

        for idx, vis in enumerate(link.visuals):
            mesh_path = resolve_mesh_path(vis.mesh, base_path)
            if not os.path.isfile(mesh_path):
                print(f"[URDF Import] Missing mesh: {mesh_path}")
                continue

            obj = import_mesh(mesh_path)
            if not obj:
                continue

            obj.name = safe_str(orig_name if idx == 0 else f"{orig_name}_{idx}")

            vis_mat = origin_to_matrix(vis.xyz, vis.rpy)
            obj.matrix_world = link_mat @ vis_mat

            obj.parent = arm_obj
            obj.parent_type = 'OBJECT'

# ============================================================
#  BLOCK 3 — FINAL HUMANOID META-RIG MIT KORRIGIERTEN CONSTRAINTS
# ============================================================

URDF_TO_HUMANOID = {
    # Kopf
    "AAHead_yaw": "Head",
    "Head_pitch": "Head",

    # Linker Arm
    "ALeft_Shoulder_Pitch": "LeftArm",
    "Left_Shoulder_Roll": "LeftArm",
    "Left_Elbow_Pitch": "LeftForeArm",
    "Left_Elbow_Yaw": "LeftForeArm",

    # Rechter Arm
    "ARight_Shoulder_Pitch": "RightArm",
    "Right_Shoulder_Roll": "RightArm",
    "Right_Elbow_Pitch": "RightForeArm",
    "Right_Elbow_Yaw": "RightForeArm",

    # Linkes Bein
    "Left_Hip_Pitch": "LeftUpLeg",
    "Left_Hip_Roll": "LeftUpLeg",
    "Left_Hip_Yaw": "LeftUpLeg",
    "Left_Knee_Pitch": "LeftLeg",
    "Left_Ankle_Pitch": "LeftFoot",
    "Left_Ankle_Roll": "LeftFoot",

    # Rechtes Bein
    "Right_Hip_Pitch": "RightUpLeg",
    "Right_Hip_Roll": "RightUpLeg",
    "Right_Hip_Yaw": "RightUpLeg",
    "Right_Knee_Pitch": "RightLeg",
    "Right_Ankle_Pitch": "RightFoot",
    "Right_Ankle_Roll": "RightFoot"
}

# ------------------------------------------------------------
# Hilfsfunktionen
# ------------------------------------------------------------
def avg_pos(arm, names):
    pts = []
    for n in names:
        pb = arm.pose.bones.get(n)
        if pb:
            pts.append(pb.head)
    if not pts:
        return mathutils.Vector((0,0,0))
    return sum(pts, mathutils.Vector((0,0,0))) / len(pts)

def create_humanoid_meta_rig(robot, urdf_arm_obj):
    """
    Erzeugt die humanoide Meta-Rig Armature basierend auf den URDF-Bones
    und verbindet die URDF-Bones über CHILD_OF Constraints.
    """
    urdf_to_generic = {j.name: j.generic_name for j in robot.joints.values()}

    humanoid_map = {}
    for urdf_name, humanoid_bone in URDF_TO_HUMANOID.items():
        if urdf_name in urdf_to_generic:
            humanoid_map.setdefault(humanoid_bone, []).append(urdf_to_generic[urdf_name])

    # Durchschnittspositionen der URDF-Bones auslesen
    def avg_pos(names):
        pts = []
        for n in names:
            pb = urdf_arm_obj.pose.bones.get(n)
            if pb:
                pts.append(pb.head)
        if pts:
            return sum(pts, mathutils.Vector((0,0,0))) / len(pts)
        else:
            return mathutils.Vector((0,0,0))

    # Schulter / Ellbogen / Hüfte / Knie / Fuß Positionen
    left_shoulder  = avg_pos(humanoid_map.get("LeftArm", []))
    right_shoulder = avg_pos(humanoid_map.get("RightArm", []))
    left_elbow     = avg_pos(humanoid_map.get("LeftForeArm", []))
    right_elbow    = avg_pos(humanoid_map.get("RightForeArm", []))
    left_hip       = avg_pos(humanoid_map.get("LeftUpLeg", []))
    right_hip      = avg_pos(humanoid_map.get("RightUpLeg", []))
    left_knee      = avg_pos(humanoid_map.get("LeftLeg", []))
    right_knee     = avg_pos(humanoid_map.get("RightLeg", []))
    left_foot      = avg_pos(humanoid_map.get("LeftFoot", []))
    right_foot     = avg_pos(humanoid_map.get("RightFoot", []))

    # Erstellen der Meta-Rig Armature
    meta_arm = bpy.data.armatures.new(f"{robot.name}_Humanoid_Armature")
    meta_obj = bpy.data.objects.new(f"{robot.name}_Humanoid_Meta_Rig", meta_arm)
    if urdf_arm_obj.users_collection:
        urdf_arm_obj.users_collection[0].objects.link(meta_obj)
    else:
        bpy.context.scene.collection.objects.link(meta_obj)

    bpy.context.view_layer.objects.active = meta_obj
    bpy.ops.object.mode_set(mode='EDIT')
    mb = meta_arm.edit_bones

    # Wirbelsäule
    hips = mb.new("Hips")
    hips.head = (left_hip + right_hip)/2
    hips.tail = hips.head + mathutils.Vector((0,0,0.15))

    spine = mb.new("Spine")
    spine.head = hips.tail
    spine.tail = spine.head + mathutils.Vector((0,0,0.15))
    spine.parent = hips

    # Kopf
    neck = mb.new("Neck")
    neck.head = spine.tail
    neck.tail = neck.head + mathutils.Vector((0,0,0.12))
    neck.parent = spine

    head = mb.new("Head")
    head.head = neck.tail
    head.tail = head.head + mathutils.Vector((0,0,0.18))
    head.parent = neck

    # Arme
    def make_arm(side, shoulder, elbow):
        if shoulder.length == 0 or elbow.length == 0:
            return
        sgn = 1 if side == "Right" else -1
        arm = mb.new(f"{side}Arm")
        arm.head = shoulder
        arm.tail = elbow
        arm.parent = spine
        fore = mb.new(f"{side}ForeArm")
        fore.head = elbow
        fore.tail = elbow + mathutils.Vector((sgn*0.2,0,0))
        fore.parent = arm

    make_arm("Left", left_shoulder, left_elbow)
    make_arm("Right", right_shoulder, right_elbow)

    # Beine
    def make_leg(side, hip, knee, foot):
        if hip.length == 0 or knee.length == 0 or foot.length == 0:
            return
        up = mb.new(f"{side}UpLeg")
        up.head = hip
        up.tail = knee
        up.parent = hips
        lo = mb.new(f"{side}Leg")
        lo.head = knee
        lo.tail = foot
        lo.parent = up
        foot_b = mb.new(f"{side}Foot")
        foot_b.head = foot
        foot_b.tail = foot + mathutils.Vector((0.12, 0, 0))
        foot_b.parent = lo

    make_leg("Left", left_hip, left_knee, left_foot)
    make_leg("Right", right_hip, right_knee, right_foot)

    # Ensure the right foot is correctly connected to the spine
    right_foot_bone = mb.new("RightFoot")
    right_foot_bone.head = right_foot
    right_foot_bone.tail = right_foot + mathutils.Vector((0.12, 0, 0))
    right_foot_bone.parent = spine

    bpy.ops.object.mode_set(mode='POSE')

    # Constraints: URDF -> Meta-Rig
    for meta_name, urdf_bones in humanoid_map.items():
        mbone = meta_obj.pose.bones.get(meta_name)
        if not mbone:
            continue
        for urdf_b in urdf_bones:
            upb = urdf_arm_obj.pose.bones.get(urdf_b)
            if not upb:
                continue
            c = upb.constraints.new("CHILD_OF")
            c.target = meta_obj
            c.subtarget = meta_name
            bpy.context.view_layer.update()
            c.inverse_matrix = upb.matrix.inverted()

    bpy.ops.object.mode_set(mode='OBJECT')
    return meta_obj

# ============================================================
#  BLOCK 4 — OPERATOR + REGISTER
# ============================================================

class IMPORT_OT_urdf_humanoid_v310(bpy.types.Operator, ImportHelper):
    bl_idname = "import_scene.urdf_robot_humanoid_v310"
    bl_label = "Import URDF Robot (Humanoid v3.1.0)"
    filename_ext = ".urdf"

    def execute(self, context):
        urdf_path = sanitize_path(self.filepath)
        base_path = sanitize_path(os.path.dirname(urdf_path))

        robot = parse_urdf(urdf_path)
        urdf_arm_obj, link_mats = create_urdf_armature(robot)
        meta_obj = create_humanoid_meta_rig(robot, urdf_arm_obj)
        bind_meshes(robot, urdf_arm_obj, link_mats, base_path)

        urdf_arm_obj.select_set(True)
        if meta_obj:
            meta_obj.select_set(True)
            bpy.context.view_layer.objects.active = meta_obj

        return {'FINISHED'}

def menu_func_import(self, context):
    self.layout.operator(
        IMPORT_OT_urdf_humanoid_v310.bl_idname,
        text="URDF Robot (Humanoid v3.1.0) (.urdf)"
    )

def register():
    bpy.utils.register_class(IMPORT_OT_urdf_humanoid_v310)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)

def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    bpy.utils.unregister_class(IMPORT_OT_urdf_humanoid_v310)

if __name__ == "__main__":
    register()