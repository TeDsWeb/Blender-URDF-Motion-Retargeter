"""
Blender armature creation and mesh binding functionality.

Handles the creation of Blender armatures from URDF data and binding
of mesh geometry to skeletal bones.
"""

import bpy
import os
import mathutils
from .urdf import URDFRobot
from .utils import origin_to_matrix


def build_link_transforms(robot: URDFRobot) -> tuple[dict, str]:
    """
    Compute world-space transformation matrices for all links in their default pose.

    Performs a depth-first traversal of the kinematic tree to calculate the
    accumulated transformations to each link.

    Args:
        robot: The URDFRobot object containing joint and link data.

    Returns:
        Tuple of (matrices_dict, root_link_name) where:
        - matrices_dict: A dictionary mapping link names to their 4x4 world matrices
        - root_link_name: The name of the root link (has no parent joint)
    """
    # Build parent-child relationships
    children = {}
    joint_by_child = {}

    for j in robot.joints.values():
        children.setdefault(j.parent, []).append(j)
        joint_by_child[j.child] = j

    # Identify root link(s): links with no parent joint
    all_links = set(l.name for l in robot.links.values())
    roots = list(all_links - set(joint_by_child.keys()))
    root = roots[0] if roots else list(all_links)[0]

    # Depth-first traversal to accumulate transforms
    mats = {root: mathutils.Matrix.Identity(4)}
    stack = [root]

    while stack:
        parent_link = stack.pop()

        for joint in children.get(parent_link, []):
            # Accumulate parent's world transform with joint's local transform
            mats[joint.child] = mats[parent_link] @ origin_to_matrix(
                joint.xyz, joint.rpy
            )
            stack.append(joint.child)

    return mats, root


def create_urdf_armature(robot: URDFRobot) -> tuple[bpy.types.Object, dict]:
    """
    Create a Blender armature with bones named after URDF joints.

    Creates a hierarchical bone structure matching the URDF kinematic tree,
    stores joint limits and types as bone custom properties, and establishes
    the link-to-bone mapping for mesh binding.

    Args:
        robot: The URDFRobot object to convert into an armature.

    Returns:
        Tuple of (armature_object, link_transform_matrices) where:
        - armature_object: The created Blender armature object
        - link_transform_matrices: Dictionary of link world-space matrices used for mesh placement
    """
    link_mats, root_link_name = build_link_transforms(robot)

    # Create the armature object
    bpy.ops.object.add(type="ARMATURE", enter_editmode=True)
    arm_obj = bpy.context.object
    arm_obj.name = f"{robot.name}_Rig"
    bones = arm_obj.data.edit_bones

    # Create link-to-bone mapping (required because meshes belong to links, bones are named after joints)
    link_to_bone_name = {root_link_name: root_link_name}
    bones.new(root_link_name)  # Root bone for the base

    # Create bones for each joint
    for j_name in robot.joints.keys():
        bones.new(j_name)

    # Update link-to-bone mapping for all joints
    for j_name, j in robot.joints.items():
        link_to_bone_name[j.child] = j_name

    # Position bones and establish hierarchy
    for j in robot.joints.values():
        b = bones.get(j.name)
        if b:
            # Set bone position based on the child link's world transform
            head = link_mats[j.child].to_translation()
            joint_rot = mathutils.Euler(j.rpy, "XYZ").to_matrix()
            axis_world = (
                link_mats[j.parent].to_3x3() @ joint_rot @ mathutils.Vector(j.axis)
            ).normalized()

            b.head, b.tail = head, head + axis_world * 0.05

            # Parent to the bone that controls the parent link
            p_bone_name = link_to_bone_name.get(j.parent)
            if p_bone_name:
                b.parent = bones.get(p_bone_name)

    bpy.ops.object.mode_set(mode="OBJECT")

    # Store the mapping inside the object for later use
    arm_obj["_link_to_bone"] = link_to_bone_name

    # Store joint constraints as bone custom properties
    for j_name, j in robot.joints.items():
        if j_name in arm_obj.pose.bones:
            pb = arm_obj.pose.bones[j_name]

            # Store joint limits (prioritize soft limits if available)
            if j.soft_limit_lower is not None:
                pb["limit_lower"] = j.soft_limit_lower
            elif j.limit_lower is not None:
                pb["limit_lower"] = j.limit_lower
            else:
                pb["limit_lower"] = -3.14159

            if j.soft_limit_upper is not None:
                pb["limit_upper"] = j.soft_limit_upper
            elif j.limit_upper is not None:
                pb["limit_upper"] = j.limit_upper
            else:
                pb["limit_upper"] = 3.14159

            # Store velocity limit
            pb["velocity_limit"] = (
                j.velocity_limit if j.velocity_limit is not None else 10.0
            )

            # Store joint type
            pb["joint_type"] = j.type if j.type is not None else "fixed"

    return arm_obj, link_mats


def bind_meshes(
    robot: URDFRobot, arm_obj: bpy.types.Object, link_mats: dict, base_path: str
) -> None:
    """
    Import and bind mesh files to the armature bones.

    Loads mesh files (STL, OBJ, DAE) referenced in the URDF and parents them
    to the appropriate bones, using the link-to-bone mapping and stored
    transformation matrices.

    Args:
        robot: The URDFRobot object containing link and visual data.
        arm_obj: The armature object to bind meshes to.
        link_mats: Dictionary of link transformation matrices (world space).
        base_path: The base directory for resolving mesh file paths.
    """
    # Retrieve the mapping from the armature object
    link_to_bone_name = arm_obj.get("_link_to_bone", {})

    for link_name, link in robot.links.items():
        bone_name = link_to_bone_name.get(link_name)

        for idx, vis in enumerate(link.visuals):
            mesh_raw = vis.mesh.replace("package://", "")
            mesh_path = os.path.normpath(os.path.join(base_path, mesh_raw))

            if not os.path.exists(mesh_path):
                continue

            # Deselect to ensure the imported object is the only selection
            bpy.ops.object.select_all(action="DESELECT")

            # Import based on file extension
            ext = os.path.splitext(mesh_path)[1].lower()
            if ext == ".stl":
                bpy.ops.wm.stl_import(filepath=mesh_path)
            elif ext == ".obj":
                bpy.ops.wm.obj_import(filepath=mesh_path)
            elif ext == ".dae":
                bpy.ops.wm.collada_import(filepath=mesh_path)
            else:
                continue

            if bpy.context.selected_objects:
                obj = bpy.context.selected_objects[0]

                # Safety check: avoid parenting the armature to itself
                if obj == arm_obj:
                    continue

                obj.name = f"{link_name}_mesh_{idx}"
                obj.parent = arm_obj

                # Parent to bone using the joint name from the mapping
                if bone_name and bone_name in arm_obj.pose.bones:
                    obj.parent_type, obj.parent_bone = "BONE", bone_name
                    bone_mtx_world = (
                        arm_obj.matrix_world @ arm_obj.pose.bones[bone_name].matrix
                    )
                    obj.matrix_parent_inverse = bone_mtx_world.inverted()

                # Apply the visual's origin transformation
                obj.matrix_world = link_mats[link_name] @ origin_to_matrix(
                    vis.xyz, vis.rpy
                )

                # Apply material/color/texture if provided in URDFVisual
                try:
                    mat_name = (
                        vis.material_name
                        if vis.material_name
                        else f"{link_name}_mat_{idx}"
                    )

                    # Create or reuse a material
                    mat = bpy.data.materials.get(mat_name)
                    if mat is None:
                        mat = bpy.data.materials.new(mat_name)

                    mat.use_nodes = True
                    nodes = mat.node_tree.nodes
                    links = mat.node_tree.links

                    # Ensure Principled BSDF exists
                    bsdf = nodes.get("Principled BSDF")
                    if bsdf is None:
                        bsdf = nodes.new(type="ShaderNodeBsdfPrincipled")

                    # Apply color (RGBA) if present
                    if vis.color is not None and len(vis.color) >= 3:
                        # URDF color values are typically 0..1
                        r, g, b = vis.color[0], vis.color[1], vis.color[2]
                        a = vis.color[3] if len(vis.color) > 3 else 1.0
                        bsdf.inputs["Base Color"].default_value = (r, g, b, 1.0)
                        bsdf.inputs["Alpha"].default_value = a
                        if a < 1.0:
                            mat.blend_method = "BLEND"

                    # Apply texture if present and available on disk
                    if vis.texture:
                        tex_raw = vis.texture.replace("package://", "")
                        tex_path = os.path.normpath(os.path.join(base_path, tex_raw))
                        if os.path.exists(tex_path):
                            try:
                                img = bpy.data.images.load(tex_path)
                                tex_node = nodes.new(type="ShaderNodeTexImage")
                                tex_node.image = img
                                # Connect texture color to Base Color
                                links.new(
                                    tex_node.outputs["Color"], bsdf.inputs["Base Color"]
                                )
                            except Exception:
                                # Loading image failed; ignore and continue
                                pass

                    # Assign material to object (replace first slot)
                    if obj.data.materials:
                        obj.data.materials[0] = mat
                    else:
                        obj.data.materials.append(mat)
                except Exception:
                    # Don't fail the whole import for material issues
                    pass
