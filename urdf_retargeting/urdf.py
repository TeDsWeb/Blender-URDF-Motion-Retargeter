"""
URDF data structures and parsing functionality.

Provides classes for representing URDF robots, links, and joints,
along with XML parsing functionality to load URDF files.
"""

import xml.etree.ElementTree as ET
from typing import Optional

from .utils import parse_float_list


class URDFVisual:
    """Represents a visual element (mesh) in a URDF link, including material.

    Stores transform (origin), mesh filename and optional material properties
    such as a material name, color RGBA and a texture filename.
    """

    def __init__(
        self,
        xyz: list[float],
        rpy: list[float],
        mesh: Optional[str],
        material_name: Optional[str] = None,
        color: Optional[list[float]] = None,
        texture: Optional[str] = None,
    ):
        """
        Initialize a URDF visual element.

        Args:
            xyz: Translation of the visual [x, y, z].
            rpy: Rotation of the visual (roll, pitch, yaw) in radians.
            mesh: Path to the mesh file (may be None).
            material_name: Optional name of the material from URDF.
            color: Optional RGBA color list [r,g,b,a].
            texture: Optional texture filename.
        """
        self.xyz = xyz
        self.rpy = rpy
        self.mesh = mesh
        self.material_name = material_name
        self.color = color
        self.texture = texture


class URDFLink:
    """Represents a rigid link in a URDF robot."""

    def __init__(self, name: str):
        """
        Initialize a URDF link.

        Args:
            name: The unique identifier for this link.
        """
        self.name = name
        self.visuals = []


class URDFJoint:
    """Represents a joint connecting two links in a URDF robot."""

    def __init__(
        self,
        name: str,
        jtype: str,
        parent: str,
        child: str,
        xyz: list[float],
        rpy: list[float],
        axis: list[float],
        limit_lower: float = None,
        limit_upper: float = None,
        velocity_limit: float = None,
        soft_limit_lower: float = None,
        soft_limit_upper: float = None,
    ):
        """
        Initialize a URDF joint.

        Args:
            name: The unique identifier for this joint.
            jtype: The joint type (e.g., 'revolute', 'fixed', 'continuous').
            parent: Name of the parent link.
            child: Name of the child link.
            xyz: Translation of the joint origin [x, y, z].
            rpy: Rotation of the joint origin (roll, pitch, yaw) in radians.
            axis: The rotation axis in the joint's local frame [x, y, z].
            limit_lower: Lower rotation limit in radians (for revolute joints).
            limit_upper: Upper rotation limit in radians (for revolute joints).
            velocity_limit: Maximum angular velocity in rad/s.
            soft_limit_lower: Soft lower limit (used if present).
            soft_limit_upper: Soft upper limit (used if present).
        """
        self.name = name
        self.type = jtype
        self.parent = parent
        self.child = child
        self.xyz = xyz
        self.rpy = rpy
        self.axis = axis
        self.limit_lower = limit_lower
        self.limit_upper = limit_upper
        self.velocity_limit = velocity_limit
        self.soft_limit_lower = soft_limit_lower
        self.soft_limit_upper = soft_limit_upper


class URDFRobot:
    """Represents a complete URDF robot model."""

    def __init__(self, name: str):
        """
        Initialize a URDF robot.

        Args:
            name: The name of the robot.
        """
        self.name = name
        self.links = {}  # link_name -> URDFLink
        self.joints = {}  # joint_name -> URDFJoint


def parse_urdf(path: str) -> URDFRobot:
    """
    Parse a URDF XML file into a URDFRobot data structure.

    Extracts all links (with their visual geometries) and joints (with their
    kinematics constraints) from the URDF file.

    Args:
        path: Absolute path to the URDF XML file.

    Returns:
        A URDFRobot object containing all parsed data.

    Raises:
        ET.ParseError: If the XML file is malformed.
        FileNotFoundError: If the URDF file doesn't exist.
    """
    tree = ET.parse(path)
    root = tree.getroot()
    robot = URDFRobot(root.attrib.get("name", "URDF_Robot"))

    # Parse links and their visual elements
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

            # Parse material (optional) - color RGBA and texture filename
            material_name = None
            color = None
            texture = None
            material_el = vis_el.find("material")
            if material_el is not None:
                material_name = material_el.attrib.get("name")
                color_el = material_el.find("color")
                if color_el is not None and "rgba" in color_el.attrib:
                    color = parse_float_list(color_el.attrib.get("rgba"), 4)
                    print(f"Parsed color for material '{material_name}': {color}")
                texture_el = material_el.find("texture")
                if texture_el is not None:
                    texture = texture_el.attrib.get("filename")

            if mesh:
                link.visuals.append(
                    URDFVisual(xyz, rpy, mesh, material_name, color, texture)
                )

        robot.links[link.name] = link

    # Parse joints and their constraints
    for idx, joint_el in enumerate(root.findall("joint")):
        # Use the actual 'name' attribute from the URDF joint element
        name = joint_el.attrib.get("name")
        if not name:
            name = f"joint_{idx:03d}"  # Fallback only if attribute is missing

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

        # Parse joint limits
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
        velocity = (
            float(limit_el.get("velocity", 10.0)) if limit_el is not None else 10.0
        )

        # Parse safety controller soft limits
        safety_el = joint_el.find("safety_controller")
        soft_limit_lower = (
            float(safety_el.attrib.get("soft_lower_limit"))
            if safety_el is not None and "soft_lower_limit" in safety_el.attrib
            else None
        )
        soft_limit_upper = (
            float(safety_el.attrib.get("soft_upper_limit"))
            if safety_el is not None and "soft_upper_limit" in safety_el.attrib
            else None
        )

        robot.joints[name] = URDFJoint(
            name,
            jtype,
            parent,
            child,
            xyz,
            rpy,
            axis,
            lower,
            upper,
            velocity,
            soft_limit_lower,
            soft_limit_upper,
        )

    return robot
