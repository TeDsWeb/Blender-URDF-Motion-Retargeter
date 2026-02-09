"""
Blender URDF Retargeting Addon.

A comprehensive addon for importing URDF robots and retargeting BVH motion capture
data onto URDF-defined robot kinematics. Features include:

- URDF parsing with mesh binding
- BVH-to-URDF bone mapping with multi-axis control
- Real-time motion retargeting with root projection
- Zero-lag motion smoothing with bidirectional filtering
- Foot contact detection and dynamic foot planting
- Anti-sinking ground alignment
- Export to Beyond Mimic CSV/JSON trajectory format

Author: Dominik Brämer
Version: 1.0.0
Blender: 5.0+
"""

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
from bpy.types import Scene
from bpy.props import PointerProperty

# Import modules and register classes
from .data_structures import BVHMappingBone, BVHMappingItem, BVHMappingSettings
from .ui import (
    UL_BVHMappingList,
    UL_URDFBoneList,
    PANEL_RigSelection,
    PANEL_MotionOptions,
    PANEL_FootConfiguration,
    PANEL_BoneMapping,
    PANEL_ApplyAndExport,
)
from .operators import (
    OT_GenerateMappingList,
    OT_ApplyBVHMapping,
    OT_CalibrateRestPose,
    OT_AddBVHBone,
    OT_RemoveBVHBone,
    OT_ClearScene,
    IMPORT_OT_urdf_humanoid,
    menu_func_import,
)
from .export import OT_ExportBeyondMimic
from .retargeting import retarget_frame


# All classes to register
classes = [
    # Data structures
    BVHMappingBone,
    BVHMappingItem,
    BVHMappingSettings,
    # UI
    UL_BVHMappingList,
    UL_URDFBoneList,
    PANEL_RigSelection,
    PANEL_MotionOptions,
    PANEL_FootConfiguration,
    PANEL_BoneMapping,
    PANEL_ApplyAndExport,
    # Operators
    OT_GenerateMappingList,
    OT_ApplyBVHMapping,
    OT_CalibrateRestPose,
    OT_AddBVHBone,
    OT_RemoveBVHBone,
    OT_ClearScene,
    IMPORT_OT_urdf_humanoid,
    OT_ExportBeyondMimic,
]


def register():
    """Register addon classes and scene properties."""
    # Register classes
    for cls in classes:
        bpy.utils.register_class(cls)

    # Register scene properties
    Scene.bvh_mapping_settings = PointerProperty(type=BVHMappingSettings)
    Scene.urdf_rig_object = PointerProperty(type=bpy.types.Object)
    Scene.bvh_rig_object = PointerProperty(type=bpy.types.Object)
    Scene.smoothed_bvh_rig_object = PointerProperty(type=bpy.types.Object)

    # Add menu entry
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)

    # Register frame change handler
    bpy.app.handlers.frame_change_post.append(retarget_frame)


def unregister():
    """Unregister addon classes and scene properties."""
    # Remove menu entry
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)

    # Remove frame handler
    bpy.app.handlers.frame_change_post.remove(retarget_frame)

    # Unregister classes
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
