"""Assemble the articulated GP25 glTF asset from per-link meshes.

Run with Blender headless (Blender 4.x / 5.x):

    blender --background --python blender_build_gp25.py -- <links_dir> <out.glb>

Builds the kinematic node hierarchy with each joint node placed on its real
rotation axis (origins extracted from the CAD), assigns the Yaskawa-orange and
machined-steel PBR materials, decimates for real-time use, and exports a GLB in
the native Y-up, metre frame.

Joint origins and axes are the kinematic ground truth; they are mirrored in
assets/robots/yaskawa_gp25/gp25.kinematics.json and
libs/visualization/include/cavr/visualization/robot_model.hpp.
"""

import math
import os
import sys

import bpy
from mathutils import Vector, Matrix

MM = 0.001
COLLAPSE = 0.8       # collapse-decimation ratio (mild; preserves bolt detail)

LINK_FILES = [
    ("00_base", "link_base"), ("01_s", "link_s"), ("02_l", "link_l"),
    ("03_u", "link_u"), ("04_r", "link_r"), ("05_b", "link_b"), ("06_t", "link_t"),
]

# joint world origins in the native CAD frame (mm, Y-up)
JOINTS_MM = {
    "joint_s": (0.0, 169.0, 0.0),
    "joint_l": (-157.0, 505.0, 150.0),
    "joint_u": (0.0, 1265.0, 150.0),
    "joint_r": (0.0, 1465.0, 452.0),
    "joint_b": (0.0, 1465.0, 945.0),
    "joint_t": (0.0, 1465.0, 945.0),
}
TCP_MM = (0.0, 1465.0, 1046.0)

# kinematic chain: child -> parent node name
CHAIN = [
    ("joint_s", "link_base"), ("link_s", "joint_s"),
    ("joint_l", "joint_s"), ("link_l", "joint_l"),
    ("joint_u", "joint_l"), ("link_u", "joint_u"),
    ("joint_r", "joint_u"), ("link_r", "joint_r"),
    ("joint_b", "joint_r"), ("link_b", "joint_b"),
    ("joint_t", "joint_b"), ("link_t", "joint_t"),
    ("tcp", "joint_t"),
]

# industrial-orange paint on the arm castings; cast metal base; steel tool flange
ORANGE_LINKS = ["link_s", "link_l", "link_u", "link_r", "link_b"]
CAST_LINKS = ["link_base"]
STEEL_LINKS = ["link_t"]


def v_m(t):
    return Vector((t[0] * MM, t[1] * MM, t[2] * MM))


def srgb_to_lin(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def make_mat(name, srgb, metallic, roughness):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    lin = [srgb_to_lin(c) for c in srgb]
    b.inputs["Base Color"].default_value = (lin[0], lin[1], lin[2], 1.0)
    b.inputs["Metallic"].default_value = metallic
    b.inputs["Roughness"].default_value = roughness
    return m


def build(links_dir, out_glb):
    bpy.ops.wm.read_factory_settings(use_empty=True)

    nodes = {}
    for fname, node in LINK_FILES:
        path = os.path.join(links_dir, fname + ".obj")
        before = set(bpy.data.objects)
        # forward=Y, up=Z => identity import: keep the native CAD coordinates
        bpy.ops.wm.obj_import(filepath=path, up_axis='Z', forward_axis='Y')
        new = list(set(bpy.data.objects) - before)
        obj = new[0]
        if len(new) > 1:
            bpy.ops.object.select_all(action='DESELECT')
            for o in new:
                o.select_set(True)
            bpy.context.view_layer.objects.active = new[0]
            bpy.ops.object.join()
            obj = bpy.context.view_layer.objects.active
        obj.name = node
        obj.data.name = node + "_mesh"
        nodes[node] = obj

    # scale mm -> m
    bpy.ops.object.select_all(action='DESELECT')
    for obj in nodes.values():
        obj.select_set(True)
        obj.scale = (MM, MM, MM)
    bpy.context.view_layer.objects.active = nodes["link_base"]
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    # Weld unwelded import vertices + recalc normals BEFORE decimating. FreeCAD
    # tessellation emits duplicate vertices along shared edges; collapsing such a
    # mesh tears small features (bolt bosses) into spikes. Merge-by-distance plus
    # consistent normals is the canonical fix.
    bpy.ops.object.select_all(action='DESELECT')
    for obj in nodes.values():
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.remove_doubles(threshold=0.0001)        # weld (0.1 mm)
        bpy.ops.mesh.normals_make_consistent(inside=False)   # recalc outward
        bpy.ops.object.mode_set(mode='OBJECT')
        obj.select_set(False)

    # decimate: gentle collapse only. Planar dissolve was avoided because it
    # flattens large-radius surfaces (adjacent fine triangles are near-coplanar)
    # and leaves visible facets; a mild collapse from the fine tessellation keeps
    # curves smooth while preserving the bolt detail.
    for obj in nodes.values():
        bpy.context.view_layer.objects.active = obj
        col = obj.modifiers.new("col", 'DECIMATE')
        col.decimate_type = 'COLLAPSE'
        col.ratio = COLLAPSE
        bpy.ops.object.modifier_apply(modifier=col.name)

    # nudge the B-axis part ~3 mm along the forearm axis so its flange is not
    # coincident with the R-axis housing (avoids z-fighting at the wrist).
    nodes["link_b"].location.z += 0.003
    # shrink the R-axis bushing ~1% about the forearm axis so its outer diameter
    # clears the B-axis bore (avoids radial z-fighting inside the wrist hole).
    piv = Vector((0.0, 1.465, 0.0))
    radial = Matrix.Translation(piv) @ Matrix.Diagonal((0.99, 0.99, 1.0, 1.0)) @ Matrix.Translation(-piv)
    nodes["link_r"].data.transform(radial)
    # shrink the T-axis steel bushing ~3% about its axis so its metal surface no
    # longer intersects the orange bore (kills the z-fight stripes in the hole).
    radt = Matrix.Translation(piv) @ Matrix.Diagonal((0.97, 0.97, 1.0, 1.0)) @ Matrix.Translation(-piv)
    nodes["link_t"].data.transform(radt)
    bpy.context.view_layer.update()

    # joint empties on their rotation axes
    def add_empty(name, loc, size=0.06):
        e = bpy.data.objects.new(name, None)
        e.empty_display_type = 'ARROWS'
        e.empty_display_size = size
        e.location = loc
        bpy.context.scene.collection.objects.link(e)
        return e

    nodes["GP25"] = add_empty("GP25", Vector((0, 0, 0)), 0.12)
    for jn, mm in JOINTS_MM.items():
        nodes[jn] = add_empty(jn, v_m(mm))
    nodes["tcp"] = add_empty("tcp", v_m(TCP_MM), 0.04)

    # parent base under root, then the chain (preserving world transforms)
    def set_parent(child, parent):
        bpy.context.view_layer.update()
        world = child.matrix_world.copy()
        child.parent = parent
        child.matrix_parent_inverse.identity()
        bpy.context.view_layer.update()
        child.matrix_world = world

    set_parent(nodes["link_base"], nodes["GP25"])
    for child, parent in CHAIN:
        set_parent(nodes[child], nodes[parent])

    # materials
    orange = make_mat("Yaskawa_Orange", (0.910, 0.373, 0.071), 0.0, 0.34)
    steel = make_mat("Machined_Steel", (0.788, 0.800, 0.820), 1.0, 0.24)
    cast = make_mat("Cast_Metal", (0.231, 0.247, 0.271), 1.0, 0.50)
    try:  # glossy painted lacquer on the orange castings
        ob = orange.node_tree.nodes.get("Principled BSDF")
        ob.inputs["Coat Weight"].default_value = 0.5
        ob.inputs["Coat Roughness"].default_value = 0.12
    except Exception as exc:
        print("coat skipped:", exc)
    for node, mat in ([(n, orange) for n in ORANGE_LINKS]
                      + [(n, cast) for n in CAST_LINKS]
                      + [(n, steel) for n in STEEL_LINKS]):
        nodes[node].data.materials.clear()
        nodes[node].data.materials.append(mat)

    # shading
    bpy.ops.object.select_all(action='DESELECT')
    for node in ORANGE_LINKS + CAST_LINKS + STEEL_LINKS:
        nodes[node].select_set(True)
        bpy.context.view_layer.objects.active = nodes[node]
        bpy.ops.object.shade_smooth()
    try:
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(60))
    except Exception as exc:  # older Blender
        print("auto-smooth skipped:", exc)

    bpy.ops.export_scene.gltf(
        filepath=out_glb, export_format='GLB', export_yup=False,
        use_selection=False, export_apply=True, export_extras=True,
        export_cameras=False, export_lights=False,
    )
    print("exported", out_glb, "%.1f MB" % (os.path.getsize(out_glb) / 1e6))


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    links_dir = argv[0] if len(argv) > 0 else "gp25_links"
    out_glb = argv[1] if len(argv) > 1 else "gp25.glb"
    build(links_dir, out_glb)


if __name__ == "__main__":
    main()
