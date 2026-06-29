"""Export the Yaskawa GP25 STEP assembly to one mesh per kinematic link.

Run with FreeCAD's headless interpreter (FreeCAD 1.x):

    freecadcmd freecad_export_gp25_links.py <GP25.stp> <out_dir>

The GP25 STEP exports each link as a single, pre-named solid
(``_BASE_AXIS``, ``_S_AXIS``, ... ``_T_AXIS``). Each is tessellated and written
as ``<id>.obj`` in millimetres in the shared assembly (world) frame, ready for
``blender_build_gp25.py``.
"""

import os
import sys

import FreeCAD as App
import Import
import MeshPart

# CAD label -> output link id (drives the build order downstream)
LINKS = {
    "_BASE_AXIS_SW0001": "00_base",
    "_S_AXIS_SW0001": "01_s",
    "_L_AXIS_SW0002": "02_l",
    "_U_AXIS_SW0001": "03_u",
    "_R_AXIS_SW0001": "04_r",
    "_B_AXIS_SW0001": "05_b",
    "_T_AXIS_SW0001": "06_t",
}

LINEAR_DEFLECTION = 0.15   # mm
ANGULAR_DEFLECTION = 0.18  # rad (drives smoothness of curved surfaces)


def main(step_path, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    doc = App.newDocument("gp25")
    Import.insert(step_path, "gp25")

    found = 0
    for obj in doc.Objects:
        link_id = LINKS.get(getattr(obj, "Label", None))
        if not link_id:
            continue
        mesh = MeshPart.meshFromShape(
            Shape=obj.Shape,
            LinearDeflection=LINEAR_DEFLECTION,
            AngularDeflection=ANGULAR_DEFLECTION,
            Relative=False,
        )
        path = os.path.join(out_dir, link_id + ".obj")
        mesh.write(path)
        found += 1
        print("%-8s %7d tris -> %s" % (link_id, mesh.CountFacets, path))

    if found != len(LINKS):
        raise SystemExit("expected %d links, exported %d" % (len(LINKS), found))
    print("done:", out_dir)


if __name__ == "__main__":
    step = sys.argv[1] if len(sys.argv) > 1 else "GP25.stp"
    out = sys.argv[2] if len(sys.argv) > 2 else "gp25_links"
    main(step, out)
