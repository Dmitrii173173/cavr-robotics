# Asset build scripts

Reproducible pipeline that turns vendor CAD into the articulated, real-time robot
assets under `assets/robots/`.

## Yaskawa GP25

Requires FreeCAD 1.x (`freecadcmd`) and Blender 4.x/5.x (`blender`) on PATH.

```bash
# 1. STEP -> one mesh per kinematic link (millimetres, assembly frame)
freecadcmd freecad_export_gp25_links.py /path/to/GP25.stp ./gp25_links

# 2. links -> articulated, materialled, decimated GLB (metres, Y-up)
blender --background --python blender_build_gp25.py -- ./gp25_links \
    ../../assets/robots/yaskawa_gp25/gp25.glb
```

Step 1 relies on the STEP components being pre-named per axis
(`_BASE_AXIS`, `_S_AXIS`, … `_T_AXIS`) — one solid per link. Step 2 places each
joint node on its real rotation axis using origins baked into the script.

The joint origins, axes, limits and link→node mapping are the kinematic ground
truth and are kept in three places that must stay in sync:

- `scripts/assets/blender_build_gp25.py` (geometry construction)
- `assets/robots/yaskawa_gp25/gp25.kinematics.json` (portable descriptor)
- `libs/visualization/include/cavr/visualization/robot_model.hpp` (C++ + FK)

The CAD source (`GP25.stp`) is not stored in this repository; only the derived
visualization mesh is committed.
