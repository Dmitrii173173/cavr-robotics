# Assets

Binary and descriptor assets used by CAVR Studio at runtime.

```
assets/
└─ robots/
   └─ yaskawa_gp25/      Yaskawa Motoman GP25 — articulated 6-axis robot
      ├─ gp25.glb               glTF 2.0 mesh + kinematic node hierarchy
      ├─ gp25.kinematics.json   joint axes, origins, limits, link↔node map
      └─ README.md              provenance, kinematics, regeneration
```

Robot meshes are articulated: every joint is a named node placed on its real
rotation axis, so a model can be posed by setting joint rotations and made to
replay recorded robot motion. The C++ kinematics for a robot live in
`cavr::visualization` (`libs/visualization/.../robot_model.hpp`); the matching
portable descriptor is the `*.kinematics.json` next to each mesh.

Assets are regenerated from vendor CAD with the scripts in
[`scripts/assets/`](../scripts/assets/README.md). Vendor CAD sources are not
committed — only the derived, decimated visualization meshes.

Paths are repository-relative; run apps from the repository root so the relative
asset paths resolve.
