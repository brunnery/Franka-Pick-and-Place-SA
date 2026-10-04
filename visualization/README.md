# 3D visualisation: pick and place with the Franka FR3

`index.html` is a standalone page (all models are embedded). Open it in a browser by double-clicking it.
It needs an internet connection once to load three.js from the jsDelivr CDN.

What it shows:

- `cad/export/Setup.step`: base plate, the two fixtures, the M6 nut and the screw, to scale
- `cad/export/Iris main file_new design.step`: the iris wheel gripper (six blades, two drive wheels), mounted on the FR3 flange
- the Franka FR3 arm (visual meshes from [franka_description](https://github.com/frankarobotics/franka_description), Apache-2.0)

The sequence is: approach the nut, lower, close the iris, lift, move to the screw, lower onto the thread,
screw the nut on (15 turns, 1 mm pitch, the arm follows 15 mm down), open the iris, retract, return home.
The joint angles come from a damped least-squares IK with the FR3 kinematics and joint limits, with the tool kept vertical.

The robot base sits at x = -200 mm in plate coordinates (the placeholder arm in `Setup.step` sits at x = +50 mm).
With the 156 mm gripper, the nut and the screw are too close to the base at +50 mm to be reached with a vertical tool.

## Files

| Path | Content |
| --- | --- |
| `index.html` | built page, open this |
| `src/viewer.src.html` | page source (three.js scene, timeline, UI) |
| `src/kin.js` | FR3 forward kinematics and IK |
| `assets/*.glb` | converted meshes (metres, z up) |
| `tools/step_to_glb.py` | STEP to GLB conversion (OpenCascade via `cadquery-ocp`) |
| `tools/build_assets.py` | builds the three GLB files in `assets/` |
| `tools/build_viewer.py` | inlines `kin.js` and the GLB files into `index.html` |

## Rebuild after a CAD change

```bash
pip install cadquery-ocp trimesh pycollada fast_simplification
# 1. export the new STEP files to cad/export/
# 2. convert them and rebuild the assets (paths in build_assets.py)
python tools/step_to_glb.py "../cad/export/Setup.step" cad/Setup.glb
python tools/step_to_glb.py "../cad/export/Iris main file_new design.step" cad/Iris_wheel.glb
python tools/build_assets.py
# 3. rebuild the page
python tools/build_viewer.py
```

Positions of the nut, screw and robot base are set at the top of the module script in `src/viewer.src.html` (`BASE`, `NUT0`, `SCREW`).
