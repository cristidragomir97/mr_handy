# Robot description and CAD

[← README](../README.md)

`handy101_description/urdf/handy101.xacro` imports the source chassis and arms.
Each arm root is attached to its respective `lift_*_main` at a measured transform.
The lift platforms replace standalone mod101 base housings, while retaining their
base servos. Servo and shoulder fits agree within 0.03 mm in the STEP bounds.
Measurement records live in `handy101_description/config/`.

For CAD Viewer inspection from sibling sources:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/expand_description.py --bundle-meshes
check_urdf tmp/handy101.urdf
```

Only the ignored preview folder bundles imported meshes. The maintained
assembly imports packages. `tools/export_lift_meshes.py ../base101_lift.step`
refreshes overlay-owned meshes using OpenCascade; authored xacro constants must
be reviewed after CAD changes. Mesh coordinates are millimetres, scaled 0.001.
Extrusions use aluminum; lift tops, bases and platforms use black plastic.
Steel guides/screws and brass nuts use separate material visuals.

`head_camera_tilt_joint` rotates around the measured servo shaft (+Y); positive
angles tilt downward. Zero retains the exported pose. `head_tilt_lower` and
`head_tilt_upper` are radians, provisionally −π/2 to +π/2. Per-arm tool, camera,
wrist-camera and rail-length arguments are explicit in `arms.xacro`, independent
of the bench configurator and chassis camera argument.
