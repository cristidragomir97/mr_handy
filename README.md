# handy101



An overlay importing the base101 chassis and two mod101 arms, with independent
450 mm lifts and a tilting top RealSense camera. The STEP pose is fully lowered.

| Arm | Tool | Wrist camera | Shoulder / elbow extrusion |
|---|---|---|---|
| Right (7DOF) | PGGripper | Disabled | 80 / 100 mm |
| Left (7DOF) | Luxonis camera tool | Disabled | 80 / 100 mm |

Chassis movement, lidar, IMU, navigation, SLAM and base camera remain owned by
base101. Arm geometry and tools remain owned by mod101. This repository owns
assembly, lift/head geometry, controller composition and the derived MuJoCo scene.
See [ownership](docs/ownership.md) and [control details](docs/control.md).

## Container workflow
Keep the three working trees side by side:

```
mr_handy/
  base101/
  mod101/
  handy101/
  base101_lift.step
```

The current base101/mod101 composition changes are required: the reusable wheel
interface macro, optional arm base housing and optional source control blocks,
and the camera end-effector package. Rosboard also includes a small fix that
serializes unavailable numeric measurements as JSON null. The Docker image builds the current sibling
sources, including uncommitted changes. The overlay does not vendor these repos.

```bash
cd /home/cdr/mr_handy/handy101
# Optional: copy .env.example to .env to change sibling paths/domain/GUI mode.
./scripts/manage.sh start
./scripts/manage.sh test
./scripts/manage.sh sim
```

Compose publishes rosboard on host port 8888 and the bundled Zenoh router
on port 7447, including with Docker Desktop.

Open **http://localhost:8888** for rosboard topic observation. In another terminal,
`./scripts/manage.sh test --sim` checks active controllers, trajectory actions,
measured joint states, TF, base odometry and the rosboard WebSocket stream against the running simulation.

Run one robot stack at a time; the script rejects a second `sim` or `mock` launch.
Stop the active launch with **Ctrl+C** before starting another.

The default opens the MuJoCo desktop window. The upstream ROS plugin hides
its panels initially: press **Tab** for the left panel and **Shift+Tab** for
the right panel. To run without a display:

```bash
./scripts/manage.sh sim headless:=true
```

Stop the foreground robot launch with Ctrl+C. `./scripts/manage.sh stop` stops the
project's containers. `restart`, `recreate`, `logs`, `exec` and `build` follow the grove-g1 management-script convention; `help` lists commands.
`recreate` preserves source and named build volumes. `start` builds a complete
image; after source edits, use `build` before launching again. Container build,
install and log directories use named volumes separate from native build files.

The included Zenoh router listens on port 7447. If a router already runs locally,
set `HANDY101_START_ROUTER=false` in `.env`; the stack will use that router.
The default ROS domain is 184, shared by the robot, rosboard and container shells.
The script leaves other projects' processes and containers alone.

The image uses official ROS Jazzy, ros2_control, mujoco_ros2_control and Mesa;
no GPU is needed for the headless controller/physics workflow. The image also
contains the scene checks and a standalone built assembly. Compose additionally
mounts the source repositories for development. Rendering uses the available GPU:

## Native build and launch

```bash
source /opt/ros/jazzy/setup.bash
# Build updated mod101 descriptions/tools in their workspace first.
source ../mod101/install/setup.bash
colcon build --symlink-install --base-paths src ../base101/src \
  --packages-select base101_description base101_control rosboard \
  handy101_description handy101_control handy101_bringup handy101_mujoco
source install/setup.bash
export ROS_DOMAIN_ID=184 RMW_IMPLEMENTATION=rmw_zenoh_cpp
# Start ros2 run rmw_zenoh_cpp rmw_zenohd in another terminal if no router runs.
ros2 launch handy101_mujoco sim.launch.py headless:=true
```

For a controller routing dry run, use
`ros2 launch handy101_bringup control.launch.py backend:=mock`.
Both launches start rosboard; `rosboard:=false` disables it and
`rosboard_port:=8889` changes its port. `control_mode:=position` activates direct
position controllers instead of their trajectory counterparts. `arms:=false`
provides a base/lifts/head variant. `floating_base:=false` on the MuJoCo launch
anchors the chassis for bench testing. `scene_dir:=/path` retains generated
`scene.xml` and `robot.urdf` for inspection; otherwise a temporary directory is used.

## Description and CAD

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


## Simulation sensors and movement

`./scripts/manage.sh sim` starts native MuJoCo publishers on the live physics model:

| Topic | Measurement / frame |
|---|---|
| `/scan` | 360 lidar rays, `lidar_frame`, 10Hz |
| `/sensors/imu` | orientation, angular velocity, specific acceleration including gravity; `imu_link` |
| `/base_camera/color/image_raw` | base RGB, `camera_optical_frame` |
| `/head_camera/color/image_raw` | tilting top RGB, `head_camera_optical_frame` |
| `/left_arm_camera/color/image_raw` | Luxonis RGB, `left_arm_camera_tool_optical_frame` |

All three cameras publish 640x480 RGB and `<camera>/camera_info` at 5Hz, plus
`<camera>/depth/image_raw` (32FC1 metres). GoPro and wrist cameras are absent.
Camera FOVs are provisional nominal values, and top/Luxonis optical
centres use mechanical front-face datums until calibrated.

Base101-compatible image aliases `/base_camera/image_raw` and `/base_camera/depth_image`
are also available, along with the older rosboard base-camera topic names.
The default world is a furnished three-room RoboCasa home; `world:=empty`
selects the commissioning floor. Lidar returns infinity for rays with no obstacle. These are scene measurements, not prerecorded sensor placeholders.

Movement uses base101's `twist_mux.yaml`: `/cmd_vel_joy`, `/cmd_vel_key`,
`/cmd_vel_agent` and `/cmd_vel_nav` take `TwistStamped`. Conventional `/cmd_vel`
`Twist` input is stamped and fed into the keyboard mux input. Rosboard permits
Twist publishing, so its teleop card can drive `/cmd_vel_joy`. The controller still
provides `/diff_drive_controller/odom` and `odom -> base_link`.

`./scripts/manage.sh test --sim` checks sensors and the public movement input as well
as controller actions, TF and the rosboard stream. This drives short test motions.

Current graphics limitation: one GUI shutdown with camera rendering active crashed
in Mesa/libgallium cleanup and required launch escalation. Runtime camera, sensor
and drive tests pass; GPU shutdown cleanup remains to be resolved.

## Furnished home and joint sliders

The default `world:=robocasa_home` is a custom 9m × 9m, three-room MuJoCo home:
kitchen, dining room and living room, connected through 1.4m openings. It uses
textured, articulated RoboCasa-compatible Lightwheel refrigerator/stove assets,
plus an authored counter, prep table, dining table/chairs, sofa, coffee table,
bookshelf and room flooring. Nine loose, positive-mass props (cups, honey bottles
and coloured blocks) have free joints and collide with the robot, furniture and
floor. Cup/bottle assets are scaled to 45% to fit the PGGripper's 54mm aperture.

This is an asset-level integration into Mr Handy's ROS-controlled MuJoCo scene.
The robot and its control remain owned by the sibling projects. The scene's
furniture/objects come from the [Lightwheel kitchen MJCF collection](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-Manipulation-Objects-Kitchen-MJCF),
credited to NVIDIA / Lightwheel, licensed CC-BY-4.0. `assets/robocasa/manifest.json`
records the pinned revision, file hashes, bounds and scales. This custom layout
adds dining/living rooms to the kitchen-focused asset collection.

```bash
./scripts/manage.sh world-assets   # prepare assets; also run automatically by start
./scripts/manage.sh start
./scripts/manage.sh sim            # furnished home, normal GUI
./scripts/manage.sh sim world:=empty   # commissioning floor
```

The setup caches about 90MB of download archives and keeps only selected models.
Prepared assets are bundled into Docker; source caches are ignored by Git.
