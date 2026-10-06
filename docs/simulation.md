# Simulation

[← README](../README.md)

## The furnished home

The default `world:=robocasa_home` is a custom 9m × 9m, three-room MuJoCo home:
kitchen, dining room and living room, connected through 1.4m openings. It uses
textured, articulated RoboCasa-compatible Lightwheel refrigerator/stove assets,
plus an authored counter, prep table, dining table/chairs, sofa, coffee table,
bookshelf and room flooring. Nine loose, positive-mass props (cups, honey bottles
and coloured blocks) have free joints and collide with the robot, furniture and
floor. Cup/bottle assets are scaled to 45% to fit the PGGripper's 54mm aperture.

The refrigerator, stove, cups and bottles come from the
[Lightwheel kitchen MJCF collection](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-Manipulation-Objects-Kitchen-MJCF),
credited to NVIDIA / Lightwheel and licensed CC-BY-4.0. The remaining furniture
and room layout are authored here. `assets/robocasa/manifest.json` records the
pinned revision, file hashes, bounds and scales.

```bash
./scripts/manage.sh world-assets   # prepare assets; also run automatically by start
./scripts/manage.sh start
./scripts/manage.sh sim            # furnished home, normal GUI
./scripts/manage.sh sim world:=empty   # commissioning floor
```

The setup caches about 90MB of download archives and keeps only selected models.
Prepared assets are bundled into Docker; source caches are ignored by Git.

This uses RoboCasa-compatible assets in a custom MuJoCo layout. It does not run
the RoboCasa task/reward framework, and there is no automatic pickup policy.

## Sensors and movement

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
The native lidar plugin reports out-of-range rays as `-1`; discard values outside
the scan's range bounds when processing it.

Movement uses base101's `twist_mux.yaml`: `/cmd_vel_joy`, `/cmd_vel_key`,
`/cmd_vel_agent` and `/cmd_vel_nav` take `TwistStamped`. Conventional `/cmd_vel`
`Twist` input is stamped and fed into the keyboard mux input. Rosboard permits
Twist publishing, so its teleop card can drive `/cmd_vel_joy`. The controller still
provides `/diff_drive_controller/odom` and `odom -> base_link`.

`./scripts/manage.sh test --sim` checks sensors and the public movement input as well
as controller actions, TF and the rosboard stream. This drives short test motions.

## Joint sliders

Open **Joint sliders** in rosboard. The panel includes the two lifts, head tilt,
six motion axes on each arm and the right gripper. Lift values are in metres;
rotary values are in radians. The readouts show measured joint feedback.

The panel loads its joint order, limits and controller topics from the assembled
robot. Commands use the selected trajectory or position controllers. Refresh the
browser after rebuilding rosboard or changing the robot configuration.

## Checks

```bash
./scripts/manage.sh test          # description, controller coverage and physics
./scripts/manage.sh test --sim    # run against an already launched simulation
```

The live checks exercise trajectory actions, measured joint states, TF, odometry,
rosboard, camera streams, IMU and lidar. They move the robot briefly, so run them
when the surrounding scene is clear.

Offline checks also verify camera/ray transforms, sensor attachment, object mass
and gravity, room contacts and usable lidar returns. Fixed-body fusion is disabled:
it can misbind robot sensor sites when appliance sites are added. Rangefinders are
attached to the lidar housing body so they exclude its own rotor shell.
