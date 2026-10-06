# Control composition

One controller manager loads one `handy101_system`. It exposes four wheel
velocity commands and 16 overlay position commands, each with position/velocity
feedback. Source arm/tool control blocks are suppressed in this assembly, so
MuJoCo tool feedback cannot accidentally come from mock hardware. Their default
behavior remains unchanged in standalone mod101.

Wheel interfaces come from `base101_description/urdf/wheel_interfaces.xacro`.
Drive geometry and settings come from `base101_control/config/controllers.sim.yaml`;
the overlay adds controller declarations and enables command-limit enforcement.
It uses `mock_components/GenericSystem` for routing checks and the installed
`mujoco_ros2_control/MujocoSystemInterface` for physics. MuJoCo requires its own
`ros2_control_node` executable, as described in the
[official Jazzy interface documentation](https://control.ros.org/jazzy/doc/mujoco_ros2_control/mujoco_ros2_control/docs/hardware_interface.html).

| Controller | Joint order |
|---|---|
| diff_drive_controller | Base101's front/back left/right wheels |
| left_lift_controller | left_lift_joint |
| right_lift_controller | right_lift_joint |
| head_controller | head_camera_tilt_joint |
| left_arm_controller | left_arm_joint_base, joint_shoulder, joint_elbow, joint_wrist_yaw, joint_wrist_tilt, joint_wrist_roll (all left_arm_ prefixed) |
| right_arm_controller | Same six joints with right_arm_ prefix |
| right_gripper_controller | right_arm_6 (PGGripper motor) |

`joint_state_broadcaster` publishes all 22 joints on `/joint_states`, and
robot_state_publisher uses them for TF. The six overlay trajectory controllers
provide `<controller>/follow_joint_trajectory` actions and accept
`trajectory_msgs/JointTrajectory` on `<controller>/joint_trajectory`.
Linear positions use metres; rotary positions use radians.

For example, with default trajectory mode:

```bash
./scripts/manage.sh exec ros2 topic pub --once /left_lift_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [left_lift_joint], points: [{positions: [0.1], time_from_start: {sec: 10}}]}'
```

A 100 mm lift move takes at least five seconds at the provisional 20 mm/s limit.
The example allows ten seconds. Head position is relative to the exported camera
pose; positive rotation tilts down.

Position mode replaces each group with `<group>_position_controller`. Publish
`std_msgs/Float64MultiArray` on its `/commands` topic in the same joint order.
Each pair claims the same interfaces, so only one mode is activated at startup.
Tools/lifts stay independent. Both arms include wrist yaw. The left camera tool
is passive and has no gripper controller. Right PGGripper motor `right_arm_6`
runs from 0 to 2.271 rad; two passive sliders convert radians to metres through
the CAD-derived rack ratio. Their source URDF mimics become MuJoCo equality
constraints and expose feedback only. Both wrist cameras are disabled.

The base drive consumes `geometry_msgs/TwistStamped` on
`/diff_drive_controller/cmd_vel`. It publishes `/diff_drive_controller/odom` and
`odom -> base_link`, using the inherited base101 dimensions and timeout settings.
MuJoCo also exposes `/simulation/ground_truth_odom` for the floating base and
`/clock`; the complete ROS stack uses simulation time in this backend.

The scene derives every joint origin, axis, bound, mesh and inertial from the
same expanded URDF used by the controller manager. Axes are normalized by the
physics engine. The base has a free joint; fixed URDF bodies are retained with their
imported mass/inertia to preserve sensor attachment. Position servos use provisional proportional gains with
MuJoCo mass-scaled critical damping; wheel actuators use velocity control.
Tool coupling uses joint equalities plus provisional 0.0001 kg·m² reflected
drive inertia and 0.002 N·m·s/rad damping to regularize its frame-only coupler.
`implicitfast`, a 2 ms physics step and 100 Hz ROS updates provide stable
commissioning dynamics. The lightweight tool links require scaled
damping rather than a large fixed damping gain.

Self-contact is disabled in this commissioning model because imported CAD
mechanisms overlap coarse URDF collision hulls. Floor/environment contact stays
enabled. Soft joint stops and gravity produce small measured position offsets;
this is simulated feedback, not exact command echo. Hardware torque, velocity,
head range, lift drive and homing calibration are not established by these gains.

Rosboard is imported from base101's existing package and started by bringup.
It observes the graph over HTTP/WebSocket on port 8888. Its publishing allowlist
permits Twist and TwistStamped for base teleoperation, and JointTrajectory and
Float64MultiArray for joint control.

## Browser joint control

Bringup generates `/rosboard/joint-controls` from the assembled URDF and selected
controller mode. The slider panel uses this configuration for all 16 position
axes, including both wrist-yaw joints. Mimic followers expose feedback only.

In trajectory mode, sliders publish complete, ordered groups of joint positions
with velocity-aware durations on each controller's `/joint_trajectory` topic.
Position mode uses `/commands`. The server checks trajectory topics, joint order,
finite positions and joint bounds against the robot configuration before publishing.

Native CameraPlugin and RangefinderLidarPlugin publish from the same MuJoCo
state used by controllers. The base IMU exposes native frame-quaternion, gyro
and accelerometer state interfaces to imu_broadcaster; its output is remapped
to /sensors/imu. Base101 twist_mux settings route stamped velocity commands,
and the command bridge stamps conventional /cmd_vel Twist messages.
