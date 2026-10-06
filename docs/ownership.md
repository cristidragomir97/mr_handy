# Composition boundaries

| Responsibility | Owner |
|---|---|
| Chassis geometry, wheel frames and wheel joints | base101_description |
| Drive control, odometry, lidar, IMU, base camera | base101 packages |
| Navigation and SLAM | base101 packages |
| Lift geometry, vertical joints and controllers | handy101 |
| Two arm mounting transforms and per-arm configuration | handy101 |
| Arm links, joints, tools and reusable arm description | mod101 |
| Upper camera mounting/tilt control and simulation image integration | handy101 |
| Complete robot assembly, controller composition and MuJoCo scene | handy101 |

The assembly imports the original chassis xacro, with its legacy single arm
and optional raised deck/standoffs disabled, and instantiates the lift subsystem,
tilting upper camera and two mod101 arms under lift_left_main and lift_right_main.
All arm parameters are explicit; prefixes are left_arm_ and right_arm_.
The right 7DOF arm has a PGGripper, no wrist camera and 80/100 mm shoulder/elbow extrusions.
The left 7DOF arm has a Luxonis camera tool, without GoPro, no wrist camera and 80/100 mm extrusions.
The lift platforms replace the standalone arm base housing, using the source
macro's optional base_housing=false mode. Arm geometry stays owned by mod101.

The STEP pose is the lowest position; both lifts have 450 mm travel. Force,
velocity, damping and mass properties are simulation estimates. The prismatic
joints describe carriage travel directly. Screw pitch/lead, servo gearing and
hardware calibration belong to the future control layer.

The base camera parameter remains owned by base101. Upper and arm camera
choices must use separate overlay parameters so they cannot change the base
camera when mod101_config.xacro is included.

The overlay selects native MuJoCo sensor plugins for the base-owned frames and
loads base101_control’s twist multiplexer settings. Base hardware lidar/camera
drivers are not launched in a physics simulation. Camera plugin configuration
is in handy101_control/config/sensors.mujoco.yaml.
