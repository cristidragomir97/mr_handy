"""One controller manager, shared base drive configuration, overlay joints."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
import xacro
import xml.etree.ElementTree as ET
import yaml
import tempfile
import json


def _controller_file(config):
    with tempfile.NamedTemporaryFile(mode="w", prefix="handy101-controllers-", suffix=".yaml", delete=False) as file:
        yaml.safe_dump(config, file)
        return file.name


def build(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    backend = arg("backend")
    namespace = arg("namespace").strip("/")
    description = get_package_share_directory("handy101_description")
    mappings = {
        "simulator": "none",
        "control_backend": backend,
        "arms": arg("arms"),
        "headless": arg("headless"),
        "mujoco_model": arg("mujoco_model"),
    }
    if backend == "mujoco" and not Path(mappings["mujoco_model"]).is_file():
        raise RuntimeError(
            "Pass mujoco_model:=<compiled scene.xml>, or use handy101_mujoco sim.launch.py."
        )
    robot = xacro.process_file(
        str(Path(description) / "urdf/handy101.xacro"), mappings=mappings
    ).toxml()
    joints = {j.get("name"): j for j in ET.fromstring(robot).findall("joint")}
    use_sim_time = backend == "mujoco"
    groups = ["left_lift", "right_lift", "head"]
    if arg("arms").lower() == "true":
        groups += ["left_arm", "right_arm"]
        groups += [side + "_gripper" for side in ("left", "right") if side + "_arm_6" in joints]
    suffix = "_controller" if arg("control_mode") == "trajectory" else "_position_controller"
    controllers = ["joint_state_broadcaster", "diff_drive_controller"] + [
        g + suffix for g in groups
    ]
    if backend == "mujoco":
        controllers.append("imu_broadcaster")
    manager = "/" + (
        "/".join([namespace, "controller_manager"]) if namespace else "controller_manager"
    )
    # Derive arm axis lists from the assembled URDF, including optional wrist yaw.
    overlay = yaml.safe_load((Path(get_package_share_directory("handy101_control")) / "config/controllers.yaml").read_text())
    for side in ("left", "right"):
        arm_joints = [side + "_arm_" + name for name in
                      ("joint_base", "joint_shoulder", "joint_elbow", "joint_wrist_yaw", "joint_wrist_tilt", "joint_wrist_roll")
                      if side + "_arm_" + name in joints]
        for mode in ("_controller", "_position_controller"):
            overlay[side + "_arm" + mode]["ros__parameters"]["joints"] = list(arm_joints)
    # Separate ROS node parameter dictionaries retain the controller names.
    config = [
        str(Path(get_package_share_directory("base101_control")) / "config/controllers.sim.yaml"),
        # Controller-specific node parameters require a YAML file (not a node-local dict).
        _controller_file(overlay),
        {"use_sim_time": use_sim_time},
    ]
    if backend == "mujoco":
        config.append(str(Path(get_package_share_directory("handy101_control")) / "config/sensors.mujoco.yaml"))
    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[{"robot_description": robot, "use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="mujoco_ros2_control" if backend == "mujoco" else "controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            namespace=namespace,
            parameters=config,
            remappings=[
                ("~/robot_description", "robot_description"),
                ("/robot_description", "robot_description"),
                ("/imu_broadcaster/imu", "/sensors/imu"),
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=controllers
            + [
                "--controller-manager",
                manager,
                "--controller-manager-timeout",
                "60",
                "--activate-as-group",
            ],
            output="screen",
        ),
    ]
    actions.append(Node(package="twist_mux", executable="twist_mux", name="twist_mux",
                        parameters=[str(Path(get_package_share_directory("base101_control")) / "config/twist_mux.yaml"), {"use_sim_time":use_sim_time}],
                        remappings=[("cmd_vel_out", "/diff_drive_controller/cmd_vel")], output="screen"))
    actions.append(Node(package="handy101_bringup", executable="cmd_vel_bridge.py",
                        parameters=[{"use_sim_time":use_sim_time}], output="screen"))
    manager_node = actions[1]
    spawner = actions[2]
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=manager_node,
                on_exit=[EmitEvent(event=Shutdown(reason="Controller manager exited"))],
            )
        )
    )

    def spawner_exit(event, context):
        return (
            [EmitEvent(event=Shutdown(reason="Controller activation failed"))]
            if event.returncode
            else []
        )

    actions.append(RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=spawner_exit)))
    if arg("rosboard").lower() == "true":
        slider_groups = []
        for group in groups:
            controller = group + suffix
            ordered = overlay[controller]["ros__parameters"]["joints"]
            slider_groups.append({"title":group.replace("_", " "),
                "topic":"/" + "/".join(filter(None, [namespace, controller, "joint_trajectory" if arg("control_mode") == "trajectory" else "commands"])),
                "type":"trajectory_msgs/msg/JointTrajectory" if arg("control_mode") == "trajectory" else "std_msgs/msg/Float64MultiArray",
                "joints":[{"name":name, "min":float(joints[name].find("limit").get("lower")),
                           "max":float(joints[name].find("limit").get("upper")),
                           "velocity":min(float(joints[name].find("limit").get("velocity")), 1.5),
                           "unit":"m" if joints[name].get("type") == "prismatic" else "rad"} for name in ordered]})
        actions.append(
            Node(
                package="rosboard",
                executable="rosboard_node",
                namespace=namespace,
                parameters=[
                    {
                        "port": int(arg("rosboard_port")),
                        "title": "handy101",
                        "publish_allowlist": ["geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped", "trajectory_msgs/msg/JointTrajectory", "std_msgs/msg/Float64MultiArray"],
                        "joint_control_config":json.dumps({"groups":slider_groups}),
                    }
                ],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("rosboard", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("rosboard_port", default_value="8888"),
            DeclareLaunchArgument("backend", default_value="mock", choices=["mock", "mujoco"]),
            DeclareLaunchArgument(
                "control_mode", default_value="trajectory", choices=["trajectory", "position"]
            ),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("arms", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("mujoco_model", default_value=""),
            DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
            OpaqueFunction(function=build),
        ]
    )
