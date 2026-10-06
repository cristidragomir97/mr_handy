"""Build a derived scene from the exact imported URDF, then start ROS control."""

from pathlib import Path
import math
import tempfile
from ament_index_python.packages import get_package_share_directory
from handy101_mujoco.scene import build_scene
from handy101_mujoco.world import add_home, default_assets
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro
from launch_ros.actions import Node, SetRemap


def build(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    mapping = arg("mapping").lower() == "true"
    scene_dir = (
        Path(arg("scene_dir"))
        if arg("scene_dir")
        else Path(tempfile.mkdtemp(prefix="handy101-mujoco-"))
    )
    scene = scene_dir / "scene.xml"
    source = Path(get_package_share_directory("handy101_description")) / "urdf/handy101.xacro"
    urdf = xacro.process_file(
        str(source),
        mappings={
            "simulator": "none",
            "control_backend": "mujoco",
            "mujoco_model": str(scene),
            "headless": arg("headless"),
            "arms": arg("arms"),
        },
    ).toxml()
    build_scene(urdf, scene, floating=arg("floating_base").lower() == "true")
    if arg("world") == "robocasa_home":
        add_home(scene, arg("world_assets"))
    (scene_dir / "robot.urdf").write_text(urdf)
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory("handy101_bringup"))
                    / "launch/control.launch.py"
                )
            ),
            launch_arguments={
                "backend": "mujoco",
                "fused_odometry": "true" if mapping else "false",
                "mujoco_model": str(scene),
                "headless": arg("headless"),
                "arms": arg("arms"),
                "control_mode": arg("control_mode"),
                "namespace": arg("namespace"),
                "rosboard": arg("rosboard"),
                "rosboard_port": arg("rosboard_port"),
            }.items(),
        )
    ]
    # Import base101's scan preprocessing and independent autonomy launches.
    # These packages use global topic/frame names, so reject a mixed namespace.
    navigation = arg("navigation").lower() == "true"
    if (mapping or navigation) and arg("namespace").strip("/"):
        raise RuntimeError("base101 mapping/navigation require namespace:=; disable both for namespaced control")
    if mapping or navigation:
        nav_share = Path(get_package_share_directory("base101_nav"))
        actions.append(Node(
            package="laser_filters", executable="scan_to_scan_filter_chain",
            name="scan_to_scan_filter_chain", output="screen",
            parameters=[str(nav_share / "config/handy101_lidar_filters.yaml"),
                        {"use_sim_time": True,
                         # Measured in the assembled home: chassis self-hits span
                         # -58..58 degrees around the rear-facing scan zero.
                         # 65 degrees leaves margin while retaining rear diagonals
                         # so the initial SLAM map covers the robot behind the lidar.
                         "filter1.params.lower_angle": -math.radians(65),
                         "filter1.params.upper_angle": math.radians(65)}],
            remappings=[("scan", "/scan"), ("scan_filtered", "/scan_filtered")],
        ))
    autonomy = []
    if mapping:
        autonomy.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory("base101_slam")) / "launch/slam.launch.py")),
            launch_arguments={
                "use_sim_time": "true",
                "ekf_config": str(Path(get_package_share_directory("handy101_control"))
                                  / "config/ekf.mujoco.yaml"),
            }.items(),
        ))
    if navigation:
        nav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav_share / "launch/nav.launch.py")),
            launch_arguments={"use_sim_time": "true", "rviz": arg("rviz")}.items(),
        )
        autonomy.append(GroupAction(actions=[
            SetRemap(src="/diff_drive_controller/odom", dst="/odometry/filtered"),
            nav_launch,
        ]) if mapping else nav_launch)
    if autonomy:
        actions.append(TimerAction(period=5.0, actions=autonomy))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mapping", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("navigation", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("world", default_value="robocasa_home", choices=["robocasa_home", "empty"]),
            DeclareLaunchArgument("world_assets", default_value=default_assets()),
            DeclareLaunchArgument("rosboard", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("rosboard_port", default_value="8888"),
            DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("floating_base", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("arms", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument(
                "control_mode", default_value="trajectory", choices=["trajectory", "position"]
            ),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("scene_dir", default_value=""),
            OpaqueFunction(function=build),
        ]
    )
