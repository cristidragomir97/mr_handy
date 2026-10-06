"""Build a derived scene from the exact imported URDF, then start ROS control."""

from pathlib import Path
import tempfile
from ament_index_python.packages import get_package_share_directory
from handy101_mujoco.scene import build_scene
from handy101_mujoco.world import add_home, default_assets
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro


def build(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

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
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory("handy101_bringup"))
                    / "launch/control.launch.py"
                )
            ),
            launch_arguments={
                "backend": "mujoco",
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


def generate_launch_description():
    return LaunchDescription(
        [
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
