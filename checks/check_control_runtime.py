#!/usr/bin/env python3
"""Live ROS command/feedback check against an already-running handy101 stack."""

import argparse
import math
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=["mock", "mujoco"], default="mock")
    parser.add_argument("--namespace", default="")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node(
        "handy101_control_check",
        namespace=args.namespace,
        parameter_overrides=[Parameter("use_sim_time", value=args.backend == "mujoco")],
    )
    state = {}
    velocities = {}
    transforms = {}
    odom = []

    def feedback(msg):
        state.update(zip(msg.name, msg.position))
        velocities.update(zip(msg.name, msg.velocity))

    def tf(msg):
        for transform in msg.transforms:
            transforms[transform.child_frame_id] = transform

    node.create_subscription(JointState, "joint_states", feedback, qos_profile_sensor_data)
    node.create_subscription(TFMessage, "tf", tf, 100)
    node.create_subscription(
        Odometry, "diff_drive_controller/odom", lambda msg: odom.append(msg), 10
    )

    def wait(predicate, seconds=20):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError("Timed out waiting for ROS feedback/service")

    def service(kind, name):
        client = node.create_client(kind, name)
        assert client.wait_for_service(timeout_sec=20), name
        future = client.call_async(kind.Request())
        wait(future.done)
        return future.result()

    controllers = service(ListControllers, "controller_manager/list_controllers")
    active = {c.name for c in controllers.controller if c.state == "active"}
    groups = {
        "left_lift": ["left_lift_joint"],
        "right_lift": ["right_lift_joint"],
        "head": ["head_camera_tilt_joint"],
        "left_arm": [
            "left_arm_" + n
            for n in [
                "joint_base",
                "joint_shoulder",
                "joint_elbow",
                "joint_wrist_yaw",
                "joint_wrist_tilt",
                "joint_wrist_roll",
            ]
        ],
        "right_arm": [
            "right_arm_" + n
            for n in [
                "joint_base",
                "joint_shoulder",
                "joint_elbow",
                "joint_wrist_yaw",
                "joint_wrist_tilt",
                "joint_wrist_roll",
            ]
        ],
        "right_gripper": ["right_arm_6"],
    }
    expected = {"joint_state_broadcaster", "diff_drive_controller"} | ({"imu_broadcaster"} if args.backend == "mujoco" else set()) | {
        name + "_controller" for name in groups
    }
    assert active == expected, (active, expected)
    interfaces = service(ListHardwareInterfaces, "controller_manager/list_hardware_interfaces")
    joint_names = {joint for joints in groups.values() for joint in joints} | {
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "back_left_wheel_joint",
        "back_right_wheel_joint",
    }
    hardware_interfaces = [
        i for i in interfaces.command_interfaces if i.name.split("/")[0] in joint_names
    ]
    assert len(hardware_interfaces) == 20
    assert all(i.is_claimed and i.is_available for i in hardware_interfaces)
    wait(lambda: len(state) == 22 and len(transforms) >= 19)
    print(
        "PASS: expected active controllers, all 20 interfaces claimed, JointState and TF received.",
        flush=True,
    )
    targets = {
        "left_lift": [0.04],
        "right_lift": [0.02],
        "head": [0.15],
        "left_arm": [0.1, 0.35, 0.5, 0.2, 0.1, 0.25],
        "right_arm": [-0.1, 0.3, 0.45, -0.2, 0.1, 0.25],
        "right_gripper": [1.2],
    }
    clients = []
    results = []
    for name, joints in groups.items():
        client = ActionClient(
            node, FollowJointTrajectory, name + "_controller/follow_joint_trajectory"
        )
        clients.append(client)
        assert client.wait_for_server(timeout_sec=10), name
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = targets[name]
        point.time_from_start.sec = 4
        goal.trajectory.points = [point]
        future = client.send_goal_async(goal)
        wait(future.done)
        handle = future.result()
        assert handle.accepted, name
        results.append((name, handle.get_result_async()))
    wait(lambda: all(f.done() for _, f in results), seconds=20)
    for name, future in results:
        result = future.result().result
        assert result.error_code == FollowJointTrajectory.Result.SUCCESSFUL, (
            name,
            result.error_code,
            result.error_string,
        )
    expected_state = {
        joint: value
        for group, joints in groups.items()
        for joint, value in zip(joints, targets[group])
    }
    tolerance = 0.015 if args.backend == "mujoco" else 0.001
    wait(
        lambda: all(
            abs(state.get(joint, math.inf) - value) < tolerance
            for joint, value in expected_state.items()
        )
    )
    print(
        "PASS: all six FollowJointTrajectory actions succeeded; measured joints match independent targets.",
        flush=True,
    )
    drive = state["right_arm_6"]
    import json
    from pathlib import Path
    from ament_index_python.packages import get_package_share_directory
    radius = json.loads((Path(get_package_share_directory("mod101_tool_pggripper")) / "config/measurements.json").read_text())["pitch_radius_m"]
    for side, ratio in (("left", radius), ("right", -radius)):
        assert abs(state["right_arm_pggripper_" + side + "_slider"] - drive * ratio) < 0.002
    print("PASS: both PGGripper sliders follow measured motor feedback.", flush=True)
    for side in ["left", "right"]:
        transform = transforms["lift_" + side + "_main"]
        original = 0.0957499878743 if side == "left" else 0.0955499878744
        assert (
            abs(transform.transform.translation.z - original - state[side + "_lift_joint"]) < 0.002
        )
    print("PASS: lift feedback propagates into the correct TF subtree.", flush=True)
    pub = node.create_publisher(TwistStamped, "diff_drive_controller/cmd_vel", 10)
    wait(lambda: bool(odom))
    initial = odom[-1].pose.pose.position
    start = time.monotonic()
    while time.monotonic() - start < 2:
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.twist.linear.x = 0.1
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
    distance = math.hypot(
        odom[-1].pose.pose.position.x - initial.x, odom[-1].pose.pose.position.y - initial.y
    )
    stop = TwistStamped()
    stop.header.stamp = node.get_clock().now().to_msg()
    pub.publish(stop)
    if args.backend == "mujoco":
        assert distance > 0.04, ("No base odometry motion", distance, velocities)
        print(
            f"PASS: base101 diff-drive command produced {distance:.3f} m odometry motion.",
            flush=True,
        )
    else:
        assert all(
            velocities.get(j, 0) > 1
            for j in [
                "front_left_wheel_joint",
                "front_right_wheel_joint",
                "back_left_wheel_joint",
                "back_right_wheel_joint",
            ]
        )
        print(
            "PASS: base101 diff-drive command reaches all four mock wheel velocity interfaces.",
            flush=True,
        )
    for client in clients:
        client.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
