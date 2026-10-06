#!/usr/bin/env python3
"""Consumer smoke test: robot_state_publisher produces independent lift TF."""

import os
from pathlib import Path
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
import yaml
import rclpy
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[1]


def main():
    os.environ["ROS_DOMAIN_ID"] = "171"
    # Use UDP for this isolated cross-process test; shared-memory discovery
    # can succeed while samples are unavailable in container environments.
    os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    urdf = (ROOT / "tmp" / "handy101.urdf").read_text()
    xml = ET.fromstring(urdf)
    movable = [j.attrib["name"] for j in xml.findall("joint") if j.attrib["type"] != "fixed"]
    origins = {
        j.find("child").attrib["link"]: float(j.find("origin").attrib["xyz"].split()[2])
        for j in xml.findall("joint")
        if j.attrib["name"] in ("left_lift_joint", "right_lift_joint")
    }
    with tempfile.TemporaryDirectory(prefix="handy101-tf-") as td:
        params = Path(td) / "params.yaml"
        params.write_text(
            yaml.safe_dump(
                {"robot_state_publisher": {"ros__parameters": {"robot_description": urdf}}}
            )
        )
        with (ROOT / "tmp" / "robot_state_publisher.log").open("w") as log:
            proc = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "robot_state_publisher",
                    "robot_state_publisher",
                    "--ros-args",
                    "--params-file",
                    str(params),
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            try:
                rclpy.init(domain_id=171)
                node = rclpy.create_node("handy101_lift_tf_check")
                pub = node.create_publisher(JointState, "/joint_states", 10)
                received = {}

                def callback(msg):
                    for tf in msg.transforms:
                        received[tf.child_frame_id] = tf.transform.translation.z

                sub = node.create_subscription(
                    TFMessage,
                    "/tf",
                    callback,
                    QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT),
                )
                for left, right in [(0, 0), (0.225, 0), (0, 0.225), (0.45, 0.45)]:
                    received.clear()
                    deadline = time.monotonic() + 10
                    expected = {
                        "lift_left_main": origins["lift_left_main"] + left,
                        "lift_right_main": origins["lift_right_main"] + right,
                    }
                    while time.monotonic() < deadline:
                        assert proc.poll() is None, "robot_state_publisher exited"
                        msg = JointState()
                        msg.header.stamp = node.get_clock().now().to_msg()
                        msg.name = movable
                        msg.position = [
                            left
                            if n == "left_lift_joint"
                            else right
                            if n == "right_lift_joint"
                            else 0
                            for n in movable
                        ]
                        pub.publish(msg)
                        rclpy.spin_once(node, timeout_sec=0.1)
                        if all(
                            abs(received.get(n, float("inf")) - v) < 1e-9
                            for n, v in expected.items()
                        ):
                            break
                    else:
                        raise AssertionError(
                            (
                                "Lift TF missing or incorrect",
                                expected,
                                received,
                                node.get_node_names_and_namespaces(),
                                node.get_topic_names_and_types(),
                                pub.get_subscription_count(),
                                [str(info) for info in node.get_publishers_info_by_topic("/tf")],
                            )
                        )
                node.destroy_subscription(sub)
                node.destroy_node()
                rclpy.shutdown()
                print("PASS: robot_state_publisher TF at 0, independent 225 mm, and both 450 mm.")
            finally:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()


if __name__ == "__main__":
    main()
