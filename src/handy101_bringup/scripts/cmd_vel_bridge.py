#!/usr/bin/env python3
"""Stamp conventional Twist input before entering the base-owned priority mux."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

class CommandBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel_key', 10)
        self.create_subscription(Twist, '/cmd_vel', self.forward, 10)
        # Base101's Gazebo consumers use these existing image topic names.
        self.alias_subscriptions = []
        self.alias_publishers = []
        for source, targets in (
            ('/base_camera/color/image_raw', ('/base_camera/image_raw', '/base_camera_plugin/base_camera/color')),
            ('/base_camera/depth/image_raw', ('/base_camera/depth_image', '/base_camera_plugin/base_camera/depth')),
        ):
            for target in targets:
                publisher = self.create_publisher(Image, target, qos_profile_sensor_data)
                self.alias_publishers.append(publisher)
                self.alias_subscriptions.append(self.create_subscription(Image, source, publisher.publish, qos_profile_sensor_data))
    def forward(self, message):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = message
        self.publisher.publish(stamped)

if __name__ == '__main__':
    rclpy.init()
    node = CommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
