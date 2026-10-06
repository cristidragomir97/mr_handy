#!/usr/bin/env python3
"""Verify imported SLAM/Nav2 end to end; --navigate also drives a short goal."""
import argparse
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--navigate', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('handy101_navigation_check', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    samples = {}
    subscriptions = [
        node.create_subscription(LaserScan, '/scan_filtered',
                                 lambda msg: samples.update(scan=msg), qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid, '/map', lambda msg: samples.update(map=msg),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]

    def wait(predicate, seconds=60):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if predicate():
                return
        raise AssertionError('Timed out waiting for mapping/navigation')

    def result(future, seconds=60):
        wait(future.done, seconds)
        return future.result()

    try:
        wait(lambda: 'scan' in samples and 'map' in samples and
             buffer.can_transform('map', 'base_link', Time()))
        scan = samples['scan']
        assert any(scan.range_min <= r <= scan.range_max for r in scan.ranges)
        grid = samples['map']
        assert grid.info.width > 0 and grid.info.height > 0 and 0 in grid.data
        assert buffer.can_transform('map', 'odom', Time())
        for name in ('slam_toolbox', 'planner_server', 'controller_server',
                     'bt_navigator', 'velocity_smoother'):
            client = node.create_client(GetState, f'/{name}/get_state')
            assert client.wait_for_service(timeout_sec=20), name
            deadline = time.monotonic() + 30
            while True:
                state = result(client.call_async(GetState.Request())).current_state
                if state.label == 'active':
                    break
                assert time.monotonic() < deadline, (name, state.label)
                rclpy.spin_once(node, timeout_sec=0.2)
            node.destroy_client(client)
        print('PASS: filtered lidar, populated map, map TF and all SLAM/Nav2 lifecycle nodes active.', flush=True)
        planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        assert planner.wait_for_server(timeout_sec=20)
        transform = buffer.lookup_transform('map', 'base_link', Time()).transform
        path_goal = None
        # Try nearby destinations; a furnished world may block one direction.
        for dx, dy in ((0.6, 0), (0, 0.6), (-0.6, 0), (0, -0.6)):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x = transform.translation.x + dx
            pose.pose.position.y = transform.translation.y + dy
            pose.pose.orientation.z = math.sin(math.atan2(dy, dx) / 2)
            pose.pose.orientation.w = math.cos(math.atan2(dy, dx) / 2)
            goal = ComputePathToPose.Goal()
            goal.goal = pose
            handle = result(planner.send_goal_async(goal))
            if not handle.accepted:
                continue
            response = result(handle.get_result_async())
            if response.status == GoalStatus.STATUS_SUCCEEDED and response.result.path.poses:
                path_goal = pose
                break
        assert path_goal is not None, 'No nearby goal produced a path'
        print('PASS: Nav2 computes a nonempty path in the live SLAM map.', flush=True)
        if args.navigate:
            navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
            assert navigator.wait_for_server(timeout_sec=20)
            goal = NavigateToPose.Goal()
            goal.pose = path_goal
            handle = result(navigator.send_goal_async(goal))
            assert handle.accepted
            try:
                response = result(handle.get_result_async(), 90)
            except Exception:
                result(handle.cancel_goal_async(), 10)
                raise
            assert response.status == GoalStatus.STATUS_SUCCEEDED, response
            final = buffer.lookup_transform('map', 'base_link', Time()).transform.translation
            assert math.hypot(final.x - path_goal.pose.position.x,
                              final.y - path_goal.pose.position.y) < 0.3
            print('PASS: NavigateToPose succeeds and measured pose reaches the nearby goal.', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
