#!/usr/bin/env python3
"""Compare encoder/fused/map poses to physics; --drive runs simulated motions.

Wheel slip leaves residual translation drift in local odometry. SLAM's map pose
is checked independently when available; ground truth is never an estimator input.
"""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def yaw(quaternion):
    q = quaternion
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def pose_error(rows, truth):
    estimate = np.array(rows)
    estimate[:, 3] = np.unwrap(estimate[:, 3])
    estimate = estimate[(estimate[:, 0] >= truth[0, 0]) &
                        (estimate[:, 0] <= truth[-1, 0])]
    assert len(estimate) > 10, 'Insufficient synchronized samples'
    reference = np.column_stack([
        np.interp(estimate[:, 0], truth[:, 0], truth[:, i]) for i in range(1, 4)])
    # Odom and world/map may have different initial origins and headings.
    theta = reference[0, 2] - estimate[0, 3]
    rotation = np.array([[math.cos(theta), -math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]])
    xy = (estimate[:, 1:3] - estimate[0, 1:3]) @ rotation.T
    xy -= reference[:, :2] - reference[0, :2]
    angular = (estimate[:, 3] - estimate[0, 3]) - (reference[:, 2] - reference[0, 2])
    return dict(max_xy_m=float(np.linalg.norm(xy, axis=1).max()),
                max_yaw_rad=float(np.abs(angular).max()),
                yaw_motion_estimate=float(estimate[-1, 3] - estimate[0, 3]),
                yaw_motion_truth=float(reference[-1, 2] - reference[0, 2]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('handy101_odometry_check', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    samples = {name: [] for name in ('wheel', 'truth', 'filtered', 'map', 'scan')}
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    def odometry(name, message):
        p = message.pose.pose
        samples[name].append([stamp(message), p.position.x, p.position.y,
                              yaw(p.orientation)])

    def scan(message):
        samples['scan'].append([stamp(message), node.get_clock().now().nanoseconds*1e-9,
                                message.time_increment, message.scan_time, len(message.ranges)])

    subscriptions = [node.create_subscription(
        Odometry, topic, lambda msg, name=name: odometry(name, msg), qos_profile_sensor_data)
        for name, topic in [('wheel', '/diff_drive_controller/odom'),
                            ('truth', '/simulation/ground_truth_odom'),
                            ('filtered', '/odometry/filtered')]]
    subscriptions.append(node.create_subscription(
        LaserScan, '/scan_filtered', scan, qos_profile_sensor_data))
    publisher = node.create_publisher(TwistStamped, '/cmd_vel_key', 10)

    def command(linear=0.0, angular=0.0):
        message = TwistStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.twist.linear.x = float(linear)
        message.twist.angular.z = float(angular)
        publisher.publish(message)

    def spin():
        rclpy.spin_once(node, timeout_sec=0.02)
        try:
            transform = buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return
        t = stamp(transform)
        if not samples['map'] or t > samples['map'][-1][0]:
            p = transform.transform
            samples['map'].append([t, p.translation.x, p.translation.y, yaw(p.rotation)])

    def sample(seconds, linear=0.0, angular=0.0):
        start = node.get_clock().now().nanoseconds*1e-9
        wall = time.monotonic()
        while node.get_clock().now().nanoseconds*1e-9 - start < seconds:
            assert time.monotonic() - wall < seconds*5 + 10, 'Simulation clock stopped'
            if args.drive:
                command(linear, angular)
            spin()

    try:
        deadline = time.monotonic() + 20
        while not samples['truth'] or not samples['filtered']:
            assert time.monotonic() < deadline, 'No ground-truth/fused odometry'
            spin()
        if args.drive:
            print('Testing reverse straight/curve and a full turn in simulation.', flush=True)
            # Move away from the prep table beside the default kitchen spawn.
            sample(3, -0.12); sample(6, -0.10, 0.4); sample(10, 0.0, 0.6); sample(2)
        else:
            sample(12)
    finally:
        if args.drive:
            command()
        node.destroy_node()
        rclpy.shutdown()

    truth = np.array(samples['truth'])
    truth[:, 3] = np.unwrap(truth[:, 3])
    errors = {name: pose_error(samples[name], truth) for name in ('wheel', 'filtered')}
    if len(samples['map']) > 10:
        errors['map'] = pose_error(samples['map'], truth)
    for name, error in errors.items():
        print(name, json.dumps(error), flush=True)
    if samples['scan']:
        scans = np.array(samples['scan'])
        print('Lidar timestamp delay [min, median, max] seconds:',
              np.quantile(scans[:, 1] - scans[:, 0], [0, .5, 1]).tolist())
    output = Path(__file__).resolve().parents[1] / 'tmp/odometry-check.json'
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(dict(samples=samples, errors=errors)))
    if args.drive:
        assert abs(errors['filtered']['yaw_motion_truth']) > .5, 'Insufficient physical turning'
        assert errors['filtered']['max_yaw_rad'] < .06, errors
        # Encoders cannot measure lateral skidding; lidar must correct this drift.
        assert errors['filtered']['max_xy_m'] < .20, errors
        if 'map' in errors:
            assert errors['map']['max_xy_m'] < .10, errors
            assert errors['map']['max_yaw_rad'] < .06, errors
        print('PASS: IMU-based turning odometry and available SLAM pose track independent physics.')


if __name__ == '__main__':
    main()
