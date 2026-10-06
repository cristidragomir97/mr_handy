#!/usr/bin/env python3
"""Measure encoder odometry against independent floating-base physics."""
import argparse
import json
import math
import numpy as np
import mujoco


def yaw(q):
    w, x, y, z = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', default='tmp/mujoco-check/scene.xml')
    parser.add_argument("--friction", type=float)
    parser.add_argument("--separation-multiplier", type=float, default=5.5)
    parser.add_argument("--elliptic", action="store_true")
    parser.add_argument("--torsion", type=float)
    parser.add_argument('--duration', type=float, default=6.0)
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(args.scene)
    if args.friction is not None:
        model.geom_friction[:, 0] = args.friction
    if args.elliptic:
        model.opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    if args.torsion is not None:
        model.geom_friction[:, 1] = args.torsion
    free = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'floating_base_joint')
    base = model.jnt_qposadr[free]
    wheels = ['front_left_wheel_joint', 'back_left_wheel_joint',
              'front_right_wheel_joint', 'back_right_wheel_joint']
    joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in wheels]
    addresses = model.jnt_qposadr[joints]
    actuators = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in wheels]
    radius, separation = 0.0363, 0.2899 * args.separation_multiplier
    for linear, angular in [(0.15, 0), (0, 0.4), (0.10, 0.3)]:
        data = mujoco.MjData(model)
        # Match ros2_control's authored zero-pose hold on every driven axis.
        for _ in range(round(1/model.opt.timestep)):
            mujoco.mj_step(model, data)
        start = data.qpos[base:base+7].copy()
        previous_wheels = data.qpos[addresses].copy()
        wheel_pose = np.zeros(3)
        truth_yaw = previous_yaw = yaw(start[3:7])
        left = (linear-angular*separation/2)/radius
        right = (linear+angular*separation/2)/radius
        for step in range(round(args.duration/model.opt.timestep)):
            ramp = min(1.0, step*model.opt.timestep/.5)
            data.ctrl[actuators] = np.array([left,left,right,right])*ramp
            mujoco.mj_step(model,data)
            current_wheels = data.qpos[addresses].copy()
            dl, dr = np.mean(current_wheels[:2]-previous_wheels[:2])*radius, np.mean(current_wheels[2:]-previous_wheels[2:])*radius
            ds, da = (dl+dr)/2, (dr-dl)/separation
            wheel_pose[:2] += ds*np.array([math.cos(wheel_pose[2]+da/2), math.sin(wheel_pose[2]+da/2)])
            wheel_pose[2] += da
            previous_wheels = current_wheels
            current_yaw = yaw(data.qpos[base+3:base+7])
            truth_yaw += math.atan2(math.sin(current_yaw-previous_yaw), math.cos(current_yaw-previous_yaw))
            previous_yaw = current_yaw
        theta = yaw(start[3:7]); R=np.array([[math.cos(theta),math.sin(theta)],[-math.sin(theta),math.cos(theta)]])
        truth_xy = R@(data.qpos[base:base+2]-start[:2])
        actual_yaw = truth_yaw-yaw(start[3:7])
        print(json.dumps(dict(command=[linear,angular],encoder=wheel_pose.tolist(),truth=[*truth_xy,actual_yaw],xy_error=float(np.linalg.norm(truth_xy-wheel_pose[:2])),yaw_error=wheel_pose[2]-actual_yaw,turn_ratio=actual_yaw/wheel_pose[2] if abs(wheel_pose[2])>.01 else None)),flush=True)


if __name__=='__main__':
    main()
