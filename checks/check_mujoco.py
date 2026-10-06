#!/usr/bin/env python3
"""Check derived scene FK against URDF and physics under independent commands."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=Path("tmp/mujoco-live/scene.xml"))
    parser.add_argument("--urdf", type=Path, default=Path("tmp/mujoco-live/robot.urdf"))
    args = parser.parse_args()
    robot = ET.parse(args.urdf).getroot()
    joints = {j.attrib["name"]: j for j in robot.findall("joint")}
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)}
    movable = {n for n, j in joints.items() if j.attrib["type"] != "fixed"}
    assert names == {n for n in movable if joints[n].find("mimic") is None}
    assert model.neq == 2
    urdf_mass = sum(float(i.find("mass").attrib["value"]) for i in robot.findall("link/inertial"))
    assert abs(sum(model.body_mass) - urdf_mass) < 1e-8
    children = {}
    for j in joints.values():
        children.setdefault(j.find("parent").attrib["link"], []).append(j)
    q = {
        n: (0.225 if j.attrib["type"] == "prismatic" else 0.2)
        for n, j in joints.items()
        if j.attrib["type"] != "fixed"
    }
    for n, j in joints.items():
        mimic = j.find("mimic")
        if mimic is not None:
            q[n] = q[mimic.attrib["joint"]] * float(mimic.attrib.get("multiplier", "1")) + float(
                mimic.attrib.get("offset", "0")
            )
    for n, value in q.items():
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
        data.qpos[model.jnt_qposadr[index]] = value
    mujoco.mj_forward(model, data)
    initial = np.eye(4)
    initial[:3, 3] = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")]
    initial[:3, :3] = data.xmat[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    ].reshape(3, 3)
    checked = []
    frame_poses = {"base_link": initial}

    def fk(parent, T):
        for joint in children.get(parent, []):
            o = joint.find("origin")
            relative = np.eye(4)
            if o is not None:
                relative[:3, 3] = np.fromstring(o.attrib.get("xyz", "0 0 0"), sep=" ")
                relative[:3, :3] = Rotation.from_euler(
                    "xyz", np.fromstring(o.attrib.get("rpy", "0 0 0"), sep=" ")
                ).as_matrix()
            motion = np.eye(4)
            name = joint.attrib["name"]
            if joint.attrib["type"] != "fixed":
                axis = np.fromstring(joint.find("axis").attrib["xyz"], sep=" ")
                axis /= np.linalg.norm(axis)
                if joint.attrib["type"] == "prismatic":
                    motion[:3, 3] = axis * q[name]
                else:
                    motion[:3, :3] = Rotation.from_rotvec(axis * q[name]).as_matrix()
            current = T @ relative @ motion
            child = joint.find("child").attrib["link"]
            frame_poses[child] = current
            if joint.attrib["type"] != "fixed":
                index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, child)
                assert index >= 0, child
                assert np.max(np.abs(data.xpos[index] - current[:3, 3])) < 1e-9, child
                assert np.max(np.abs(data.xmat[index].reshape(3, 3) - current[:3, :3])) < 1e-9, (
                    child
                )
                checked.append(name)
            fk(child, current)

    fk("base_link", initial)
    assert len(checked) == 22
    print(
        "PASS: all 22 joint frames match independent URDF FK at mixed poses; total mass conserved."
    )
    camera_frames = {
        "base_camera": "camera_optical_frame",
        "head_camera": "head_camera_optical_frame",
        "left_arm_camera": "left_arm_camera_tool_optical_frame",
    }
    assert model.ncam == 3
    for name, frame in camera_frames.items():
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        expected = frame_poses[frame]
        actual = data.cam_xmat[index].reshape(3, 3)
        np.testing.assert_allclose(data.cam_xpos[index], expected[:3, 3], atol=1e-10)
        np.testing.assert_allclose(-actual[:, 2], expected[:3, 2], atol=1e-10)
        np.testing.assert_allclose(actual[:, 1], -expected[:3, 1], atol=1e-10)
        np.testing.assert_allclose(actual[:, 0], expected[:3, 0], atol=1e-10)
    imu = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "base_imu_site")
    np.testing.assert_allclose(data.site_xpos[imu], frame_poses["imu_link"][:3, 3], atol=1e-10)
    for ray in range(360):
        site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "base_lidar-"+str(ray))
        expected = frame_poses["lidar_frame"]
        angle = -np.pi + ray * 2 * np.pi / 359
        direction = expected[:3, :3] @ [np.cos(angle), np.sin(angle), 0]
        np.testing.assert_allclose(data.site_xpos[site], expected[:3, 3], atol=1e-10)
        np.testing.assert_allclose(data.site_xmat[site].reshape(3, 3)[:, 2], direction, atol=1e-10)
    print("PASS: all three lens positions/view/up/right axes and 360 lidar rays match independent URDF FK after fixed-body fusion.")
    mujoco.mj_resetData(model, data)
    goals = {n: 0.0 for n in names}
    goals.update(
        left_lift_joint=0.12,
        right_lift_joint=0.04,
        head_camera_tilt_joint=0.3,
        left_arm_joint_shoulder=0.5,
        left_arm_joint_elbow=0.7,
        left_arm_joint_wrist_yaw=0.2,
        right_arm_joint_shoulder=0.4,
        right_arm_joint_elbow=0.6,
        right_arm_joint_wrist_yaw=-0.2,
        right_arm_6=1.2,
    )
    for name, value in goals.items():
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)] = value
    for _ in range(2000):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    assert np.max(np.abs(data.qvel)) < 0.02, np.max(np.abs(data.qvel))
    for name, value in goals.items():
        if "wheel" in name:
            continue
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        actual = data.qpos[model.jnt_qposadr[index]]
        assert abs(actual - value) < 0.015, (name, actual, value)
    for name in movable - names:
        mimic = joints[name].find("mimic")
        parent = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, mimic.attrib["joint"])
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        expected = data.qpos[model.jnt_qposadr[parent]] * float(mimic.attrib["multiplier"])
        assert abs(data.qpos[model.jnt_qposadr[index]] - expected) < 0.002, name
    print("PASS: both PGGripper sliders follow their motor drive.")
    assert abs(data.qpos[2]) < 0.005 and abs(data.qpos[3]) > 0.99
    print(
        "PASS: 4 seconds gravity/contact physics; independent lifts, arms, tools and head settle within 0.015 of target."
    )
    # Differential commands reach native wheel actuators and move the floating base.
    before = data.qpos[:2].copy()
    for name in names:
        if "wheel" in name:
            data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)] = 2.0
    for _ in range(1000):
        mujoco.mj_step(model, data)
    assert np.linalg.norm(data.qpos[:2] - before) > 0.05
    print("PASS: wheel velocity actuators produce floating-base motion through floor contact.")


if __name__ == "__main__":
    main()
