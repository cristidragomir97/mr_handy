#!/usr/bin/env python3
"""Check the requested dual-arm configuration and measured mounting datums."""

import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
r = ET.parse(ROOT / "tmp/handy101.urdf").getroot()
links = {l.attrib["name"]: l for l in r.findall("link")}
joints = {j.attrib["name"]: j for j in r.findall("joint")}
assert len(links) == len(r.findall("link"))
assert len(joints) == len(r.findall("joint"))
measurements = json.loads(
    (ROOT / "src/handy101_description/config/arm_mount_measurements.json").read_text()
)
for mount in measurements["mounts"]:
    p = mount["prefix"]
    joint = joints[p + "base_mount"]
    assert joint.find("parent").attrib["link"] == mount["parent"]
    xyz = list(map(float, joint.find("origin").attrib["xyz"].split()))
    assert max(abs(a - b) for a, b in zip(xyz, mount["xyz_m"])) < 1e-10
    assert max(map(abs, mount["shoulder_residual_bounds_mm"])) < 0.03
    assert links[p + "base_link"].find("visual") is None
    assert links[p + "base_link"].find("inertial") is None
    assert p + "base_cover_1" not in links
    assert p + "servo_base_1" in links
    for suffix, length in [
        ("arm_extrusion_1", 0.08),
        ("forearm_extrusion_1", 0.1),
    ]:
        assert (
            float(links[p + suffix].find("visual/geometry/box").attrib["size"].split()[0]) == length
        )
    # Every arm descendant has a path through only its own lift joint.
    by_child = {j.find("child").attrib["link"]: j for j in joints.values()}
    for name in links:
        if not name.startswith(p):
            continue
        ancestors = []
        while name in by_child:
            ancestor = by_child[name]
            ancestors.append(ancestor.attrib["name"])
            name = ancestor.find("parent").attrib["link"]
        assert mount["side"] + "_lift_joint" in ancestors
        assert ("right" if mount["side"] == "left" else "left") + "_lift_joint" not in ancestors
for side in ("left", "right"):
    assert side + "_arm_joint_wrist_yaw" in joints
assert "left_arm_6" not in joints
assert "left_arm_camera_tool_primary" in links
assert float(links["left_arm_camera_tool_primary"].find("inertial/mass").get("value")) == .061
assert "luxonis.stl" in links["left_arm_camera_tool_primary"].find("visual/geometry/mesh").get("filename")
assert "left_arm_camera_tool_gopro" not in links
assert "left_arm_camera_tool_gopro_optical_frame" not in links
assert "right_arm_pggripper_body" in links
assert "right_arm_camera_tool_primary" not in links
assert joints["right_arm_6"].get("type") == "revolute"
mimics = [j for j in joints.values() if j.find("mimic") is not None]
assert len(mimics) == 2
for j in mimics:
    assert j.find("mimic").get("joint") == "right_arm_6"
    assert abs(float(j.find("mimic").get("multiplier")) * float(joints["right_arm_6"].find("limit").get("upper")) - (.027 if "left_slider" in j.get("name") else -.027)) < 1e-10
assert "right_arm_wrist_camera_optical_frame" not in links
assert "left_arm_wrist_camera_optical_frame" not in links
assert "head_camera_link" in links
print(
    "PASS: CAD mounts, independent lift ancestry, 7DOF arms, both 80/100mm, left Luxonis, right PGGripper, no wrist cameras."
)
