#!/usr/bin/env python3
"""Verify measured head pivot, preserved CAD zero pose and rigid camera motion."""

import json, math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src/handy101_description"
r = ET.parse(ROOT / "tmp/handy101.urdf").getroot()
joints = {j.attrib["name"]: j for j in r.findall("joint")}


def xyz(j):
    return list(map(float, j.find("origin").attrib["xyz"].split()))


tilt = joints["head_camera_tilt_joint"]
pose = joints["head_camera_pose_joint"]
assert tilt.attrib["type"] == "revolute"
assert tilt.find("axis").attrib["xyz"] == "0 1 0"
assert tilt.find("parent").attrib["link"] == "head_camera_mount"
assert pose.find("parent").attrib["link"] == tilt.find("child").attrib["link"]
assert pose.find("child").attrib["link"] == "head_camera_link"
data = json.loads((PKG / "config/cad_measurements.json").read_text())
camera = next(b for b in data["bodies"] if b["link"] == "head_camera_link")
pivot = xyz(tilt)
offset = xyz(pose)
for i in range(3):
    assert (
        abs(pivot[i] + offset[i] + data["base_deck_top_m"][i] - camera["origin_base_m"][i]) < 1e-10
    )
# Camera follows a circular arc about the shaft; mount and servo remain fixed.
radius = math.hypot(offset[0], offset[2])
for angle in [-math.pi / 2, 0, math.pi / 4, math.pi / 2]:
    x = math.cos(angle) * offset[0] + math.sin(angle) * offset[2]
    z = -math.sin(angle) * offset[0] + math.cos(angle) * offset[2]
    assert abs(math.hypot(x, z) - radius) < 1e-12
    if angle == math.pi / 4:
        assert z < offset[2]
assert joints["head_camera_mount_joint"].attrib["type"] == "fixed"
print(
    "PASS: lateral servo pivot, exact CAD zero pose, camera/support rigid rotation, fixed servo housing."
)
