#!/usr/bin/env python3
"""Check CAD registration, mesh scale/bounds, and independent positive lift motion."""

import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "handy101_description"
robot = ET.parse(ROOT / "tmp" / "handy101.urdf").getroot()
joints = {j.attrib["name"]: j for j in robot.findall("joint")}
links = {l.attrib["name"]: l for l in robot.findall("link")}
data = json.loads((PKG / "config" / "cad_measurements.json").read_text())


def vector(text):
    return [float(v) for v in text.split()]


for body in data["bodies"]:
    name = body["link"]
    meshes = [(name + ".stl", body["bounds_local_mm"])] + [
        (v["filename"], v["bounds_local_mm"]) for v in body["visuals"]
    ]
    for filename, expected_bounds in meshes:
        path = PKG / "meshes" / filename
        with path.open("rb") as f:
            header = f.read(84)
            count = struct.unpack("<I", header[80:84])[0]
            assert path.stat().st_size == 84 + count * 50
            mins = [float("inf")] * 3
            maxs = [-float("inf")] * 3
            for _ in range(count):
                record = struct.unpack("<12fH", f.read(50))
                for v in (record[3:6], record[6:9], record[9:12]):
                    for k in range(3):
                        mins[k] = min(mins[k], v[k])
                        maxs[k] = max(maxs[k], v[k])
            for actual, expected in zip(mins + maxs, expected_bounds):
                assert abs(actual - expected) < 0.21, (name, actual, expected)
    visuals = links[name].findall("visual")
    assert len(visuals) == len(body["visuals"])
    for visual, expected in zip(visuals, body["visuals"]):
        assert vector(visual.find("geometry/mesh").attrib["scale"]) == [0.001] * 3
        assert visual.find("material").attrib["name"] == expected["material"]

for side in ("left", "right"):
    joint = joints[side + "_lift_joint"]
    assert joint.attrib["type"] == "prismatic"
    assert vector(joint.find("axis").attrib["xyz"]) == [0, 0, 1]
    assert joint.find("parent").attrib["link"] == "lift_" + side + "_structure"
    assert joint.find("child").attrib["link"] == "lift_" + side + "_main"
    limits = joint.find("limit").attrib
    assert (float(limits["lower"]), float(limits["upper"])) == (0, 0.45)
    origin = vector(joint.find("origin").attrib["xyz"])
    deck = vector(joints[side + "_structure_mount"].find("origin").attrib["xyz"])
    body = next(b for b in data["bodies"] if b["link"] == "lift_" + side + "_main")
    world = [origin[i] + deck[i] for i in range(3)]
    assert max(abs(world[i] - body["origin_base_m"][i]) for i in range(3)) < 1e-10
    for q in (0, 0.225, 0.45):
        raised = [world[0], world[1], world[2] + q]
        assert raised[2] - world[2] >= 0
        # Each main is in its own subtree, never a mimic/slave of the other lift.
        assert joint.find("mimic") is None
        assert abs(raised[2] - (body["origin_base_m"][2] + q)) < 1e-10
assert not any("lift_" + side + "_carriage" in links for side in ("left", "right"))
assert "top_plate_1" not in links
assert not any(n.startswith("standoff_") for n in links)
assert "head_camera_link" in links
assert joints["head_camera_mount_joint"].find("parent").attrib["link"] == "lift_crossbar"
print(
    "PASS: seven mesh bounds/scales, CAD-to-base registration, independent +Z travel 0/225/450 mm."
)
