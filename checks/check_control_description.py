#!/usr/bin/env python3
"""Check interface ownership, controller coverage, and source joint limits."""

from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
r = ET.parse(ROOT / "tmp/handy101-mujoco.urdf").getroot()
systems = r.findall("ros2_control")
assert len(systems) == 1
assert systems[0].findtext("hardware/plugin") == "mujoco_ros2_control/MujocoSystemInterface"
interfaces = {j.attrib["name"]: j for j in systems[0].findall("joint")}
joints = {j.attrib["name"]: j for j in r.findall("joint") if j.attrib["type"] != "fixed"}
assert set(interfaces) == set(joints) and len(joints) == 22
for name, interface in interfaces.items():
    commands = interface.findall("command_interface")
    if joints[name].find("mimic") is not None:
        assert not commands
        continue
    assert len(commands) == 1
    assert {i.attrib["name"] for i in interface.findall("state_interface")} == {
        "position",
        "velocity",
    }
    wheel = "wheel" in name
    assert commands[0].attrib["name"] == ("velocity" if wheel else "position")
    if not wheel:
        bounds = {p.attrib["name"]: float(p.text) for p in commands[0].findall("param")}
        limit = joints[name].find("limit").attrib
        assert bounds["min"] == float(limit["lower"]) and bounds["max"] == float(limit["upper"]), (
            name
        )
cfg = yaml.safe_load((ROOT / "src/handy101_control/config/controllers.yaml").read_text())
claimed = []
for name, settings in cfg.items():
    if name == "controller_manager" or name.endswith("_position_controller") or name == "imu_broadcaster":
        continue
    claimed.extend(settings["ros__parameters"]["joints"])
assert len(claimed) == len(set(claimed)) == 16
assert set(claimed) == {n for n in joints if "wheel" not in n and joints[n].find("mimic") is None}
# Wheel interface template remains base-owned and is included rather than copied.
control = (ROOT / "src/handy101_control/urdf/handy101.ros2control.xacro").read_text()
assert "<xacro:base101_wheel_interfaces/>" in control
print(
    "PASS: one hardware system, 20 driven joints + 2 passive mimic joints uniquely owned, controller coverage and exact source position bounds."
)
