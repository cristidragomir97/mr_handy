"""Apply Handy101 integration support to Docker's downloaded dependency copies.

Upstream geometry stays authoritative. These compatibility changes can be removed
when base101 publishes upper_deck selection and rosboard sanitizes JSON feedback.
"""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path('/opt/handy101_dependencies/src')
XACRO = 'http://www.ros.org/wiki/xacro'


def prepare_chassis():
    path = ROOT / 'base101_description/urdf/chassis.xacro'
    text = path.read_text()
    root = ET.fromstring(text)
    if any(e.attrib.get('name') == 'upper_deck' for e in root.iter(f'{{{XACRO}}}arg')):
        return
    # The lift assembly replaces this standalone raised deck and its standoffs.
    # Preserve their authored geometry for upper_deck=true, never invent inertia.
    names = {e.attrib['name'] for e in root.findall('link')
             if e.attrib['name'].startswith('standoff_') or e.attrib['name'] == 'top_plate_1'}
    if len(names) != 5:
        raise RuntimeError('base101 deck layout changed; review compatibility adapter')
    joints = {e.attrib['name'] for e in root.findall('joint')
              if e.find('child').attrib['link'] in names}
    for joint in root.findall('joint'):
        if joint.find('parent').attrib['link'] in names and joint.attrib['name'] not in joints:
            raise RuntimeError('base101 added a deck child; review compatibility adapter')
    selected = names | joints
    count = 0

    def wrap(match):
        nonlocal count
        name = re.search(r'\bname="([^"]+)"', match.group()).group(1)
        if name not in selected:
            return match.group()
        count += 1
        return '<xacro:if value="$(arg upper_deck)">\n' + match.group() + '\n</xacro:if>'

    text = re.sub(r'<(link|joint)\b[^>]*(?<!/)>.*?</\1>', wrap, text, flags=re.S)
    if count != 10:
        raise RuntimeError('Could not identify all base101 deck elements')
    text = re.sub(r'(<robot\b[^>]*>)', r'\1\n  <xacro:arg name="upper_deck" default="true"/>', text, count=1)
    ET.fromstring(text)
    path.write_text(text)


def prepare_rosboard():
    path = ROOT / 'rosboard/rosboard/handlers.py'
    text = path.read_text()
    if 'def json_safe(' in text:
        return
    helper = '''import math


def json_safe(value):
    """Represent unavailable ROS feedback as JSON null, without mutating it."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value

'''
    target = "json.dumps(message, separators=(',', ':'))"
    if text.count(target) != 2:
        raise RuntimeError('rosboard broadcast changed; review JSON adapter')
    text = text.replace(target, "json.dumps(json_safe(message), separators=(',', ':'), allow_nan=False)")
    text = text.replace('import json\n', 'import json\n' + helper, 1)
    compile(text, str(path), 'exec')
    path.write_text(text)


def prepare_slam():
    """Allow an assembly to override the EKF while retaining base101's launch."""
    path = ROOT / 'base101_slam/launch/slam.launch.py'
    text = path.read_text()
    if "LaunchConfiguration('ekf_config')" in text:
        return
    target = "parameters=[ekf_config, {'use_sim_time': use_sim_time}],"
    if text.count(target) != 1:
        raise RuntimeError('base101 EKF launch changed; review config override adapter')
    text = text.replace(target,
        "parameters=[ekf_config] + ([LaunchConfiguration('ekf_config').perform(context)] "
        "if LaunchConfiguration('ekf_config').perform(context) else []) + "
        "[{'use_sim_time': use_sim_time}],")
    anchor = "        OpaqueFunction(function=_setup),"
    if text.count(anchor) != 1:
        raise RuntimeError('base101 SLAM launch changed; review config override adapter')
    text = text.replace(anchor,
        "        DeclareLaunchArgument('ekf_config', default_value='', "
        "description='Assembly-specific EKF parameter overrides'),\n" + anchor)
    compile(text, str(path), 'exec')
    path.write_text(text)


if __name__ == '__main__':
    prepare_chassis()
    prepare_rosboard()
    prepare_slam()
