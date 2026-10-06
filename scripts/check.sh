#!/usr/bin/env bash
set -euo pipefail
# The caller sources ROS and workspace before this script.
python3 tools/expand_description.py --bundle-meshes
check_urdf tmp/handy101.urdf > tmp/check_urdf.txt
python3 checks/check_lifts.py
python3 checks/check_arms.py
python3 checks/check_head_tilt.py
python3 checks/check_rosboard_serialization.py
python3 tools/expand_description.py --output tmp/handy101-mujoco.urdf control_backend:=mujoco
python3 checks/check_control_description.py
python3 - <<'PY'
from pathlib import Path
from handy101_mujoco.scene import build_scene
urdf = Path('tmp/handy101-mujoco.urdf')
build_scene(urdf.read_text(), 'tmp/mujoco-check/scene.xml', urdf_dir=urdf.parent.resolve())
PY
/opt/handy101-checks/bin/python checks/check_mujoco.py \
    --scene tmp/mujoco-check/scene.xml --urdf tmp/handy101-mujoco.urdf

python3 tools/expand_description.py --output tmp/home/robot.urdf control_backend:=mujoco
python3 - <<'PYHOME'
from pathlib import Path
from handy101_mujoco.scene import build_scene
from handy101_mujoco.world import add_home, default_assets
path = Path('tmp/home/scene.xml')
build_scene(Path('tmp/home/robot.urdf').read_text(), path)
add_home(path, default_assets())
PYHOME
/opt/handy101-checks/bin/python checks/check_home.py
