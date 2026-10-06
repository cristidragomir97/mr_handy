#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/handy101_ws/install/setup.bash
if [[ -f /workspace/handy101/install/setup.bash ]]; then
    source /workspace/handy101/install/setup.bash
fi
exec "$@"
