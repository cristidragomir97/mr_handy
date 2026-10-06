#!/usr/bin/env bash
# Hold the lock in the ROS launch process and its children until shutdown finishes.
set -euo pipefail
exec 9>/tmp/handy101-robot.lock
if ! flock -n 9; then
    printf 'A handy101 robot stack is already running in this container.\nStop it with Ctrl+C in its terminal, or run ./scripts/manage.sh stop.\n' >&2
    exit 1
fi
exec "$@"
