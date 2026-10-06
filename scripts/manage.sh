#!/usr/bin/env bash
# Simple container lifecycle + ROS commands, following grove-g1's entry point.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ -f .env ]]; then set -a; source .env; set +a; fi
export BUILDX_BUILDER="${BUILDX_BUILDER:-default}"
compose=(docker compose -f "$ROOT/docker-compose.yml")
# WSLg exposes the Windows GPU through DXG/D3D12; Linux uses DRM render devices.
if [[ -c /dev/dxg && -f /usr/lib/wsl/lib/libd3d12.so ]]; then
    compose+=(-f "$ROOT/docker-compose.wsl.yml")
elif [[ -d /dev/dri ]]; then
    compose+=(-f "$ROOT/docker-compose.dri.yml")
fi
# argv is passed verbatim through bash; never interpolate user commands as code.
in_container() {
    local tty=()
    [[ -t 0 && -t 1 ]] || tty=(-T)
    "${compose[@]}" exec "${tty[@]}" ros-dev /usr/local/bin/handy101-entrypoint "$@"
}
build() {
    local select=(--packages-select base101_description base101_control rosboard
        mod101_tool_jaws mod101_tool_none mod101_tool_parallel
        mod101_tool_pincopen mod101_tool_camera mod101_tool_pggripper mod101_description handy101_description handy101_control
        handy101_bringup handy101_mujoco)
    (( $# == 0 )) || select=(--packages-select "$@")
    in_container colcon build --symlink-install --parallel-workers 2 \
        --base-paths src /workspace/base101/src /workspace/mod101/src \
        "${select[@]}" --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
}
usage() {
cat <<'HELP'
Usage: ./scripts/manage.sh <command> [args]
  start                 Build image and start development container + optional router.
  stop                  Stop this project's containers.
  restart               Restart this project's containers.
  recreate              Rebuild/recreate containers; preserve source and build volumes.
  logs                  Follow container logs.
  exec [cmd...]         Run a command or open a sourced shell.
  build [pkg...]        Build this assembly, or named ROS packages.
  sim [launch args...]  Run MuJoCo + ROS controllers + rosboard (localhost:8888).
  mock [launch args...] Run mock ROS controllers + rosboard.
  rosboard [args...]    Observe an externally launched robot's ROS graph.
  world-assets          Download/prepare attributed RoboCasa furniture and objects.
  test                  Run description/control/physics checks (no running sim needed).
  test --sim            Check ROS actions/TF/drive and rosboard against an active MuJoCo stack.
HELP
}
action=${1:-help}; (( $# == 0 )) || shift
case "$action" in
    start|recreate)
        bash scripts/setup-world-assets.sh
        "${compose[@]}" build ros-dev
        options=(-d); [[ "$action" != recreate ]] || options+=(--force-recreate)
        services=(ros-dev)
        [[ "${HANDY101_START_ROUTER:-true}" != true ]] || services+=(router)
        "${compose[@]}" --profile router up "${options[@]}" "${services[@]}"
        printf 'Container ready. Run ./scripts/manage.sh sim (rosboard: http://localhost:8888).\n'
        ;;
    stop) "${compose[@]}" --profile router stop ;;
    restart) "${compose[@]}" --profile router restart ;;
    logs) "${compose[@]}" --profile router logs -f ;;
    exec) (( $# != 0 )) || set -- bash; in_container "$@" ;;
    build) build "$@" ;;
    sim)
        gui=false
        [[ "${HANDY101_HEADLESS:-false}" != false ]] || gui=true
        for arg in "$@"; do
            case "$arg" in headless:=false) gui=true ;; headless:=true) gui=false ;; esac
        done
        if [[ "$gui" == true ]] && command -v xhost >/dev/null; then
            xhost +si:localuser:root >/dev/null
        fi
        in_container bash scripts/run_robot.sh ros2 launch handy101_mujoco sim.launch.py \
            "headless:=${HANDY101_HEADLESS:-false}" "$@" ;;
    mock) in_container bash scripts/run_robot.sh ros2 launch handy101_bringup control.launch.py backend:=mock "$@" ;;
    world-assets) bash scripts/setup-world-assets.sh ;;
    rosboard) in_container ros2 run rosboard rosboard_node "$@" ;;
    test)
        if [[ "${1:-}" == --sim ]]; then
            in_container python3 checks/check_control_runtime.py --backend mujoco
            in_container python3 checks/check_rosboard_runtime.py
            in_container python3 checks/check_sensors_runtime.py
        else
            in_container bash scripts/check.sh
        fi ;;
    help|-h|--help) usage ;;
    *) usage >&2; exit 2 ;;
esac
