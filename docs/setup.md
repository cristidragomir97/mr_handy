# Setup and development

[← README](../README.md)

## Requirements

Use Docker with Compose v2 and `uv` on the host. A desktop display is needed for
MuJoCo's GUI; Linux/X11 and WSLg have Compose overrides selected automatically by
`manage.sh`. Headless mode needs no display.

The first `start` runs the asset preparation helper through `uv`, then builds the
ROS 2 Jazzy image. You do not need ROS installed on the host for the container workflow.

## Companion repositories

Docker uses GitHub build contexts for
[base101](https://github.com/robocore-labs/base101) and
[mod101](https://github.com/robocore-labs/mod101). BuildKit downloads and caches its
own copies; no host clones or repository path settings are needed.

Each `start` or `recreate` resolves the latest commit on each repository's default
branch and rebuilds affected image layers. Network access to GitHub is required.
The image keeps the dependency package sources in `/opt/handy101_dependencies/src`
so `manage.sh build` can rebuild them alongside this project's mounted sources.
`build` and `restart` use the existing image sources; use `recreate` to update them.
Docker applies the compatibility adapter in `docker/prepare_dependencies.py` to
its downloaded copies: optional raised-deck selection, browser-safe rosboard
JSON, and an assembly-specific EKF configuration override for base101 SLAM. Unexpected upstream layouts fail the build for review. Host clones are
not modified. The STEP file is only needed when re-exporting CAD.

## Container commands

Run these from the Handy101 repository:

```bash
./scripts/manage.sh start             # build the image and start containers
./scripts/manage.sh sim               # launch the robot and rosboard
./scripts/manage.sh mock              # check controller routing without physics
./scripts/manage.sh build             # rebuild ROS packages after source changes
./scripts/manage.sh build rosboard    # rebuild a specific package
./scripts/manage.sh test --nav         # verify live mapping and navigation
./scripts/manage.sh exec              # open a sourced container shell
./scripts/manage.sh logs              # follow container logs
./scripts/manage.sh stop              # stop this project's containers
./scripts/manage.sh help              # list all commands
```

Run one robot stack at a time. Stop the foreground launch with Ctrl+C before
starting another; the launch script rejects a second `sim` or `mock` instance.

`start` builds a complete image. `recreate` rebuilds it and replaces the containers,
while preserving source and named build volumes. `restart` restarts the existing
containers. Build, install and log directories use Docker volumes, separate from
native builds. The image includes a built assembly, and the containers mount this
repository for development. Dependency sources come from the image.

## Configuration and networking

Copy `.env.example` to `.env` to change the ROS domain,
GUI mode or the GPU adapter. The defaults are:

| Setting | Default |
|---|---|
| `HANDY101_ROS_DOMAIN_ID` | `184` |
| `HANDY101_HEADLESS` | `false` |
| `HANDY101_START_ROUTER` | `true` |

Rosboard is exposed on localhost port **8888**, and the bundled Zenoh router on
port **7447**, including with Docker Desktop. Robot processes and container shells
use `rmw_zenoh_cpp` and the same ROS domain.

If a Zenoh router already runs locally, set `HANDY101_START_ROUTER=false` in `.env`.
Changing `rosboard_port` at launch also requires updating the Compose port mapping
if you want the new port accessible from the host.

## Graphics

MuJoCo opens its desktop window by default. Press **Tab** for the left panel and
**Shift+Tab** for the right panel. Use `headless:=true` to run without a window.

On Linux, the script passes through `/dev/dri`. On WSLg, it uses `/dev/dxg` and
Mesa's D3D12 backend. `HANDY101_GPU_ADAPTER` selects an adapter by name; the current
WSL default is `Intel`. Adapter availability depends on the host's WSL graphics
support. Recreate the container after changing this setting.

A GUI shutdown with camera rendering active has crashed in Mesa/libgallium
cleanup and required ROS launch escalation. This remains a known graphics
cleanup issue. If shutdown hangs, stop the project's containers before relaunching.

## Native ROS build

Install ROS 2 Jazzy and the simulation dependencies from `docker/Dockerfile`.
Native builds still require separate base101 and mod101 workspaces. Build the
mod101 tools first, then run these commands from Handy101:

```bash
./scripts/manage.sh world-assets
source /opt/ros/jazzy/setup.bash
# Build updated mod101 descriptions/tools in their workspace first.
source ../mod101/install/setup.bash
colcon build --symlink-install --base-paths src ../base101/src \
  --packages-select base101_description base101_control rosboard \
  handy101_description handy101_control handy101_bringup handy101_mujoco
source install/setup.bash
export ROS_DOMAIN_ID=184 RMW_IMPLEMENTATION=rmw_zenoh_cpp
# Start ros2 run rmw_zenoh_cpp rmw_zenohd in another terminal if no router runs.
ros2 launch handy101_mujoco sim.launch.py headless:=true
```

For a controller routing dry run, use
`ros2 launch handy101_bringup control.launch.py backend:=mock`.
Both launches start rosboard; `rosboard:=false` disables it and
`rosboard_port:=8889` changes its port. `control_mode:=position` activates direct
position controllers instead of their trajectory counterparts. `arms:=false`
provides a base/lifts/head variant. `floating_base:=false` on the MuJoCo launch
anchors the chassis for bench testing. `scene_dir:=/path` retains generated
`scene.xml` and `robot.urdf` for inspection; otherwise a temporary directory is used.

## Checks

See [simulation checks](simulation.md#checks) for offline and live verification.
