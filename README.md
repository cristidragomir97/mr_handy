# handy101 🤖

Mr Handy brings together a base101 wheeled chassis and two mod101 arms, each on
its own 450 mm lift. One hand carries a Luxonis camera, the other a PGGripper,
and a tilting RealSense camera sits up top.

The parts stay in their original projects. Handy101 mounts them together and
runs the whole robot in ROS 2 and MuJoCo, so changes to the base or arms can carry
through to the assembly.

## What you can do

- 🏠 Drive around a furnished kitchen, dining room and living room, with loose objects to pick up.
- 🦾 Move the arms, lifts, gripper and head from the browser's joint sliders.
- 🗺️ Build a map with base101 SLAM and navigate with its Nav2 stack.
- 👀 Watch lidar, IMU and the three camera feeds in rosboard.
- 🧩 Reuse the base101 and mod101 packages, with the lift system and robot assembly here.

Both arms use the mod101 7DOF configuration with 80 mm shoulder and 100 mm elbow
extrusions. The left has the Luxonis camera tool; the right keeps the PGGripper.
The simulation has no GoPro or wrist cameras.

## 🚀 Getting started

You'll need Docker with Compose and [uv](https://docs.astral.sh/uv/getting-started/installation/)
for the first asset download. Docker fetches and manages its own copies of
[base101](https://github.com/robocore-labs/base101) and
[mod101](https://github.com/robocore-labs/mod101); no sibling clones are needed.

From this repository:

```bash
./scripts/manage.sh start
./scripts/manage.sh sim
```

The first start downloads the home assets and builds the container. `sim` opens
MuJoCo in the furnished home with the robot ready to move.

Each `start` or `recreate` fetches the latest default-branch commits from GitHub.
Docker caches the downloads and keeps dependency sources inside the image;
`build` rebuilds ROS packages using those sources, while `restart` simply restarts
the existing containers. To update an existing setup:

```bash
./scripts/manage.sh recreate
./scripts/manage.sh test
./scripts/manage.sh sim
```

The image build applies the compatibility fixes needed by this assembly to its
own downloaded copies: raised-deck selection for the lift assembly and rosboard
serialization of missing/non-finite feedback as JSON `null`. Existing local
`base101` and `mod101` clones are unused. See [setup](docs/setup.md) for details.

`sim` also starts base101's SLAM, EKF and Nav2 stacks. Mapping uses wheel speed
and IMU yaw rate for odometry, with calibrated skid-steer control for turns. Open `/map` in rosboard
to watch mapping, or add `rviz:=true` for the navigation display and goal tool.
Use `mapping:=false navigation:=false` to run only the robot simulation.

Open **[rosboard](http://localhost:8888)** to watch topics, use the joint sliders
or drive with the teleop card. In the MuJoCo window, press **Tab** and
**Shift+Tab** to reveal the side panels.

Stop the simulation with **Ctrl+C**. To stop the containers too:

```bash
./scripts/manage.sh stop
```

For a simpler scene or a run without a window:

```bash
./scripts/manage.sh sim world:=empty
./scripts/manage.sh sim headless:=true
```

## 📚 Go further

| Guide | What's inside |
|---|---|
| [Setup and development](docs/setup.md) | Container commands, native builds, networking and graphics |
| [Simulation](docs/simulation.md) | The home, sensors, teleoperation and checks |
| [Robot description and CAD](docs/description.md) | Lift geometry, arm mounting, camera tilt and mesh exports |
| [ROS control](docs/control.md) | Controllers, trajectory commands and joint sliders |
| [Project boundaries](docs/ownership.md) | What belongs to Handy101, base101 and mod101 |
