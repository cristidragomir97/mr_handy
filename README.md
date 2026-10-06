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
- 👀 Watch lidar, IMU and the three camera feeds in rosboard.
- 🧩 Reuse the base101 and mod101 packages, with the lift system and robot assembly here.

Both arms use the mod101 7DOF configuration with 80 mm shoulder and 100 mm elbow
extrusions. The left has the Luxonis camera tool; the right keeps the PGGripper.
The simulation has no GoPro or wrist cameras.

## 🚀 Getting started

You'll need Docker with Compose, [uv](https://docs.astral.sh/uv/getting-started/installation/)
for the first asset download, and the three repositories side by side:

```text
mr_handy/
  base101/
  mod101/
  handy101/
```

Use the companion base101 and mod101 working trees with the Handy101 integration
changes applied. See [setup](docs/setup.md) for the required changes and custom paths.

From `handy101`:

```bash
./scripts/manage.sh start
./scripts/manage.sh sim
```

The first start downloads the home assets and builds the container. Once it opens,
you're in the furnished home with the robot ready to move.

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
