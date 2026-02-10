# luffy_bot

Robotics project using ROS 2 Jazzy and Gazebo Harmonic. The robot model is derived
from an older URDF of the Fetch robot (Fetch Robotics) and has been updated to run
with ROS 2 and Gazebo Harmonic.

## Current status

- ✅ Simulation is working
- ❌ No controllers configured yet
- 🛠️ Navigation and vision packages will be added soon

## Packages

- `luffy_bot_description`: robot description (URDF/Xacro, meshes, RViz)
- `luffy_bot_gz`: Gazebo Harmonic simulation, bridge and configuration files

## Prerequisites

- ROS 2 Jazzy
- Gazebo Harmonic
- `colcon`

## Build

```bash
cd ~/ROS/pizi_ws
colcon build --symlink-install
source install/setup.bash
```

## Launching the robot description (RViz)

```bash
ros2 launch luffy_bot_description rsp.launch.py
```

## Launching the Gazebo simulation

```bash
ros2 launch luffy_bot_gz launch_sim.launch.py
```

## Pre-commit setup (after cloning)

To enable automatic code formatting and linting on commit, set up pre-commit hooks:

```bash
python3 -m pip install --upgrade pip
pip install pre-commit black isort flake8 yamllint
pre-commit install
pre-commit run --all-files
```

This ensures all contributors use the same code style and catch errors before pushing.

## Notes

- The robot is based on the Fetch URDF and has been adapted and modernized.
- Controllers (ros2_control / controller manager) are not configured yet and
	will be added in a future update.
