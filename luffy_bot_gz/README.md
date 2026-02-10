# luffy_bot_gz

Gazebo Harmonic simulation package for the Luffy robot.

## Contents

- `launch/`: simulation launch files
- `params/`: configuration files (bridge, twist_mux)
- `worlds/`: example and test worlds
- `rviz/`: RViz configuration for simulation

## Launch the simulation

```bash
ros2 launch luffy_bot_gz launch_sim.launch.py
```

## Launch the minimal simulation

```bash
ros2 launch luffy_bot_gz launch_sim_empty.launch.py
```

## Notes

- The simulation is functional, but no controllers are configured yet.
- Navigation and vision packages will be added in upcoming iterations.
