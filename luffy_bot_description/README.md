# luffy_bot_description

Description package for the Luffy robot (URDF/Xacro). This package originates from
an older Fetch model and has been updated to run on ROS 2 Jazzy and Gazebo Harmonic.

## Contents

- `urdf/`: Xacro and URDF files
- `meshes/`: robot meshes
- `rviz/`: RViz configurations
- `launch/`: launch files (e.g. robot_state_publisher)

## Launching the robot description

```bash
ros2 launch luffy_bot_description rsp.launch.py
```

## Notes

- This package does not configure controllers.
- Sensors are described in the Xacro files and will be extended as required.
