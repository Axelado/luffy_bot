import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch luffy_bot simulation with Gazebo Harmonic (gz-sim)."""

    package_name = "luffy_bot_gz"
    description_pkg = "luffy_bot_description"

    pkg_path = get_package_share_directory(package_name)

    # Arguments
    gazebo_world_arg = DeclareLaunchArgument(
        "gazebo_world",
        default_value=os.path.join(pkg_path, "worlds", "industrial-warehouse.sdf"),
        description="Full path to the world SDF file to use for simulation",
    )
    gazebo_world = LaunchConfiguration("gazebo_world")

    x_pose_arg = DeclareLaunchArgument(
        "x_pose", default_value="0.0", description="robot x coordinate"
    )
    x_pose = LaunchConfiguration("x_pose")

    y_pose_arg = DeclareLaunchArgument(
        "y_pose", default_value="0.0", description="robot y coordinate"
    )
    y_pose = LaunchConfiguration("y_pose")

    z_pose_arg = DeclareLaunchArgument(
        "z_pose", default_value="0.05", description="robot z coordinate"
    )
    z_pose = LaunchConfiguration("z_pose")

    description_path = get_package_share_directory(description_pkg)

    # Set Gazebo resource paths for mesh resolution
    description_share_parent = os.path.dirname(description_path)
    set_gz_resource_path = AppendEnvironmentVariable(
        "GZ_RESOURCE_PATH", description_share_parent
    )
    set_gz_sim_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", description_share_parent
    )

    # Robot description command
    robot_description_cmd = Command(
        [
            "xacro",
            " ",
            os.path.join(description_path, "urdf", "luffy_bot.urdf.xacro"),
            " ",
            "sim_mode:=true",
        ]
    )

    # Robot state publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    robot_description_cmd, value_type=str
                ),
                "use_sim_time": True,
                "publish_frequency": 50.0,
            }
        ],
        output="screen",
    )

    # Gazebo Harmonic (gz-sim) - using ExecuteProcess
    gz_sim = ExecuteProcess(cmd=["gz", "sim", gazebo_world, "-v", "4"], output="screen")

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "luffy_bot",
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
        ],
        output="screen",
    )

    # Load ROS 2 <-> Gazebo bridge configuration
    bridge_params = os.path.join(
        get_package_share_directory(package_name), "params", "gz_bridge.yaml"
    )

    # Twist mux parameters
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), "params", "twist_mux.yaml"
    )

    # Start ROS 2 <-> Gazebo topic bridge
    gazebo_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    # Bridge camera image topics (if present)
    gazebo_image_bridge_depth_cam_depth_imagee = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge_depth1",
        arguments=["/head_camera/depth_camera/depth_image/depth_image"],
        output="screen",
    )

    gazebo_image_bridge_depth_cam_iamge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge_depth2",
        arguments=["/head_camera/depth_camera/depth_image/image"],
        output="screen",
    )

    gazebo_image_bridge_rgb = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge_rgb",
        arguments=["/head_camera/rgb_camera/image"],
        output="screen",
    )

    gazebo_image_bridge_sem_colored = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge_sem_colored",
        arguments=["/head_camera/rgb_camera/semantic/colored_map"],
        output="screen",
    )

    gazebo_image_bridge_sem_labels = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge_sem_labels",
        arguments=["/head_camera/rgb_camera/semantic/labels_map"],
        output="screen",
    )

    # Start twist_mux to prioritize cmd_vel sources
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[twist_mux_params],
        remappings=[
            ("cmd_vel_out", "cmd_vel_gz"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo_world_arg,
            x_pose_arg,
            y_pose_arg,
            z_pose_arg,
            set_gz_resource_path,
            set_gz_sim_resource_path,
            rsp,
            gz_sim,
            spawn_entity,
            gazebo_ros_bridge,
            gazebo_image_bridge_rgb,
            gazebo_image_bridge_depth_cam_depth_imagee,
            gazebo_image_bridge_depth_cam_iamge,
            gazebo_image_bridge_sem_colored,
            gazebo_image_bridge_sem_labels,
            twist_mux,
        ]
    )
