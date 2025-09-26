# ...existing code...
"""
Launch File: rsp.launch.py

Description:
This ROS 2 launch file initializes the Robot State Publisher for a robot defined in a XACRO file.
It performs the following actions:
- Declares the `use_sim_time` argument to determine if simulated time should be used.
- Declares the `use_ros2_control` argument to enable generating ros2_control-compatible URDF.
- Loads and processes the robot's URDF from a `.xacro` file using the `xacro` command.
- Launches the `robot_state_publisher` node with the generated URDF and simulation time setting.

This launch file is typically included in larger simulation or control stacks and is responsible
for publishing the `tf` transforms of the robot based on the URDF model.
"""

import os

# For locating shared package resources on the system (required to find files in other packages)
from ament_index_python.packages import get_package_share_directory


# Core launch functionality
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue


# To launch ROS 2 nodes
from launch_ros.actions import Node

# To process XACRO files into URDF (not directly called; we use the xacro command via Command())
import xacro


def generate_launch_description():
    """
    Generate and return the launch description for starting robot_state_publisher.

    The function:
    - Declares launch configurations (use_sim_time, use_ros2_control).
    - Locates the package share directory and the XACRO file.
    - Builds a Command substitution that runs the `xacro` CLI to expand the XACRO into URDF,
      passing in the launch-time arguments (use_ros2_control and sim_mode).
    - Creates a Node for robot_state_publisher with the generated robot_description
      and the use_sim_time parameter.
    """

    # LaunchConfiguration substitution reads the value provided at runtime (or the default)
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Locate the package share directory and the robot xacro file within the package
    pkg_path = os.path.join(get_package_share_directory('roverproject'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Process the xacro file to generate the robot's URDF.
    # Using Command allows the xacro expansion to happen at launch time and accept substitutions.
    # We pass use_ros2_control and sim_mode (mapped to use_sim_time) into the xacro invocation.
    # Note: this invokes the system `xacro` CLI, so xacro must be available in the environment.
    robot_description_config = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=', use_sim_time
    ])

    # Prepare parameters for robot_state_publisher:
    # - robot_description: the expanded URDF (provided as a ParameterValue so the substitution is evaluated)
    # - use_sim_time: whether to use simulated time (useful for Gazebo / playback)
    params = {
        'robot_description': ParameterValue(robot_description_config, value_type=str),
        'use_sim_time': use_sim_time
    }

    # Define the node that will publish the robot state (TFs) from the URDF
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',            # print logs to screen
        parameters=[params]         # provide the parameters dict created above
    )

    # Build and return the full LaunchDescription, including declared arguments and nodes.
    # DeclareLaunchArgument ensures defaults and descriptions are available to users invoking the launch.
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'
        ),

        node_robot_state_publisher
    ])
