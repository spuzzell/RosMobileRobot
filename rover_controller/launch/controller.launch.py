# ROS2 Python launch file for testing nodes on the PC.
# - Start nodes with test parameters and local remappings.
# - Edit packages, executables, and params to match your project.
import os

# Core launch functionalities
from launch import LaunchDescription

# For launching ROS 2 nodes
from launch_ros.actions import Node


def generate_launch_description():

    # Name of the package containing the rover controller nodes
    package_name = 'rover_controller'

    # Rover controller node
    rover_controller_node = Node(
        package=package_name,
        executable='rover_controller_node',
        name='rover_controller_node',
        output='screen'
    )

    # Commander writer node
    commander_writer = Node(
        package=package_name,
        executable='CommanderWriter',
        name='CommanderWriter',
        output='screen'
    )

    # Serial manager node
    serial_manager = Node(
        package=package_name,
        executable='serial_manager_node',
        name='serial_manager',
        output='screen'
    )

    # Commander reader node
    commander_reader = Node(
        package=package_name,
        executable='commander_reader_node',
        name='commander_reader',
        output='screen'
    )

    odom_publisher = Node(
        package=package_name,
        executable='encoder_to_odom_node',
        name='encoder_to_odom_node',
        output='screen'
    )

    bno055_params_path = '/home/rover/robot_ws/src/bno055/bno055/params/bno055_params_i2c.yaml'
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        arguments=['--ros-args', '--params-file', bno055_params_path]
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=['/path/to/ekf.yaml']
    )


    # Combine and return all launch elements
    return LaunchDescription([
        rover_controller_node,
        commander_writer,
        #serial_manager,
        commander_reader,
        odom_publisher,
        #bno055_node,
        #ekf
    ])