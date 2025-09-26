import os

# Core launch functionalities
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# For launching ROS 2 nodes
from launch_ros.actions import Node

# to locate package share directories
from ament_index_python.packages import get_package_share_directory


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

    # bno055 node (runs equivalent to:
    # ros2 run bno055 bno055 --ros-args --params-file /home/rover/robot_ws/src/bno055/bno055/params/bno055_params_i2c.yaml
    # Uses the absolute path specified by the user
    bno055_params_path = '/home/rover/robot_ws/src/bno055/bno055/params/bno055_params_i2c.yaml'
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        arguments=['--ros-args', '--params-file', bno055_params_path]
    )
    
    lidar = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('roverproject'),'launch','lidar.launch.py')]))


    # Combine and return all launch elements
    return LaunchDescription([
        #rover_controller_node,
        #commander_writer,
        serial_manager,
        bno055_node,
        lidar,
        #commander_reader,
        #odom_publisher
    ])