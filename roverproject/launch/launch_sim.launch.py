"""
###########################################################################################################################################################
Launch File: simulation_launch.py

Description:
This ROS 2 launch file sets up a complete simulation environment using Gazebo and ROS-GZ bridge.
It performs the following actions:
- Loads the robot description from a separate launch file (rsp.launch.py).
- Declares and loads a default Gazebo world (with option to override via command line).
- Launches the Gazebo simulator using ros_gz_sim.
- Spawns the robot entity into the simulation using the robot_description topic.
- Bridges topics between Gazebo and ROS using ros_gz_bridge with a configuration file.
- Bridges the robot's camera image topic using ros_gz_image.
###########################################################################################################################################################
"""


import os

# Get the path to a package's share directory
from ament_index_python.packages import get_package_share_directory

# Core launch functionalities
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# For launching ROS 2 nodes
from launch_ros.actions import Node
from launch.actions import LogInfo

banner = LogInfo(msg="""
				 
░▒▓███████▓▒░ ░▒▓██████▓▒░ ░▒▓███████▓▒░      ░▒▓███████▓▒░ ░▒▓██████▓▒░░▒▓█▓▒░░▒▓█▓▒░▒▓████████▓▒░▒▓███████▓▒░  
░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░             ░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░      ░▒▓█▓▒░░▒▓█▓▒░ 
░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░             ░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒▒▓█▓▒░░▒▓█▓▒░      ░▒▓█▓▒░░▒▓█▓▒░ 
░▒▓███████▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓██████▓▒░       ░▒▓███████▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒▒▓█▓▒░░▒▓██████▓▒░ ░▒▓███████▓▒░  
░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░      ░▒▓█▓▒░      ░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░ ░▒▓█▓▓█▓▒░ ░▒▓█▓▒░      ░▒▓█▓▒░░▒▓█▓▒░ 
░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░      ░▒▓█▓▒░      ░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░ ░▒▓█▓▓█▓▒░ ░▒▓█▓▒░      ░▒▓█▓▒░░▒▓█▓▒░ 
░▒▓█▓▒░░▒▓█▓▒░░▒▓██████▓▒░░▒▓███████▓▒░       ░▒▓█▓▒░░▒▓█▓▒░░▒▓██████▓▒░   ░▒▓██▓▒░  ░▒▓████████▓▒░▒▓█▓▒░░▒▓█▓▒░ 
                                                                                                                 
Rover Simulation Launch
(Gazebo + ROS 2 Integration)

""")


def generate_launch_description():

	# Set the package name and mesh path for the rover project
	package_name='roverproject'
	roverproject_mesh_path = os.path.join(get_package_share_directory(package_name), 'meshes')


	# Launch the robot state publisher with simulation time enabled and ros2_control disabled
	rsp = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([os.path.join(
			get_package_share_directory(package_name),'launch','rsp.launch.py'
		)]),
		launch_arguments={
			'use_sim_time': 'true', 
			'use_ros2_control': 'false'
		}.items()
	)

	# Launch joystick control with simulation time enabled
	joystick = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([os.path.join(
			get_package_share_directory(package_name),'launch','joystick.launch.py'
		)]),
		launch_arguments={
			'use_sim_time': 'true'
		}.items()
	)

	# Configure and launch the twist_mux node for velocity command multiplexing
	twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
	twist_mux = Node(
		package="twist_mux",
		executable="twist_mux",
		parameters=[twist_mux_params, {'use_sim_time': True}],
		remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
	)

	# Launch the twist_stamper node to add timestamps to velocity commands
	twist_stamper = Node(
		package='twist_stamper',
		executable='twist_stamper',
		parameters=[{'use_sim_time': True}],
		remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
					('/cmd_vel_out','/diff_cont/cmd_vel')]
	)

	# Set the path to the default Gazebo world file
	default_world = os.path.join(
		get_package_share_directory(package_name),
		'worlds',
		'empty.world'
	)    


	# Allow the world file to be overridden via a launch argument
	world = LaunchConfiguration('world')
	world_arg = DeclareLaunchArgument(
		'world', 
		default_value=default_world, 
		description='World to load'
		)


	# Launch Gazebo (gz_sim) with the selected world file and enable exit-on-shutdown
	gazebo = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([os.path.join(
			get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
		)]),
		launch_arguments={
			'gz_args': ['-r -v1 ', world],  # -r = run immediately, -v4 = verbose level 4
			'on_exit_shutdown': 'true'
		}.items()
	)

	# Spawn the robot into the simulation using the robot_description topic
	spawn_entity = Node(
		package='ros_gz_sim', 
		executable='create',
		arguments=['-topic', 'robot_description',
				   '-name', 'my_bot',
		           '-z', '0.3'     # Spawn 10 cm above the ground to avoid clipping
		],
	    output='screen'
	)    


	# Path to the parameter file for ros_gz_bridge
	bridge_params = os.path.join(
		get_package_share_directory(package_name),
		'config','gz_bridge.yaml'
	)

	# Launch the parameter bridge for simulation-to-ROS topics
	ros_gz_bridge = Node(
		package="ros_gz_bridge", 
		executable="parameter_bridge",
		arguments=['--ros-args','-p',f'config_file:={bridge_params}',]
	)

	# Bridge Gazebo image topic to ROS for the robot's camera
	ros_gz_image_bridge = Node(
		package="ros_gz_image", 
		executable="image_bridge", 
		arguments=["/camera/image_raw"]
	)

	diff_drive_spawner = Node(
		package="controller_manager",
		executable="spawner",
		arguments=["diff_cont"],
	)

	joint_broad_spawner = Node(
		package="controller_manager",
		executable="spawner",
		arguments=["joint_broad"],
	)

	# Combine and return all launch elements
	return LaunchDescription([
		banner,
        rsp,
        joystick,
        twist_mux,
        twist_stamper,
        world_arg,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        ros_gz_image_bridge,
        diff_drive_spawner,
        joint_broad_spawner
    ])
