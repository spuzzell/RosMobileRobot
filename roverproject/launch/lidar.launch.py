import os
from launch import LaunchDescription
from launch_ros.actions import Node

# This launch file starts the RPLIDAR ROS node composition.
# It creates a LaunchDescription containing a single Node that runs
# the rplidar_composition executable from the rplidar_ros package.
# Parameters passed here configure the serial port, TF frame id,
# angle compensation option and the scan mode.

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            # Parameters for the node. These are provided as a list
            # containing a single dictionary of parameter names and values.
            parameters=[{
                # serial_port: device file for the lidar USB/serial adapter.
                # On Linux systems the device path under /dev/serial/by-path
                # is often stable across reboots. Adjust if your device differs.
                'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0',

                # frame_id: name of the TF frame that lidar scans will be published under.
                'frame_id': 'laser_frame',

                # angle_compensate: whether to enable angle compensation for scans.
                # Useful when the lidar driver can fill missing angle buckets.
                'angle_compensate': True,

                # scan_mode: lidar-specific scan mode. 'Standard' is common for RPLIDAR.
                'scan_mode': 'Standard'
            }]
        )
    ])

