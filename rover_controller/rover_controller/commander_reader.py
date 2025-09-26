#!/usr/bin/env python3

"""
commander_reader.py
Node that reads raw encoder counts from /serial_rx, accumulates them,
converts to wheel angles, angular velocities and linear velocities,
and publishes cleaned data for other nodes (/encoder_data, /encoder_data_raw, /joint_states).
"""

import math
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import JointState

class CommanderReader(Node):
    def __init__(self):
        super().__init__('commander_reader')

        # Declare and load parameters used for conversions
        self._declare_parameters()
        self._load_parameters()


        # Subscriber: receives raw encoder delta counts (Int32MultiArray) on /serial_rx
        self.vector_subscriber = self.create_subscription(
            Int32MultiArray,
            '/serial_rx',
            self._encoder_callback,
            10
        )

        # Publisher: filtered/converted encoder data (linear velocities) on /encoder_data
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/encoder_data',
            10
        )

        # Publisher: raw accumulated encoder counts on /encoder_data_raw
        self.raw_publisher = self.create_publisher(
            Int32MultiArray,
            '/encoder_data_raw',
            10
        )

        # Publisher: standard JointState message with positions and velocities
        self.joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

 


    def _declare_parameters(self):
        # Encoder counts per rotation (ticks per revolution)
        self.declare_parameter('count_rotation', 2096)
        # Wheel radius in meters (can be adjusted by parameter)
        self.declare_parameter('wheel_radius', 0.08)
        # Counter modulus (e.g. 16-bit encoder rollover)
        self.declare_parameter('counter_mod', 65536)

        # Accumulated encoder counts for each wheel (initially zero)
        self.encFL_val = 0
        self.encFR_val = 0
        self.encRL_val = 0
        self.encRR_val = 0


    def _load_parameters(self):
        # Load parameters from ROS parameter server and compute derived constants
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        # Apply an empirically determined scale factor (2.5) to wheel radius
        # — keep this if it's intended calibration for your hardware
        self.wheel_radius = self.wheel_radius*2.5
        self.count_rotation = self.get_parameter('count_rotation').get_parameter_value().integer_value
        self.counter_mod = int(self.get_parameter('counter_mod').get_parameter_value().integer_value)
        # Radians per encoder count (one full rotation = 2*pi radians)
        self.rads_per_count = (2 * math.pi) / self.count_rotation


    def _encoder_callback(self, msg: Int32MultiArray):
        """
        Callback for raw encoder delta counts.
        Expects msg.data to contain four integers: dFL, dFR, dRL, dRR
        representing change in encoder counts since last message.
        """
         # Extract delta counts from incoming message
        dFL = int(msg.data[0])
        dFR = int(msg.data[1])
        dRL = int(msg.data[2])
        dRR = int(msg.data[3])

        # Assumed loop/measurement period in seconds (must match sender timing)
        dt_s = 0.01

        # Accumulate the raw counts to keep absolute encoder position
        self.encFL_val += dFL
        self.encFR_val += dFR
        self.encRL_val += dRL
        self.encRR_val += dRR

        # Counts per second (delta counts divided by time)
        cpsFL = dFL / dt_s
        cpsFR = dFR / dt_s
        cpsRL = dRL / dt_s
        cpsRR = dRR / dt_s  

        # Convert accumulated counts to wheel angle in radians
        rads_FL = self.encFL_val * self.rads_per_count
        rads_FR = self.encFR_val * self.rads_per_count
        rads_RL = self.encRL_val * self.rads_per_count
        rads_RR = self.encRR_val * self.rads_per_count

        # Convert counts-per-second to angular velocity (rad/s)
        rads_per_sec_FL = cpsFL * self.rads_per_count
        rads_per_sec_FR = cpsFR * self.rads_per_count
        rads_per_sec_RL = cpsRL * self.rads_per_count
        rads_per_sec_RR = cpsRR * self.rads_per_count   

         # Convert angular velocity to linear wheel velocity (m/s)
        v_FL = rads_per_sec_FL * self.wheel_radius
        v_FR = rads_per_sec_FR * self.wheel_radius
        v_RL = rads_per_sec_RL * self.wheel_radius
        v_RR = rads_per_sec_RR * self.wheel_radius

        # Average left and right side wheel linear velocities
        v_left = (v_FL + v_RL) / 2.0
        v_right = (v_FR + v_RR) / 2.0

        # Simple sanity filter: drop outlier data if velocities implausibly large
        if (abs(v_left) > 5) or (abs(v_right) > 5):
            # Ignore this sample to avoid publishing invalid spikes
            return

        # Publish raw accumulated encoder counts for debugging/ground-truth
        encoder_raw_msg = Int32MultiArray()
        encoder_raw_msg.data = [self.encFL_val, self.encFR_val, self.encRL_val, self.encRR_val]
        self.raw_publisher.publish(encoder_raw_msg)

        # Publish filtered/converted velocities (left, right) as Float32MultiArray
        encoder_msg = Float32MultiArray()
        encoder_msg.data = [
            v_left,      # left wheel linear velocity (m/s)
            v_right      # right wheel linear velocity (m/s)
        ]
        self.publisher.publish(encoder_msg)

         # Publish JointState with wheel angles and angular velocities
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        # Order of joints matches the order used elsewhere in this node
        joint_state.name = ["Fleft_wheel_joint",'Rleft_wheel_joint', 'Rright_wheel_joint', "Fright_wheel_joint"]
        # Positions (radians) - accumulated wheel angles
        joint_state.position = [rads_FL, rads_RL, rads_RR, rads_FR]
        # Velocities (rad/s)
        joint_state.velocity = [rads_per_sec_FL, rads_per_sec_RL, rads_per_sec_RR, rads_per_sec_FR]
        joint_state.effort = []  # No effort/torque data available

        self.joint_state_publisher.publish(joint_state)







def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = CommanderReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()