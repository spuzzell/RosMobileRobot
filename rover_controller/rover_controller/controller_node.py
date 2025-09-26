#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
import math

class KinematicPUB(Node):
    """ROS2 node that converts Twist (cmd_vel) into motor encoder-rate setpoints.

    The node subscribes to a /diff_cont/cmd_vel_unstamped Twist topic and publishes
    four int16 wheel velocity values (FL, FR, RL, RR) as encoder counts per second
    (or similarly scaled motor-command units).
    """

    def __init__(self):
        super().__init__('rover_controller_node')

        # Parameters with default values (can be overridden via launch/ros2 param)
        self.declare_parameter('wheel_radius', 0.08)   # meters
        self.declare_parameter('wheel_base', 0.39)     # meters (distance between wheels)
        self.declare_parameter('count_rotation', 2096) # encoder counts per wheel revolution

        # Allowed motor velocity (in whatever internal units v_l/v_r yield before scaling)
        self.declare_parameter('min_speed', -8.7)
        self.declare_parameter('max_speed', 8.7)

        # Fetch parameter values into instance variables for fast access
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.count_rotation = self.get_parameter('count_rotation').get_parameter_value().integer_value

         # Subscribe to the (un-stamped) Twist topic that provides linear.x and angular.z
        self.twist_subscriber = self.create_subscription(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            self.cmd_vel_callback,
            10
        )

        # Publisher to the wheel velocity topic expected by the motor controller
        # Format: Int16MultiArray.data = [FL, FR, RL, RR]
        self.publisher = self.create_publisher(
            Int16MultiArray,
            '/wheel_velocities',
            10
        )


    def cmd_vel_callback(self, msg: Twist):
        """Convert incoming Twist into individual wheel motor setpoints and publish.

        Steps:
        - Validate the incoming message to avoid NaN/inf values.
        - Compute left/right wheel angular velocities using differential-drive kinematics.
        - Normalize by wheel radius to get motor angular velocity (rad/s) -> convert to
          encoder counts/sec using counts per revolution.
        - Clamp values to allowed motor range and then to int16 (driver expected format).
        - Publish as Int16MultiArray: [FL, FR, RL, RR].
        """

        try:
            # Validate input
            if not self._is_valid_twist(msg):
                # Use warn (not error) so transient invalid messages don't terminate behavior
                self.get_logger().warn("Invalid twist message received")
                return
                

            v = msg.linear.x     # Forward velocity (m/s)
            omega = msg.angular.z   # Angular velocity (rad/s, yaw)

            # Differential drive kinematics
            L = self.wheel_base    # distance between wheels (m)
            R = self.wheel_radius  # wheel radius (m)

            # Compute wheel angular velocities (rad/s) for left and right wheels:
            # v = R * omega_wheel  -> omega_wheel = v / R
            # For differential drive:
            v_l = (v - (omega * L / 2)) / R
            v_r = (v + (omega * L / 2)) / R

            # Clamp to configured min/max speed to avoid commanding unsafe values
            v_l = max(self.min_speed, min(self.max_speed, v_l))
            v_r = max(self.min_speed, min(self.max_speed, v_r))

            # Convert angular velocity (rad/s) into encoder counts per second:
            # counts_per_sec = (rad_per_sec * counts_per_revolution) / (2*pi)
            L_motor_counts_per_second = (v_l*self.count_rotation) / (2 * math.pi)
            R_motor_counts_per_second = (v_r*self.count_rotation) / (2 * math.pi)

            # Convert to integers (driver expects integer counts/sec)
            L_motor = int(L_motor_counts_per_second)
            R_motor = int(R_motor_counts_per_second)


            # Helper to clamp to int16 range which the motor driver/protocol expects
            def to_int16(v):
                iv = int(round(v))
                if iv < -32768:
                    return -32768
                if iv > 32767:
                    return 32767
                return iv

            L_motor = to_int16(L_motor)
            R_motor = to_int16(R_motor)

            # Duplicate mapping into FL, FR, RL, RR order expected downstream
            wheel_vel_msg = Int16MultiArray()
            wheel_vel_msg.data = [L_motor, R_motor, L_motor, R_motor]

            # Publish the wheel velocity command
            self.publisher.publish(wheel_vel_msg)

            # Debug log for developers: shows remapped wheel values
            # Note: formatting with .3f will cast ints to float for readability            
            self.get_logger().debug(
                f"Published wheel velocities (remapped): FL={L_motor:.3f}, "
                f"FR={R_motor:.3f}, RL={L_motor:.3f}, RR={R_motor:.3f}"
            )
            
        except Exception as e:
             # Catch-all to ensure node keeps running and logs useful info
            self.get_logger().error(f"Error in cmd_vel_callback: {e}")
    

    def _is_valid_twist(self, msg: Twist) -> bool:
        """Validate twist message for NaN or infinite values.

        Returns True when both linear.x and angular.z are finite numbers.
        """
        return (not math.isnan(msg.linear.x) and not math.isinf(msg.linear.x) and
                not math.isnan(msg.angular.z) and not math.isinf(msg.angular.z))   
    


    def _publish_stop_command(self):
        """Publish zero velocities to stop the rover immediately.

        This helper is useful for safety or shutdown procedures where a guaranteed
        stop command must be sent to the motor controller.
        """
        wheel_vel_msg = Int16MultiArray()
        wheel_vel_msg.data = [0, 0, 0, 0]
        self.publisher.publish(wheel_vel_msg)

def main(args=None):
    """Node entrypoint: initialize rclpy, create node, spin, then cleanup."""
    rclpy.init(args=args)
    node = KinematicPUB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()