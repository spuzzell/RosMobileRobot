#!/usr/bin/env python3

import math
from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Imu
import tf2_ros

class EncoderToOdom(Node):
    """
    Node that converts wheel encoder increments (from /serial_rx) into
    a nav_msgs/Odometry message and broadcasts a TF from odom->base_link.
    It optionally fuses yaw with the latest IMU orientation to reduce drift.
    """
    def __init__(self):

        super().__init__('encoder_to_odom')

        # parameters (with defaults)
        self.declare_parameter('wheel_radius', 0.08)     # meters
        self.declare_parameter('track_width', 0.39)     # distance between left and right wheels (meters)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_fusion_gain', 0) # fraction of IMU correction to apply

        # read parameters into attributes
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_radius = self.wheel_radius 

        self.imu_fusion_gain = self.get_parameter('imu_fusion_gain').get_parameter_value().double_value

        self.track_width = self.get_parameter('track_width').get_parameter_value().double_value
        # scale track_width by a calibration factor (1.4) — preserves previous behaviour
        self.track_width = self.track_width*1.4
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
        # encoder counts to radians conversion (counts per revolution = 2096)
        self.rads_per_count = 2 * math.pi / 2096

        # time of last encoder update (rclpy Time object)
        self.last_update_time = None


        # pose state maintained by dead-reckoning (x, y, yaw)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # last raw encoder values (not currently used individually, kept for future)
        self.encFL_val = 0
        self.encFR_val = 0
        self.encRL_val = 0
        self.encRR_val = 0

        # last IMU values for optional fusion
        self.imu_timestamp = None
        self.imu_yaw = None
        self.imu_ang_vel_z = None

        # ROS publishers and TF broadcaster
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # subscribe to encoder increments published as Int32MultiArray on /serial_rx
        self.vector_subscriber = self.create_subscription(
            Int32MultiArray,
            '/serial_rx',
            self._encoder_callback,
            10
        )

        # subscribe to IMU to obtain yaw and angular velocity for fusion
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu',
            self._imu_callback,
            10
        )

         # initialize odom publisher with a starting pose/transform
        self._init_odom_set()

        self.get_logger().info('encoder_to_odom node started')


    def _imu_callback(self, msg: Imu):
        """
        Store the latest IMU yaw and angular velocity z for use in sensor fusion.
        If quaternion->euler conversion fails, imu_yaw is set to None.
        """
        # extract yaw from quaternion and store angular velocity z
        q = msg.orientation
        q_list = [q.x, q.y, q.z, q.w]
        try:
            # extract yaw (z rotation) from IMU quaternion
            _, _, yaw = euler_from_quaternion(q_list)
            self.imu_yaw = yaw
        except Exception:
            # if conversion fails, disable IMU fusion for this sample
            self.imu_yaw = None

        try:
            # store angular velocity around z (useful as alternative to encoder-based omega)
            self.imu_ang_vel_z = msg.angular_velocity.z
        except Exception:
            self.imu_ang_vel_z = None


    def _encoder_callback(self, msg: Int32MultiArray):
        """
        Called when incremental encoder counts arrive.
        Expects msg.data = [dFL, dFR, dRL, dRR] as incremental counts since last message.
        Performs differential-drive kinematics to update pose and publishes Odometry.
        """
        # Expecting [dFL, dFR, dRL, dRR] as incremental counts
        dFL, dFR, dRL, dRR = [float(v) for v in msg.data]

        # Average left and right wheels
        delta_left = (dFL + dRL) / 2.0
        delta_right = (dFR + dRR) / 2.0

        # Convert to radians
        delta_left_rad = delta_left * self.rads_per_count
        delta_right_rad = delta_right * self.rads_per_count

        # Compute wheel displacements in meters
        dL = delta_left_rad * self.wheel_radius
        dR = delta_right_rad * self.wheel_radius

        # Linear and angular displacement
        d_center = (dL + dR) / 2.0
        d_theta = (dR - dL) / self.track_width

        # Compute dt dynamically
        now = self.get_clock().now()
        if self.last_update_time is None:
            dt_s = 0.01  # fallback for first update
        else:
            dt_s = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        # Midpoint integration
        dx = d_center * math.cos(self.yaw + d_theta / 2.0)
        dy = d_center * math.sin(self.yaw + d_theta / 2.0)
        self.x += dx
        self.y += dy

        # predicted yaw from wheel integration
        yaw_pred = self.yaw + d_theta
        # fuse with IMU yaw (if available) using small-angle correction
        if self.imu_yaw is not None:
            # angle error imu - predicted (wrapped to [-pi,pi])
            err = math.atan2(math.sin(self.imu_yaw - yaw_pred), math.cos(self.imu_yaw - yaw_pred))
            # correct predicted yaw by a fraction of the error
            self.yaw = (yaw_pred + self.imu_fusion_gain * err)
        else:
            self.yaw = yaw_pred

        # normalize
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        # Velocities
        v = d_center / dt_s

        if self.imu_ang_vel_z is not None:
            omega = self.imu_ang_vel_z
        else:
            omega = d_theta / dt_s if dt_s > 0.0 else 0.0

        # Prepare odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega

        # Publish odometry
        self.odom_pub.publish(odom)

        # transform publishing is currently commented out; keep code available if TF needed
        # t = TransformStamped()
        # t.header.stamp = odom.header.stamp
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'base_link'
        # t.transform.translation.x = self.x
        # t.transform.translation.y = self.y
        # t.transform.translation.z = 0.0
        # t.transform.rotation = odom.pose.pose.orientation
        # self.tf_broadcaster.sendTransform(t)

    
    def _init_odom_set(self):
        """
        Initialize/reset the odom pose and publish an initial odometry + TF.
        Currently this sets x,y,yaw to zero and publishes that initial frame.
        """
        # reset pose state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.get_logger().info(f"Odom coordinates set to: x={self.x}, y={self.y}, yaw={self.yaw}")


        # prepare odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        # publish odom
        self.odom_pub.publish(odom)

        # publish transform
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
 

def main(args=None):
    rclpy.init(args=args)
    node = EncoderToOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

