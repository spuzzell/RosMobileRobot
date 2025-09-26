#!/usr/bin/env python3

import struct
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, UInt8MultiArray

# Protocol constants
HEADER = 0xAA          # Start byte for frames
CMD_MOTOR_TICK = 0x01  # Command ID for motor tick / wheel velocities

class CommanderWriter(Node):
    """
    ROS2 node that listens for wheel velocity vectors and publishes
    framed serial packets (UInt8MultiArray) to a /serial_tx topic.
    """
    def __init__(self):
        super().__init__('commander_writer')

         # Latest wheel velocities in order [FL, FR, RL, RR] as int16
        self.latest_wheel_vel = [0, 0, 0, 0]
        # Timestamp of the most recent velocity message
        self.last_vel_time = self.get_clock().now()
    
         # Subscriber: receives Int16MultiArray messages with wheel velocities
        self.vector_subscriber = self.create_subscription(
            Int16MultiArray,
            '/wheel_velocities',
            self._vel_writer_callback,
            10
        )

        # Publisher: sends framed bytes as UInt8MultiArray to be transmitted over serial
        self.publisher = self.create_publisher(
            UInt8MultiArray,
            '/serial_tx',
            10
        )

        # Periodic timer to build and send frames at 5 Hz
        self.serial_timer = self.create_timer(0.2, self._command_writer_callback)


    def _vel_writer_callback(self, msg: Int16MultiArray):
        # Expect at least 4 elements: FL, FR, RL, RR
        if not msg.data or len(msg.data) < 4:
            # Ignore malformed messages
            return
        # store most recent four wheel velocities (int16)
        self.latest_wheel_vel = [int(msg.data[0]), int(msg.data[1]), int(msg.data[2]), int(msg.data[3])]
        self.last_vel_time = self.get_clock().now()
    
    def _command_writer_callback(self):
        """
        Called by timer. Builds a frame from the latest wheel velocities and
        publishes it to /serial_tx. If velocities are stale, the function returns.
        """
        # If no recent command (older than 2 seconds), do nothing
        if not hasattr(self, 'latest_wheel_vel') or (self.get_clock().now() - self.last_vel_time).nanoseconds > 2_000_000_000:
            return
        else:
            FL, FR, RL, RR = self.latest_wheel_vel

        # Build payload (binary packed int16 values)
        payload = self._make_payload(FL, FR, RL, RR)
        if payload is None:
            # payload creation failed (out of range); do not publish invalid frame
            self.get_logger().error("Payload creation failed; skipping publish")
            return
        
        # Construct the complete frame (header + cmd + len + payload + crc)
        frame = self._frame_builder(CMD_MOTOR_TICK, payload)

        # Publish as UInt8MultiArray (list of bytes -> list of ints 0..255)
        msg = UInt8MultiArray()
        msg.data = list(frame)  # converts bytes -> list of uint8 (0..255)
        self.publisher.publish(msg)

    def _make_payload(self, FL, FR, RL, RR):
        """
        Pack four int16 wheel velocities into little-endian binary payload.
        Validate range for int16 before packing.
        Returns bytes payload or None on error.
        """
        vals = [int(FL), int(FR), int(RL), int(RR)]
        if any(v < -32768 or v > 32767 for v in vals):
            self.get_logger().error(f"Wheel speed out of int16 value range: {vals}")
            return None
        # '<hhhh' => little-endian, four signed short (int16)
        payload = struct.pack('<hhhh', *vals)
        # payload length validation (protocol uses a single length byte)
        if len(payload) > 255:
            self.get_logger().error("Payload too large")
            return None
        return payload
        

    def _frame_builder(self, command, payload):
        """
        Build the complete frame:
        [HEADER][CMD][LEN][PAYLOAD...][CRC]
        CRC is calculated over everything except the CRC byte itself.
        """
        body = bytes([HEADER, command & 0xFF, len(payload) & 0xFF]) + payload
        return body + bytes([self._crc8(body)])


    def _crc8(self, data):
        """
        Compute CRC-8 (polynomial 0x07) over input data.
        Polynomial implementation follows standard bitwise algorithm.
        """
        crc = 0x00
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc
    


def main(args=None):
    """
    Standard ROS2 Python node entrypoint. Initializes rclpy, spins the node,
    and performs clean shutdown.
    """
    import rclpy
    rclpy.init(args=args)
    node = CommanderWriter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()