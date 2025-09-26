#!/usr/bin/env python3
import struct
import time
import serial

from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray, Int32MultiArray

# Protocol constants
HEADER = 0xAA           # Frame start byte
CMD_MOTOR_TICK = 0x01   # Example command id (unused in parsing below)

class SerialManager(Node):
    """
    ROS2 node that manages serial communication with the rover's MCU.
    - Subscribes to /serial_tx (UInt8MultiArray) and writes raw bytes to serial.
    - Publishes encoder delta ticks on /serial_rx (Int32MultiArray).
    - Reads framed data from serial, validates CRC8 and unpacks encoder values.
    """
    def __init__(self):
        super().__init__('serial_manager')

        # Set up parameters, serial port and ROS interfaces
        self._declare_parameters()
        self._load_parameters()
        self._serial_init()
        if self.ser:
            # Clear any stale data from device on start
            self.ser.reset_input_buffer()

        # Subscriber: expects raw bytes wrapped in UInt8MultiArray
        self.vector_subscriber = self.create_subscription(
            UInt8MultiArray,
            '/serial_tx',
            self._wheel_vel_callback,
            10
        )

        # Publisher: sends four int32 delta encoder ticks
        self.publisher = self.create_publisher(
            Int32MultiArray,
            '/serial_rx',
            10
        )

        # Periodic timer to poll serial input
        self._rx_timer = self.create_timer(0.02, self._manage_serial)

        
    def _declare_parameters(self):
        # Default serial device and baud rate. User can override via ROS params.
        #self.declare_parameter('serial_device', '/dev/ttyUSB0')
        self.declare_parameter('serial_device', '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
        self.declare_parameter('baud_rate', 115200)
        
    def _load_parameters(self):
        # Read parameters into instance variables
        self.serial_device = self.get_parameter('serial_device').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Previous encoder counts for computing deltas
        self.prevFL = 0
        self.prevFR = 0
        self.prevRL = 0
        self.prevRR = 0

    def _serial_init(self):
        """Attempt to open the serial port and prepare buffers."""
        try:
            self.ser = serial.Serial(port=self.serial_device,baudrate=self.baud_rate, timeout=0.1, write_timeout=0.1)
            self.rx_buf = bytearray()
            # Give the serial device time to come up (USB-serial adapters / MCUs often reset)
            time.sleep(5)
            self.get_logger().info(f"Connected to {self.serial_device} at {self.baud_rate} baud")
        except serial.SerialException as e:
            # Keep running even if serial is unavailable; node can still exist for debugging
            self.get_logger().error(f"Failed to connect to {self.serial_device}: {e}")
            self.ser = None

    def _wheel_vel_callback(self, msg: UInt8MultiArray):
        """
        Callback for outgoing serial frames.
        Expects msg.data to be a list/array of bytes that can be written directly.
        """
        data_bytes = bytes(msg.data)
        if self.ser:
            self.ser.write(data_bytes)
            self.get_logger().info(f"Sent wheel velocities: {msg.data}")

    def _manage_serial(self):
        """Called periodically by the ROS timer to process incoming serial data."""
        if self.ser:
            self._read_serial()



    def _read_serial(self):
        """
        Read any available bytes from serial, append to rx buffer,
        parse complete frames and publish extracted encoder deltas.
        """
        # The old branch using readline() is disabled; keep code for reference.
        if False:
            if not self.ser:
               return

            try:
                # Keep reading while there's bytes waiting (non-blocking because timeout is set)
                while self.ser.in_waiting:
                    line = self.ser.readline()
                    if not line:
                        break
                    self.get_logger().info(f"Received: {line.strip()}")
            except Exception as e:
                self.get_logger().warning(f"Error reading serial: {e}")

            return
        
        else:
            # Read all available bytes from serial (non-blocking)
            data = self.ser.read(self.ser.in_waiting)
            if not data:
                return
            # Append to rolling receive buffer
            self.rx_buf.extend(data)

            # Try to extract full frames from the buffer
            frames = self.parse_frame_from_buffer(self.rx_buf)

            if not frames:
                return

            for cmd, payload in frames:
                # Expecting at least 16 bytes = 4 x int32 encoder counts
                if len (payload) >= 16:
                    try:
                        # Unpack little-endian signed 32-bit ints
                        encFL_val, encFR_val, encRL_val, encRR_val = struct.unpack('<iiii', payload[:16])
                        # Apply scaling and invert sign if necessary to match wheel convention
                        encFL_val = int(encFL_val/4 *-1)
                        encFR_val = int(encFR_val/4 *-1)
                        encRL_val = int(encRL_val/4 *-1)
                        encRR_val = int(encRR_val/4 *-1)

                        # Compute deltas since last published values
                        dtFL = encFL_val - self.prevFL
                        dtFR = encFR_val - self.prevFR
                        dtRL = encRL_val - self.prevRL
                        dtRR = encRR_val - self.prevRR

                        # Update previous values for next delta computation
                        self.prevFL = encFL_val
                        self.prevFR = encFR_val
                        self.prevRL = encRL_val
                        self.prevRR = encRR_val

                        # Clamp deltas to reasonable range to avoid spikes
                        clamp = lambda v, lo=-30, hi=30: max(lo, min(hi, v))
                        dtFL = clamp(dtFL)
                        dtFR = clamp(dtFR)
                        dtRL = clamp(dtRL)
                        dtRR = clamp(dtRR)

                        # Publish as Int32MultiArray
                        msg = Int32MultiArray()
                        msg.data = [dtFL, dtFR, dtRL, dtRR]
                        self.publisher.publish(msg)

                        # Debug logging (comment out if too verbose)
                        #self.get_logger().info(f"Received encoder ticks: dtFL={dtFL}, dtFR={dtFR}, dtRL={dtRL}, dtRR={dtRR}")

                    except struct.error as e:
                        # If payload length / format doesn't match expected, skip it
                        self.get_logger().warning(f"Skipping malformed payload: {e}")
                        continue
                

    def parse_frame_from_buffer(self,buf):
        """
        Parse framed packets from the provided bytearray buffer.
        Frame format assumed:
         [HEADER (1)][CMD (1)][LENGTH (1)][PAYLOAD (LENGTH)][CRC8 (1)]
        - HEADER: single byte 0xAA
        - CRC8 covers all bytes except the final CRC byte
        This function modifies the buffer in-place, removing consumed bytes.
        Returns a list of tuples: (cmd, payload_bytes)
        """
        frames = []
        header_b = bytes([HEADER])
        while True:
            # Find header byte
            idx = buf.find(header_b)
            if idx == -1:
                # No header found, clear buffer to avoid unbounded growth
                buf.clear()
                break
            if idx>0:
                # Discard leading garbage bytes before header
                del buf[:idx]
            # Need at least header + cmd + length + crc to proceed
            if len(buf) < 4:
                break
            cmd = buf[1]
            length = buf[2]
            total_length = 4 + length  # header+cmd+len + payload + crc
            if len(buf) < total_length:
                # Wait for the rest of the frame to arrive
                break
            # Extract the full frame for CRC check
            frame = bytes(buf[:total_length])

            if self._crc8(frame[:-1]) == frame[-1]:
                # Valid frame: payload is between index 3 and last-1
                payload = frame[3:-1]
                frames.append((cmd, payload))
                # Remove consumed bytes
                del buf[:total_length]
            else:
                # CRC failed: skip this header and try to resync
                del buf[0]
        return frames
    

    def _crc8(self, data):
        """
        Compute CRC-8 (polynomial 0x07) over 'data'.
        This mirrors a common 8-bit CRC implementation used in embedded systems.
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
    

    def _close_serial(self):
        """Safely close the serial port if open."""
        ser = getattr(self, 'ser', None)
        if ser:
            try:
                if getattr(ser, 'is_open', False):
                    try:
                        ser.reset_input_buffer()
                        ser.reset_output_buffer()
                    except Exception:
                        pass
                    ser.close()
                    self.get_logger().info("Serial port closed")
            except Exception as e:
                self.get_logger().warning(f"Error closing serial port: {e}")
            finally:
                self.ser = None


    def destroy_node(self):
        # ensure serial is closed before node is destroyed
        self._close_serial()
        return super().destroy_node()


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = SerialManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()