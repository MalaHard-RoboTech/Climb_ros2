#!/usr/bin/env python3
"""
dongle_node.py — simple ROS 2 node that bridges USB serial to topics.

Params:
  serial_port (string, default /dev/ttyUSB0)
  baud        (int,    default 115200)
  poll_rate   (float,  default 200.0 Hz)

Subs (Float32):
  alpine/dongle/motorSpeed  -> send "m<val>"
  alpine/dongle/servoValve1 -> send "s1 <deg>"
  alpine/dongle/servoValve2 -> send "s2 <deg>"

Pubs:
  alpine/dongle/telemetry/raw   (std_msgs/String): raw CSV lines (complete, de-chunked)
  alpine/dongle/telemetry       (std_msgs/Float32MultiArray): [epoch_ms, imu1[11], imu2[11]]


Usage: 
ros2 topic pub -1 /alpine/dongle/motorSpeed std_msgs/msg/Float32 "{data: 0.3}"
ros2 topic pub -1 /alpine/dongle/servoValve1 std_msgs/msg/Float32 "{data: 45.0}"
ros2 topic pub -1 /alpine/dongle/servoValve2 std_msgs/msg/Float32 "{data: 30.0}"


"""

import re
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String, Float32, Float32MultiArray, MultiArrayLayout, MultiArrayDimension

import serial

_RX_PREFIX = re.compile(r'^\[RX [0-9A-Fa-f:]{17}\]\s*')

def _strip_prefix(line: str) -> str:
    return _RX_PREFIX.sub('', line).strip()

def _split_flex_csv(s: str) -> List[str]:
    s = s.replace('\t', ',')
    s = re.sub(r'[ ,]+', ',', s.strip())
    if s.endswith(','):
        s = s[:-1]
    return [p for p in s.split(',') if p != '']

def _parse_dual_imu(line: str) -> Optional[Tuple[int, List[float], List[float]]]:
    # Expect: epoch_ms,<11 imu1>,<11 imu2>
    core = _strip_prefix(line)
    parts = _split_flex_csv(core)
    if len(parts) < 23:
        return None
    try:
        epoch_ms = int(float(parts[0]))
        vals = [float(x) for x in parts[1:]]
    except Exception:
        return None
    if len(vals) < 22:
        return None
    imu1 = vals[0:11]
    imu2 = vals[11:22]
    return epoch_ms, imu1, imu2

class DongleNode(Node):
    def __init__(self):
        super().__init__('dongle_node')

        # Params
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('poll_rate', 200.0)

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud = int(self.get_parameter('baud').value)
        self.poll_rate = float(self.get_parameter('poll_rate').value)
        self.period = max(0.001, 1.0 / self.poll_rate)

        # Publishers
        qos = QoSProfile(depth=100)
        self.pub_raw = self.create_publisher(String, 'alpine/dongle/telemetry/raw', qos)
        self.pub_parsed = self.create_publisher(Float32MultiArray, 'alpine/dongle/telemetry', qos)

        # Subscribers -> send commands to dongle
        self.create_subscription(Float32, 'alpine/dongle/motorSpeed', self._cb_motor, qos)
        self.create_subscription(Float32, 'alpine/dongle/servoValve1', self._cb_s1, qos)
        self.create_subscription(Float32, 'alpine/dongle/servoValve2', self._cb_s2, qos)

        # Serial (buffered, non-blocking reads)
        self.ser = None
        self._buf = bytearray()
        self._open_serial()

        # Polling timer
        self.timer = self.create_timer(self.period, self._poll_serial)

        self.get_logger().info(
            f"dongle_node: port={self.serial_port} baud={self.baud} poll_rate={self.poll_rate} Hz"
        )

    # --- Serial helpers -----------------------------------------------------
    def _open_serial(self):
        try:
            # small timeout so .read() returns quickly but allows aggregation
            self.ser = serial.Serial(self.serial_port, self.baud, timeout=0.01)
            self.get_logger().info(f"Opened serial: {self.serial_port} @ {self.baud}")
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            self.ser = None

    def _send_line(self, text: str):
        if not text.endswith('\n'):
            text += '\n'
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.write(text.encode('utf-8', errors='ignore'))
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

    # --- Subscribers --------------------------------------------------------
    def _cb_motor(self, msg: Float32):
        self._send_line(f"m{float(msg.data):.6f}")

    def _cb_s1(self, msg: Float32):
        self._send_line(f"s1 {float(msg.data):.3f}")

    def _cb_s2(self, msg: Float32):
        self._send_line(f"s2 {float(msg.data):.3f}")

    # --- Poll serial (buffered) --------------------------------------------
    def _poll_serial(self):
        if self.ser is None or not self.ser.is_open:
            self._open_serial()
            return

        try:
            # read any available bytes (chunked)
            chunk = self.ser.read(1024)
            if chunk:
                self._buf.extend(chunk)

            # extract complete lines ending with '\n'
            while True:
                nl = self._buf.find(b'\n')
                if nl < 0:
                    break
                line_bytes = self._buf[:nl+1]
                del self._buf[:nl+1]
                line = line_bytes.decode(errors='ignore').strip()
                if not line:
                    continue

                # publish raw complete line
                self.pub_raw.publish(String(data=line))

                # parse & publish structured
                parsed = _parse_dual_imu(line)
                if parsed is not None:
                    epoch_ms, imu1, imu2 = parsed
                    data = [float(epoch_ms)] + (imu1 + [0.0]*11)[:11] + (imu2 + [0.0]*11)[:11]
                    layout = MultiArrayLayout(
                        dim=[MultiArrayDimension(label='fields', size=len(data), stride=len(data))],
                        data_offset=0
                    )
                    self.pub_parsed.publish(Float32MultiArray(layout=layout, data=data))

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None  # retry next tick

def main():
    rclpy.init()
    node = DongleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
