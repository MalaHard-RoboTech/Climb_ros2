#!/usr/bin/env python3
"""
joy_to_three_outputs.py

Subscribes:
  /joy  (sensor_msgs/Joy)

Publishes:
  /alpine/dongle/motorSpeed  (std_msgs/Float32)   # axis[axis_motor] * speed_scale
  /alpine/dongle/servoValve1 (std_msgs/Float32)   # L2 axis (default 2): 1 -> 0 deg, -1 -> 90 deg
  /alpine/dongle/servoValve2 (std_msgs/Float32)   # R2 axis (default 5): 1 -> 0 deg, -1 -> 90 deg

Params:
  axis_motor   (int, default 1)
  axis_l2      (int, default 2)
  axis_r2      (int, default 5)
  speed_scale  (float, default 0.5)
"""

from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy


def clamp(x: float, a: float, b: float) -> float:
    return max(a, min(b, x))


class JoyMapNode(Node):
    def __init__(self) -> None:
        super().__init__('joy_map_node')

        # Parameters
        self.declare_parameter('axis_motor', 1)
        self.declare_parameter('axis_l2', 2)
        self.declare_parameter('axis_r2', 5)
        self.declare_parameter('speed_scale', 0.5)

        self.axis_motor = int(self.get_parameter('axis_motor').value)
        self.axis_l2 = int(self.get_parameter('axis_l2').value)
        self.axis_r2 = int(self.get_parameter('axis_r2').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)

        # Publishers
        self.pub_motor = self.create_publisher(Float32, '/alpine/dongle/motorSpeed', 10)
        self.pub_s1 = self.create_publisher(Float32, '/alpine/dongle/servoValve1', 10)
        self.pub_s2 = self.create_publisher(Float32, '/alpine/dongle/servoValve2', 10)

        # Subscriber
        self.sub_joy = self.create_subscription(Joy, '/joy', self.on_joy, 10)

        self.get_logger().info(
            f'joy_map_node: axis_motor={self.axis_motor} axis_l2={self.axis_l2} axis_r2={self.axis_r2} '
            f'speed_scale={self.speed_scale}'
        )

    def on_joy(self, msg: Joy) -> None:
        axes: List[float] = list(msg.axes) if msg.axes else []

        def ax(i: int) -> float:
            return axes[i] if 0 <= i < len(axes) else 0.0

        # Motor speed: axis * speed_scale, clamp to [-speed_scale, +speed_scale]
        raw_motor = ax(self.axis_motor)
        motor_cmd = clamp(raw_motor * self.speed_scale, -self.speed_scale, self.speed_scale)

        # L2 & R2 remap: 1 -> 0 deg, -1 -> 90 deg
        # deg = ((1 - value) / 2) * 90
        l2_raw = ax(self.axis_l2)
        r2_raw = ax(self.axis_r2)
        s1_deg = ((1.0 - l2_raw) * 0.5) * 90.0
        s2_deg = ((1.0 - r2_raw) * 0.5) * 90.0

        # Publish
        self.pub_motor.publish(Float32(data=float(motor_cmd)))
        self.pub_s1.publish(Float32(data=float(s1_deg)))
        self.pub_s2.publish(Float32(data=float(s2_deg)))


def main() -> None:
    rclpy.init()
    node = JoyMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

