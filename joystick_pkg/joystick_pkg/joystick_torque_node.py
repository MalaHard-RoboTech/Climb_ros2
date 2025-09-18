#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Int32

class JoystickTorqueNode(Node):
    def __init__(self):
        super().__init__('joystick_torque_node')

        # Declare parameters
        self.declare_parameter('sx_torque_multiplier', 1.0)
        self.declare_parameter('dx_torque_multiplier', 1.0)

        # Get parameters
        self.sx_multiplier = self.get_parameter('sx_torque_multiplier').get_parameter_value().double_value
        self.dx_multiplier = self.get_parameter('dx_torque_multiplier').get_parameter_value().double_value

        # Publishers
        self.sx_pub = self.create_publisher(Float32, '/arganello/sx/torque', 10)
        self.dx_pub = self.create_publisher(Float32, '/arganello/dx/torque', 10)

        # Subscribers
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_subscription(Int32, '/arganello/sx/encoderCounts', self.sx_encoder_callback, 10)
        self.create_subscription(Int32, '/arganello/dx/encoderCounts', self.dx_encoder_callback, 10)

    def joy_callback(self, msg: Joy):
        try:
            sx_input = msg.axes[1]
            dx_input = msg.axes[3]
        except IndexError:
            self.get_logger().warn("Joystick axes index out of range")
            return

        sx_torque = sx_input * self.sx_multiplier
        dx_torque = dx_input * self.dx_multiplier

        self.sx_pub.publish(Float32(data=sx_torque))
        self.dx_pub.publish(Float32(data=dx_torque))

    def sx_encoder_callback(self, msg: Int32):
        self.get_logger().info(f"🔧 SX Encoder: {msg.data}")

    def dx_encoder_callback(self, msg: Int32):
        self.get_logger().info(f"🔧 DX Encoder: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickTorqueNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
