#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
import math

STEP_TO_METERS = (-2 * math.pi * 0.025) / 400  # meters per encoder step
PUBLISH_HZ = 200.0


class EncoderVelocityNode(Node):
    def __init__(self):
        super().__init__('encoder_velocity_node')

        # Internal state
        self.current_count = None
        self.last_count = None
        self.last_time = self.get_clock().now()

        # ROS interfaces
        self.subscription = self.create_subscription(
            Int32,
            '/arganello/sx/encoder_count',
            self.encoder_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32,
            '/arganello/sx/encoder_vel',
            10
        )

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.timer_callback)

        self.get_logger().info("✅ EncoderVelocityNode running at 200 Hz")

    def encoder_callback(self, msg):
        self.current_count = msg.data

    def timer_callback(self):
        now = self.get_clock().now()

        if self.current_count is not None and self.last_count is not None:
            dt = (now - self.last_time).nanoseconds / 1e9  # seconds
            if dt > 0:
                delta_steps = self.current_count - self.last_count
                velocity = (delta_steps * STEP_TO_METERS) / dt
                msg = Float32()
                msg.data = velocity
                self.publisher.publish(msg)

        self.last_time = now
        self.last_count = self.current_count


def main(args=None):
    rclpy.init(args=args)
    node = EncoderVelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
