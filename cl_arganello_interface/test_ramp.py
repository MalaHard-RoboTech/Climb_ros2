#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# ─────────────────────────────────────────────────────────────
# A single step in the velocity sequence: value + duration
# ─────────────────────────────────────────────────────────────
class VelocityStep:
    def __init__(self, velocity: float, duration: float):
        """
        :param velocity: The target velocity to publish
        :param duration: How long to hold this velocity (in seconds)
        """
        self.velocity = velocity
        self.duration = duration

# ─────────────────────────────────────────────────────────────
# Object that runs a sequence of velocity steps
# ─────────────────────────────────────────────────────────────
class VelocityStepSequence:
    def __init__(self, node: Node, topic: str, steps: list[VelocityStep], dt: float = 0.01):
        """
        :param node: The parent ROS2 node
        :param topic: Topic to publish velocities on (e.g., /arganello/sx/target_velocity)
        :param steps: A list of VelocityStep objects
        :param dt: Timer frequency in seconds (default = 10 ms)
        """
        self.node = node
        self.steps = steps
        self.publisher = node.create_publisher(Float32, topic, 10)
        self.dt = dt

        self.current_step_index = 0
        self.step_start_time = node.get_clock().now()

        # Start a timer that checks and publishes at `dt` rate
        self.timer = node.create_timer(dt, self._timer_callback)

    def _timer_callback(self):
        # Time since current step started
        now = self.node.get_clock().now()
        elapsed = (now - self.step_start_time).nanoseconds / 1e9

        # Sequence finished
        if self.current_step_index >= len(self.steps):
            self.publisher.publish(Float32(data=0.0))  # Stop
            self.node.get_logger().info(f"[{self.publisher.topic}] ✅ Sequence completed.")
            self.timer.cancel()
            return

        # Get current step
        current = self.steps[self.current_step_index]

        # Publish current velocity
        self.publisher.publish(Float32(data=current.velocity))
        self.node.get_logger().info(f"[{self.publisher.topic}] Step {self.current_step_index + 1}: velocity = {current.velocity:.2f}")

        # Move to next step if time exceeded
        if elapsed >= current.duration:
            self.current_step_index += 1
            self.step_start_time = now

# ─────────────────────────────────────────────────────────────
# Example ROS2 Node that runs multiple sequences
# ─────────────────────────────────────────────────────────────
class VelocityDemoNode(Node):
    def __init__(self):
        super().__init__('velocity_demo_node')

        # Define a common velocity sequence: 0 → 1 → 2 → 3 → 4, each for 1s
        sequence = [
            VelocityStep(0.0, 1.0),
            VelocityStep(3.0, 1.0),
            VelocityStep(0.0, 1.0),
            VelocityStep(-3.0, 1.0),
            VelocityStep(0.0, 1.0),
            
            VelocityStep(0.0, 1.0),
            VelocityStep(2.0, 1.0),
            VelocityStep(0.0, 1.0),
            VelocityStep(-2.0, 1.0),
            VelocityStep(0.0, 1.0),
            
            VelocityStep(0.0, 1.0),
            VelocityStep(1.0, 1.0),
            VelocityStep(0.0, 1.0),
            VelocityStep(-1.0, 1.0),
            VelocityStep(0.0, 1.0),
        ]

        # Create 5 independent publishers using the same sequence
        VelocityStepSequence(self, '/arganello/sx/target_velocity', sequence)
        VelocityStepSequence(self, '/arganello/dx/target_velocity', sequence)
        VelocityStepSequence(self, '/motor1/target_velocity', sequence)
        VelocityStepSequence(self, '/motor2/target_velocity', sequence)
        VelocityStepSequence(self, '/test_motor/target_velocity', sequence)

# ─────────────────────────────────────────────────────────────
# Standard ROS2 Python main
# ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = VelocityDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
