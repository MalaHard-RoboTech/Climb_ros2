import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from std_srvs.srv import Empty

class ManualWheelsTest(Node):
    def __init__(self):
        super().__init__('manual_wheels_test')

        # Declare and get scale factor parameter
        self.declare_parameter('scale_factor', 10.0)
        self.scale_factor = self.get_parameter('scale_factor').value

        # Subscribe to /joy topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Publishers for wheel velocity commands
        self.left_wheel_pub = self.create_publisher(Float32, '/left_wheel/cmd_vel', 10)
        self.right_wheel_pub = self.create_publisher(Float32, '/right_wheel/cmd_vel', 10)

    def joy_callback(self, msg):
        # Get joystick values (axes[1] for left wheel, axes[4] for right wheel)
        left_wheel_speed = msg.axes[1] * self.scale_factor
        right_wheel_speed = msg.axes[1] * self.scale_factor

        # Publish speeds
        self.publish_speed(self.left_wheel_pub, left_wheel_speed)
        self.publish_speed(self.right_wheel_pub, right_wheel_speed)

    def publish_speed(self, publisher, speed):
        msg = Float32()
        msg.data = speed
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ManualWheelsTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
