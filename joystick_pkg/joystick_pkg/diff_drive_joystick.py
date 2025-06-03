import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class DiffDriveJoystick(Node):
    def __init__(self):
        super().__init__('diff_drive_joystick')

        # Declare and get scale factor parameter
        self.declare_parameter('scale_factor', 2.0)
        self.scale_factor = self.get_parameter('scale_factor').value

        # Subscribe to /joy topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Publisher for differential drive velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel', 10)

        # State tracking
        self.motor_running = False  # Track whether motor is running
        self.prev_button_state = 0  # Used to detect button press events

    def joy_callback(self, msg):
        # Toggle motor state when button 1 is pressed
        if msg.buttons[1] == 1 and self.prev_button_state == 0:  # Button press event
            self.toggle_motor()
        
        self.prev_button_state = msg.buttons[1]  # Store button state for next callback

        # Get joystick values (axes[1] for linear.x, axes[0] for angular.z)
        twist_msg = Twist()
        twist_msg.linear.x = msg.axes[1] * self.scale_factor
        twist_msg.angular.z = msg.axes[0] * self.scale_factor

        # Publish velocity command
        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveJoystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
