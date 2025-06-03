import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger  # Import Trigger service type
import odrive
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, CONTROL_MODE_VELOCITY_CONTROL, AXIS_STATE_IDLE
import time

class ODriveVelocityController(Node):
    def __init__(self):
        super().__init__('odrive_velocity_controller')

        self.get_logger().info("Connecting to ODrive...")
        self.odrv = odrive.find_any()

        if self.odrv is None:
            self.get_logger().error("Failed to connect to ODrive.")
            self.odrv.clear_errors()  # Reset all errors
            return
        
        self.get_logger().info("ODrive connected. Initializing motors...")

        # Enable closed-loop control and set to velocity mode
        self.odrv.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        
        # Start at 0 velocity
        self.odrv.axis0.controller.input_vel = 0.0
        self.odrv.axis1.controller.input_vel = 0.0

        self.declare_parameter('left_wheel/encoder/multiplier', 0.77)
        self.declare_parameter('right_wheel/encoder/multiplier', 0.77)

        self.get_logger().info("Motors set to velocity control mode and initialized at 0 velocity.")

        # Create ROS2 Subscribers
        self.create_subscription(Float32, 'left_wheel/cmd_vel', self.left_wheel_callback, 10)
        self.create_subscription(Float32, 'right_wheel/cmd_vel', self.right_wheel_callback, 10)

        # Create ROS2 Publishers
        self.left_encoder_pub = self.create_publisher(Float32, 'left_wheel/encoder', 10)
        self.right_encoder_pub = self.create_publisher(Float32, 'right_wheel/encoder', 10)
        # self.left_temp_pub = self.create_publisher(Float32, 'left_wheel/temperature', 10)
        # self.right_temp_pub = self.create_publisher(Float32, 'right_wheel/temperature', 10)
        self.battery_pub = self.create_publisher(Float32, 'battery/voltage', 10)

        # Create timer to publish sensor data at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

        # Create ROS2 Services
        self.detach_service = self.create_service(Trigger, 'detach_odrive', self.detach_odrive_callback)
        self.reset_encoder_service = self.create_service(Trigger, 'reset_encoder', self.reset_encoder_callback)

    def left_wheel_callback(self, msg):
        if self.odrv is None:
            self.get_logger().warn("ODrive is disconnected. Ignoring velocity command.")
            return
        self.odrv.axis0.controller.input_vel = msg.data

    def right_wheel_callback(self, msg):
        if self.odrv is None:
            self.get_logger().warn("ODrive is disconnected. Ignoring velocity command.")
            return
        self.odrv.axis1.controller.input_vel = msg.data

    def publish_sensor_data(self):
        if self.odrv is None:
            self.get_logger().warn("ODrive is disconnected. Ignoring velocity command.")
            return
        # Get encoder multipliers from parameters
        left_multiplier = self.get_parameter('left_wheel/encoder/multiplier').value
        right_multiplier = self.get_parameter('right_wheel/encoder/multiplier').value
        
        # Publish encoder positions
        left_encoder = Float32()
        left_encoder.data = self.odrv.axis0.encoder.pos_estimate * left_multiplier
        self.left_encoder_pub.publish(left_encoder)
        
        right_encoder = Float32()
        right_encoder.data = self.odrv.axis1.encoder.pos_estimate * right_multiplier
        self.right_encoder_pub.publish(right_encoder)
        
        # # Publish motor temperatures
        # left_temp = Float32()
        # left_temp.data = self.odrv.axis0.fet_thermistor.temperature
        # self.left_temp_pub.publish(left_temp)
        
        # right_temp = Float32()
        # right_temp.data = self.odrv.axis1.fet_thermistor.temperature
        # self.right_temp_pub.publish(right_temp)
        
        # Publish battery voltage
        battery_voltage = Float32()
        battery_voltage.data = self.odrv.vbus_voltage
        self.battery_pub.publish(battery_voltage)

    def detach_odrive_callback(self, request, response):
        """ROS2 Service Callback to Detach ODrive (without rebooting)."""
        self.get_logger().info("Detaching ODrive (disconnecting from USB)...")

        try:
            if self.odrv is not None:
                # Set motors to IDLE before disconnecting
                self.odrv.axis0.requested_state = AXIS_STATE_IDLE
                self.odrv.axis1.requested_state = AXIS_STATE_IDLE
                self.get_logger().info("Motors set to IDLE.")

                # Safely remove the ODrive connection
                self.odrv = None  # Remove reference
                self.get_logger().info("ODrive disconnected successfully.")

            response.success = True
            response.message = "ODrive successfully detached."
        except Exception as e:
            self.get_logger().error(f"Failed to detach ODrive: {e}")
            response.success = False
            response.message = f"Error detaching ODrive: {e}"
        
        return response

    def reset_encoder_callback(self, request, response):
        """ROS2 Service Callback to Reset Encoder Counts."""
        self.get_logger().info("Resetting ODrive encoder counts...")

        try:
            if self.odrv is not None:
                self.odrv.axis0.encoder.set_linear_count(0)
                self.odrv.axis1.encoder.set_linear_count(0)
                self.get_logger().info("Encoder counts reset successfully.")

                response.success = True
                response.message = "Encoders reset to zero."
            else:
                raise Exception("ODrive is disconnected.")

        except Exception as e:
            self.get_logger().error(f"Failed to reset encoders: {e}")
            response.success = False
            response.message = f"Error resetting encoders: {e}"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ODriveVelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
