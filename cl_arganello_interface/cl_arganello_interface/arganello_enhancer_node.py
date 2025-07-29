#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cl_arganello_interface.msg import ArganelloRawTelemetry, ArganelloEnhancedTelemetry
from std_srvs.srv import Trigger
import time

# === Constants ===
DRUM_DIAMETER = 0.05  # Diameter of sync roller (5 cm)
DRUM_RADIUS = DRUM_DIAMETER / 2  # 0.025 m
MOTOR_ROLLER_DIAMETER = 0.04  # Diameter of motor pulley (4 cm)
MOTOR_ROLLER_RADIUS = MOTOR_ROLLER_DIAMETER / 2  # 0.02 m
CPR = 2600  # Encoder counts per revolution

# Circumference of the drum
DRUM_CIRCUMFERENCE = 2 * 3.1415926 * DRUM_RADIUS  # ≈ 0.15708 m/rev

# Radians per tick
RAD_PER_TICK = 2 * 3.1415926 / CPR  # ≈ 0.00242 rad/tick

# Arc distance per tick (rope length moved per encoder tick)
ARC_PER_TICK = DRUM_RADIUS * RAD_PER_TICK  # ≈ 0.0000605 m/tick

# Minimum mechanical gear ratio when rope is directly on the shaft (DRUM_RADIUS / MOTOR_ROLLER_RADIUS)
MIN_GEAR_RATIO = DRUM_RADIUS / MOTOR_ROLLER_RADIUS  # 0.025 / 0.02 = 1.25


class ArganelloEnhancerNode(Node):
    def __init__(self):
        super().__init__('arganello_enhancer_node')

        # Parameters
        self.declare_parameter("arganello_id", "dx")
        self.id = self.get_parameter("arganello_id").get_parameter_value().string_value

        # State
        self.initial_sync_raw = 0
        self.last_sync_raw = 0

        # Topics
        ns = f"/arganello/{self.id}"
        self.sub = self.create_subscription(ArganelloRawTelemetry, f"{ns}/telemetry/raw", self.cb, 10)
        self.enhanced_pub = self.create_publisher(ArganelloEnhancedTelemetry, f"{ns}/telemetry/enhanced", 10)
        self.csv_pub = self.create_publisher(String, f"{ns}/telemetry/enhanced/csv", 10)
        self.create_service(Trigger, f"{ns}/set_initial_rope_position", self.set_rope_zero_cb)

    def set_rope_zero_cb(self, request, response):
        self.initial_sync_raw = self.last_sync_raw
        response.success = True
        response.message = f"Initial sync_roller_raw set to {self.initial_sync_raw}"
        self.get_logger().info(response.message)
        return response

    def cb(self, msg: ArganelloRawTelemetry):
        self.last_sync_raw = msg.sync_roller_raw

        # === Derived Calculations ===

        # Distance rope has traveled since zeroing (ticks * arc per tick)
        tick_diff = msg.sync_roller_raw - self.initial_sync_raw
        rope_length = tick_diff * ARC_PER_TICK  # meters

        # Rope velocity = omega * radius
        rope_velocity = msg.sync_roller_vel * DRUM_RADIUS  # m/s

        # Variable gear ratio = motor_vel / sync_roller_vel (min 1.25)
        raw_ratio = msg.motor_vel / (msg.sync_roller_vel + 1e-6)  # prevent div by 0
        variable_gear_ratio = max(raw_ratio, MIN_GEAR_RATIO)

        # Force on the rope = torque * gear ratio
        rope_force = msg.motor_torque * variable_gear_ratio

        # === Enhanced Message ===
        enhanced = ArganelloEnhancedTelemetry()
        enhanced.header = msg.header
        enhanced.brake_status = msg.brake_status
        enhanced.vbus_voltage = msg.vbus_voltage
        enhanced.motor_temperature = msg.motor_temperature
        enhanced.input_id = msg.input_id
        enhanced.input_iq = msg.input_iq
        enhanced.input_position = msg.input_position
        enhanced.input_velocity = msg.input_velocity
        enhanced.input_torque = msg.input_torque
        enhanced.motor_torque = msg.motor_torque
        enhanced.motor_pos = msg.motor_pos
        enhanced.motor_vel = msg.motor_vel
        enhanced.sync_roller_pos = msg.sync_roller_pos
        enhanced.sync_roller_vel = msg.sync_roller_vel
        enhanced.sync_roller_raw = msg.sync_roller_raw

        enhanced.rope_lenght = float(rope_length)
        enhanced.rope_velocity = float(rope_velocity)
        enhanced.rope_force = float(rope_force)
        enhanced.variable_gear_ratio = float(variable_gear_ratio)
        enhanced.tau_friction = 1.0
        enhanced.tau_stiction = 1.0

        self.enhanced_pub.publish(enhanced)

        # CSV publishing
        csv_line = f"{int(time.time())},{int(msg.brake_status)},{msg.vbus_voltage:.3f},{msg.motor_temperature:.3f}," \
                   f"{msg.input_id:.3f},{msg.input_iq:.3f},{msg.input_position:.3f},{msg.input_velocity:.3f}," \
                   f"{msg.input_torque:.3f},{msg.motor_torque:.3f},{msg.motor_pos:.3f},{msg.motor_vel:.3f}," \
                   f"{msg.sync_roller_pos:.3f},{msg.sync_roller_vel:.3f},{msg.sync_roller_raw}," \
                   f"{rope_length:.6f},{rope_velocity:.6f},{rope_force:.6f},{variable_gear_ratio:.6f},1.0,1.0"

        self.csv_pub.publish(String(data=csv_line))


def main(args=None):
    rclpy.init(args=args)
    node = ArganelloEnhancerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
