#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cl_arganello_interface.msg import ArganelloRawTelemetry, ArganelloEnhancedTelemetry
from std_srvs.srv import Trigger
import time
import math


class ArganelloEnhancerNode(Node):
    def __init__(self):
        super().__init__('arganello_enhancer_node')

        # === Parameters ===
        self.declare_parameter("arganello_id", "dx")
        self.declare_parameter("synchronous_roller_radius", 0.025)   # 5 cm
        self.declare_parameter("cable_diameter", 0.004)              # 4 mm
        self.declare_parameter("motor_pignon_radius", 0.02)          # 4 cm
        self.declare_parameter("tau_friction_static", 0.2)           # Nm
        self.declare_parameter("tau_stiction_static", 0.5)           # Nm
        self.declare_parameter("stiction_threshold", 1.0)            # rad/s

        # === Get Parameters ===
        self.id = self.get_parameter("arganello_id").get_parameter_value().string_value
        self.r_sync = self.get_parameter("synchronous_roller_radius").value
        self.r_cable = self.get_parameter("cable_diameter").value
        self.r_motor = self.get_parameter("motor_pignon_radius").value
        self.tau_friction_static = self.get_parameter("tau_friction_static").value
        self.tau_stiction_static = self.get_parameter("tau_stiction_static").value
        self.stiction_threshold = self.get_parameter("stiction_threshold").value

        # === Internal State ===
        self.initial_sync_raw = 0
        self.last_sync_raw = 0

        # === ROS Interfaces ===
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

        # Rope length (in meters)
        rope_length = (msg.sync_roller_pos - self.initial_sync_raw) * self.r_sync

        # Rope velocity
        rope_velocity = msg.sync_roller_vel * (self.r_sync + self.r_cable / 2)

        # Variable gear ratio (prefer velocity ratio if moving)
        if abs(msg.motor_vel) > 1e-3:
            variable_gear_ratio = msg.sync_roller_vel / msg.motor_vel
        else:
            variable_gear_ratio = self.r_sync / self.r_motor

        # Rope force
        rope_force = msg.motor_torque * variable_gear_ratio

        # Friction/Stiction Torque Estimation
        if abs(msg.motor_vel) < self.stiction_threshold:
            tau_stiction = self.tau_stiction_static
            tau_friction = 0.0
        else:
            tau_friction = self.tau_friction_static
            tau_stiction = 0.0

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
        enhanced.tau_friction = float(tau_friction)
        enhanced.tau_stiction = float(tau_stiction)

        self.enhanced_pub.publish(enhanced)

        # === CSV Output ===
        csv_line = f"{int(time.time())},{int(msg.brake_status)},{msg.vbus_voltage:.3f},{msg.motor_temperature:.3f}," \
                   f"{msg.input_id:.3f},{msg.input_iq:.3f},{msg.input_position:.3f},{msg.input_velocity:.3f}," \
                   f"{msg.input_torque:.3f},{msg.motor_torque:.3f},{msg.motor_pos:.3f},{msg.motor_vel:.3f}," \
                   f"{msg.sync_roller_pos:.3f},{msg.sync_roller_vel:.3f},{msg.sync_roller_raw}," \
                   f"{rope_length:.6f},{rope_velocity:.6f},{rope_force:.6f},{variable_gear_ratio:.6f}," \
                   f"{tau_friction:.3f},{tau_stiction:.3f}"

        self.csv_pub.publish(String(data=csv_line))


def main(args=None):
    rclpy.init(args=args)
    node = ArganelloEnhancerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
