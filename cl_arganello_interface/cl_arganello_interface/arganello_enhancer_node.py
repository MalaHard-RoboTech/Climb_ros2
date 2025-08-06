#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from cl_arganello_interface.msg import ArganelloRawTelemetry, ArganelloEnhancedTelemetry
from std_srvs.srv import Trigger
import time
import math
from collections import deque


class ArganelloEnhancerNode(Node):
    def __init__(self):
        super().__init__('arganello_enhancer_node')

        # === Parameters ===
        self.declare_parameter("arganello_id", "dx")
        self.declare_parameter("synchronous_roller_radius", 0.025)   # 2.5 cm
        self.declare_parameter("motor_winch_radius", 0.02)           # 2 cm
        self.declare_parameter("cable_diameter", 0.005)              # 5 mm
        self.declare_parameter("tau_friction_static", 0.2)           # Nm
        self.declare_parameter("tau_stiction_static", 0.5)           # Nm
        self.declare_parameter("stiction_threshold", 1.0)            # rad/s
        self.declare_parameter("reverse_sync_roller_dir", -1)          # +1 (default), or -1 to invert

        # === Get Parameters ===
        self.id = self.get_parameter("arganello_id").get_parameter_value().string_value
        self.sync_roller_r = self.get_parameter("synchronous_roller_radius").value
        self.r_motor_winch = self.get_parameter("motor_winch_radius").value
        self.d_cable = self.get_parameter("cable_diameter").value
        self.tau_friction_static = self.get_parameter("tau_friction_static").value
        self.tau_stiction_static = self.get_parameter("tau_stiction_static").value
        self.stiction_threshold = self.get_parameter("stiction_threshold").value
        self.sync_roller_dir = self.get_parameter("reverse_sync_roller_dir").value

        # === Internal State ===
        self.initial_sync_raw = 0
        self.last_sync_raw = 0
        self.variable_gear_ratio = 1.0
        # the lillte amount to compute in order to have f_rope correct compensating for the caoyic winding of the cable on motor_winch
        self.x = self.d_cable/2
        

        # === ROS Interfaces ===
        ns = f"/arganello/{self.id}"
        self.sub = self.create_subscription(ArganelloRawTelemetry, f"{ns}/telemetry/raw", self.cb, 10)
        self.enhanced_pub = self.create_publisher(ArganelloEnhancedTelemetry, f"{ns}/telemetry/enhanced", 10)
        self.csv_pub = self.create_publisher(String, f"{ns}/telemetry/enhanced/csv", 10)
        self.create_service(Trigger, f"{ns}/set_initial_rope_position", self.set_rope_zero_cb)
        self.target_torque_pub = self.create_publisher(Float32, f"{ns}/target_torque", 10)
        self.create_subscription(Float32, f"{ns}/cmd_rope_force", self.cb_cmd_rope_force, 10)

    def cb_cmd_rope_force(self, msg: Float32):
        requested_rope_force = msg.data

        # Ensure we avoid division by zero
        if abs(self.variable_gear_ratio) < 1e-3:
            self.get_logger().warn("Gear ratio too small to compute torque safely. Skipping command.")
            return

        target_torque = requested_rope_force * self.r_motor_winch / self.variable_gear_ratio

        # Publish the resulting motor torque command
        self.target_torque_pub.publish(Float32(data=target_torque))
        self.get_logger().info(f"[{self.id}] Cmd rope force = {requested_rope_force:.2f} N → target_torque = {target_torque:.3f} Nm")


    def set_rope_zero_cb(self, request, response):
        self.initial_sync_raw = self.last_sync_raw
        response.success = True
        response.message = f"Initial sync_roller_raw set to {self.initial_sync_raw}"
        self.get_logger().info(response.message)
        return response

    def cb(self, msg: ArganelloRawTelemetry):
        # === Convert to rad/s
        motor_velocity = msg.motor_vel * 2 * math.pi
        sync_roller_vel = self.sync_roller_dir * msg.sync_roller_vel
        sync_roller_pos = self.sync_roller_dir * msg.sync_roller_pos
        sync_roller_raw = self.sync_roller_dir * msg.sync_roller_raw
        self.last_sync_raw = sync_roller_raw

        # === velocity_threshold to accurately caclcate mtor gearing (in m/s)
        velocity_threshold = 0.2



        # === Rope length
        rope_length = (sync_roller_pos - self.initial_sync_raw) * self.sync_roller_r

        # === Rope velocity (in m/s)
        rope_velocity = sync_roller_vel * 2 * math.pi * (self.sync_roller_r + self.d_cable / 2)

        # === Update gear ratio only if velocities are meaningful
        if abs(rope_velocity) > velocity_threshold and abs(motor_velocity) > velocity_threshold:
            self.variable_gear_ratio = sync_roller_vel / msg.motor_vel

        # === Compute x and r_effective (chaotic winding correction)
        if abs(motor_velocity) > velocity_threshold and abs(rope_velocity) > velocity_threshold:
            self.x = (rope_velocity * self.r_motor_winch) / (motor_velocity * self.r_motor_winch) - self.r_motor_winch
            self.get_logger().info(f"x = {self.x:.6f} m")

        r_effective = self.r_motor_winch + self.x
        self.get_logger().info(f"r_effective = {r_effective:.6f} m")

        # === Rope force
        rope_force = (msg.motor_torque * self.variable_gear_ratio) / r_effective

        # === Friction/Stiction Torque Estimation ===
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
        enhanced.ibus = msg.ibus
        enhanced.motor_temperature = msg.motor_temperature

        enhanced.input_position = msg.input_position
        enhanced.input_velocity = msg.input_velocity
        enhanced.input_torque = msg.input_torque
        enhanced.motor_torque = msg.motor_torque
        enhanced.motor_pos = msg.motor_pos
        enhanced.motor_vel = msg.motor_vel

        # Publish corrected roller values
        enhanced.sync_roller_pos = sync_roller_pos
        enhanced.sync_roller_vel = sync_roller_vel
        enhanced.sync_roller_raw = sync_roller_raw

        # Enhanced computed fields
        enhanced.rope_lenght = float(rope_length)
        enhanced.rope_velocity = float(rope_velocity)
        enhanced.rope_force = float(rope_force)
        enhanced.variable_gear_ratio = float(self.variable_gear_ratio)
        enhanced.tau_friction = float(tau_friction)
        enhanced.tau_stiction = float(tau_stiction)

        self.enhanced_pub.publish(enhanced)

        # === CSV Output ===
        csv_line = f"{int(time.time())},{int(msg.brake_status)},{msg.vbus_voltage:.3f},{msg.ibus:.3f}," \
                f"{msg.motor_temperature:.3f},{msg.input_position:.3f},{msg.input_velocity:.3f}," \
                f"{msg.input_torque:.3f},{msg.motor_torque:.3f},{msg.motor_pos:.3f},{msg.motor_vel:.3f}," \
                f"{sync_roller_pos:.3f},{sync_roller_vel:.3f},{sync_roller_raw}," \
                f"{rope_length:.6f},{rope_velocity:.6f},{rope_force:.6f},{self.variable_gear_ratio:.6f}," \
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
