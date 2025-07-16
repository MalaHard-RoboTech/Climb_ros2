#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float32
from std_srvs.srv import SetBool
from cl_arganello_interface.srv import SetControlMode  # ✅ Use your custom service
import serial
import struct
from dataclasses import dataclass


@dataclass
class MessageIn:
    brake_command: bool
    motor_mode: bool
    control_mode: int  # 0 = torque, 1 = velocity, 2 = position
    target_torque: float
    target_velocity: float
    target_position: float


@dataclass
class MessageOut:
    brake_status: bool
    motor_mode_status: bool
    encoder_count: int
    iq_current: float
    vbus_voltage: float
    motor_temperature: float


class ArganelloNode(Node):

    def __init__(self):
        super().__init__('arganello_node')

        # Parameters
        self.declare_parameter("serial_port", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970047399-if00")
        self.declare_parameter("arganello_id", "sx")
        self.declare_parameter("loop_rate", 200.0)

        self.port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.id = self.get_parameter("arganello_id").get_parameter_value().string_value
        self.rate = self.get_parameter("loop_rate").get_parameter_value().double_value

        # Serial setup
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.05)
            self.get_logger().info(f"✅ Serial opened: {self.port}")
            self.ser.flushInput()
        except Exception as e:
            self.get_logger().error(f"❌ Failed to open serial: {e}")
            raise

        # Struct formats
        self.struct_out_fmt = "<??ifff"   # MessageOut: 1+1+4+4+4+4 = 18 bytes
        self.struct_out_size = struct.calcsize(self.struct_out_fmt)

        self.struct_in_fmt = "<??Bfff"    # MessageIn: 1+1+1+4+4+4 = 15 bytes
        self.struct_in_size = struct.calcsize(self.struct_in_fmt)

        # State to send
        self.cmd = MessageIn(
            brake_command=True,
            motor_mode=False,
            control_mode=0,
            target_torque=0.0,
            target_velocity=0.0,
            target_position=0.0
        )

        ns = f"/arganello/{self.id}"
        self.setup_publishers(ns)
        self.setup_subscribers(ns)
        self.setup_services(ns)

        # Loop timers
        self.create_timer(1.0 / self.rate, self.read_serial)
        self.create_timer(1.0 / self.rate, self.send_command)

    def setup_publishers(self, ns):
        self.pub_brake = self.create_publisher(Bool, f"{ns}/brake_status", 10)
        self.pub_motor_mode = self.create_publisher(Bool, f"{ns}/motor_mode_status", 10)
        self.pub_encoder = self.create_publisher(Int32, f"{ns}/encoder_count", 10)
        self.pub_iq = self.create_publisher(Float32, f"{ns}/iq_current", 10)
        self.pub_voltage = self.create_publisher(Float32, f"{ns}/vbus_voltage", 10)
        self.pub_temp = self.create_publisher(Float32, f"{ns}/motor_temperature", 10)

    def setup_subscribers(self, ns):
        self.create_subscription(Float32, f"{ns}/target_torque", self.torque_cb, 10)
        self.create_subscription(Float32, f"{ns}/target_velocity", self.velocity_cb, 10)
        self.create_subscription(Float32, f"{ns}/target_position", self.position_cb, 10)

    def setup_services(self, ns):
        self.create_service(SetBool, f"{ns}/set_brake", self.set_brake_cb)
        self.create_service(SetBool, f"{ns}/set_motor_mode", self.set_motor_mode_cb)
        self.create_service(SetControlMode, f"{ns}/set_control_mode", self.set_control_mode_cb)

    # ───── READ TELEMETRY ─────────────────────────────
    def read_serial(self):
        if self.ser.in_waiting >= self.struct_out_size:
            data = self.ser.read(self.struct_out_size)
            try:
                brake, motor_mode, enc, iq, vbus, temp = struct.unpack(self.struct_out_fmt, data)

                self.pub_brake.publish(Bool(data=brake))
                self.pub_motor_mode.publish(Bool(data=motor_mode))
                self.pub_encoder.publish(Int32(data=enc))
                self.pub_iq.publish(Float32(data=iq))
                self.pub_voltage.publish(Float32(data=vbus))
                self.pub_temp.publish(Float32(data=temp))

            except struct.error as e:
                self.get_logger().warn(f"❌ Failed to parse data: {e}")

    # ───── SEND COMMAND TO SERIAL ─────────────────────
    def send_command(self):
        try:
            t_torque = self.cmd.target_torque if self.cmd.control_mode == 0 else 0.0
            t_vel = self.cmd.target_velocity if self.cmd.control_mode == 1 else 0.0
            t_pos = self.cmd.target_position if self.cmd.control_mode == 2 else 0.0

            packed = struct.pack(
                self.struct_in_fmt,
                self.cmd.brake_command,
                self.cmd.motor_mode,
                self.cmd.control_mode,
                t_torque,
                t_vel,
                t_pos
            )
            self.ser.write(packed)
        except Exception as e:
            self.get_logger().warn(f"❌ Failed to send command: {e}")

    # ───── SUBSCRIBERS ────────────────────────────────
    def torque_cb(self, msg):
        self.cmd.target_torque = msg.data
        self.send_command()

    def velocity_cb(self, msg):
        self.cmd.target_velocity = msg.data
        self.send_command()

    def position_cb(self, msg):
        self.cmd.target_position = msg.data
        self.send_command()

    # ───── SERVICE CALLBACKS ──────────────────────────
    def set_brake_cb(self, req, res):
        self.cmd.brake_command = req.data
        self.send_command()
        res.success = True
        res.message = f"Brake set to {req.data}"
        return res

    def set_motor_mode_cb(self, req, res):
        self.cmd.motor_mode = req.data
        self.send_command()
        res.success = True
        res.message = f"Motor mode set to {'CLOSED_LOOP' if req.data else 'IDLE'}"
        return res

    def set_control_mode_cb(self, req, res):
        mode = req.data
        if mode in [0, 1, 2]:
            self.cmd.control_mode = mode
            self.send_command()
            res.success = True
            res.message = f"Control mode set to {mode} ({['TORQUE', 'VELOCITY', 'POSITION'][mode]})"
        else:
            res.success = False
            res.message = "❌ Invalid control mode. Use 0 (TORQUE), 1 (VELOCITY), or 2 (POSITION)."
        return res


def main():
    rclpy.init()
    node = ArganelloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down")
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

