#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float32
from std_srvs.srv import SetBool  # ✅ Use this for both services
import serial
import struct


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

        # Serial
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.05)
            self.get_logger().info(f"✅ Serial opened: {self.port}")
            self.ser.flushInput()
        except Exception as e:
            self.get_logger().error(f"❌ Failed to open serial: {e}")
            raise

        # Structs
        self.struct_out_fmt = "<??ifff"  # MessageOut: 1+1+4+4+4+4 = 18 bytes
        self.struct_out_size = struct.calcsize(self.struct_out_fmt)

        self.struct_in_fmt = "<??fff"    # MessageIn: 1+1+4+4+4 = 14 bytes
        self.struct_in_size = struct.calcsize(self.struct_in_fmt)

        # State to be sent
        self.cmd = {
            'brake_command': True,
            'motor_mode': False,
            'target_torque': 0.0,
            'target_velocity': 0.0,
            'target_position': 0.0
        }

        ns = f"/arganello/{self.id}"
        self.setup_publishers(ns)
        self.setup_subscribers(ns)
        self.setup_services(ns)

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

    # ───── TELEMETRY IN ─────────────────────────────
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

    # ───── COMMAND OUT ──────────────────────────────
    def send_command(self):
        try:
            packed = struct.pack(
                self.struct_in_fmt,
                self.cmd['brake_command'],
                self.cmd['motor_mode'],
                self.cmd['target_torque'],
                self.cmd['target_velocity'],
                self.cmd['target_position']
            )
            self.ser.write(packed)
        except Exception as e:
            self.get_logger().warn(f"❌ Failed to send command: {e}")

    # ───── TOPIC CALLBACKS ──────────────────────────
    def torque_cb(self, msg):
        self.cmd['target_torque'] = msg.data
        self.cmd['target_velocity'] = 0.0
        self.cmd['target_position'] = 0.0
        # preserve brake_command and motor_mode
        self.send_command()

    def velocity_cb(self, msg):
        self.cmd['target_torque'] = 0.0
        self.cmd['target_velocity'] = msg.data
        self.cmd['target_position'] = 0.0
        # preserve brake_command and motor_mode
        self.send_command()

    def position_cb(self, msg):
        self.cmd['target_torque'] = 0.0
        self.cmd['target_velocity'] = 0.0
        self.cmd['target_position'] = msg.data
        # preserve brake_command and motor_mode
        self.send_command()


      # ───── SERVICE CALLBACKS ────────────────────────
    def set_brake_cb(self, req, res):
        self.cmd['brake_command'] = req.data

        self.send_command()  # 👈 Send immediately

        brake_status = "ENGAGED" if req.data else "DISENGAGED"
        res.success = True
        res.message = f"Brake command set to {req.data} → {brake_status}"
        return res


    def set_motor_mode_cb(self, req, res):
        self.cmd['target_torque'] = 0.0
        self.cmd['target_velocity'] = 0.0
        self.cmd['target_position'] = 0.0
        self.cmd['motor_mode'] = req.data

        self.send_command()  # 👈 Send immediately

        mode_str = "CLOSED_LOOP" if req.data else "IDLE"
        res.success = True
        res.message = f"Motor mode set to {req.data} → {mode_str}"
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

