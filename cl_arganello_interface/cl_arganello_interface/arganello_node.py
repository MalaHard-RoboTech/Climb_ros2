#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool, Trigger
from cl_arganello_interface.msg import ArganelloRawTelemetry # type: ignore
from datetime import datetime
import serial
import time

class ArganelloNode(Node):
    def __init__(self):
        super().__init__('arganello_node')

        self.declare_parameter("serial_port", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970047399-if00")
        self.declare_parameter("arganello_id", "sx")
        self.declare_parameter("poll_rate", 200.0)  # ✅ New parameter for polling frequency

        self.port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.id = self.get_parameter("arganello_id").get_parameter_value().string_value
        self.poll_rate = self.get_parameter("poll_rate").get_parameter_value().double_value

        try:
            self.ser = serial.Serial(self.port, 1000000, timeout=0.05)
            self.get_logger().info(f"✅ Serial opened: {self.port}")
            self.ser.flushInput()
        except Exception as e:
            self.get_logger().error(f"❌ Failed to open serial: {e}")
            raise

        naming = f"/arganello/{self.id}"
        self.telemetry_pub = self.create_publisher(ArganelloRawTelemetry, f"{naming}/telemetry/raw", 10)
        self.csv_pub = self.create_publisher(String, f"{naming}/telemetry/raw/csv", 10)

        self.create_subscription(Float32, f"{naming}/target_torque", self.torque_cb, 10)
        self.create_subscription(Float32, f"{naming}/target_velocity", self.velocity_cb, 10)
        self.create_subscription(Float32, f"{naming}/target_position", self.position_cb, 10)

        self.create_service(SetBool, f"{naming}/set_brake", self.set_brake_cb)
        self.create_service(Trigger, f"{naming}/set_idle", self.set_idle_cb)
        self.create_service(Trigger, f"{naming}/set_closed_loop", self.set_closed_loop_cb)
        self.create_service(Trigger, f"{naming}/set_torque_mode", self.set_torque_mode_cb)
        self.create_service(Trigger, f"{naming}/set_velocity_mode", self.set_velocity_mode_cb)
        self.create_service(Trigger, f"{naming}/set_position_mode", self.set_position_mode_cb)

        

        self.create_timer(1.0 / self.poll_rate, self.poll_serial)

    def poll_serial(self):

        if self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                # Split the CSV line into individual fields
                fields = line.split(',')
                # If there are at least 15 fields, process it as raw CSV
                if len(fields) >= 15:
                    # Replace the first field (usually an ESP32 timestamp) with the current UNIX time
                    fields[0] = str(int(time.time()))
                    # Rebuild the modified CSV line
                    modified_line = ','.join(fields)
                    # Publish the modified CSV string to a topic for later logging/analysis
                    self.csv_pub.publish(String(data=modified_line))

            # If the CSV line has at least 14 commas (i.e., 15 fields), try to parse it into the custom message
            if line.count(',') >= 13:
                try:
                    # Create a new message of type ArganelloRawTelemetry
                    msg = ArganelloRawTelemetry()

                    # Add a ROS timestamp to the message header
                    msg.header.stamp = self.get_clock().now().to_msg()

                    # Parse and assign each telemetry field from the CSV
                    msg.brake_status = bool(int(fields[1]))
                    msg.vbus_voltage = float(fields[2])
                    msg.ibus = float(fields[3])
                    msg.motor_temperature = float(fields[4])
                    
                    msg.input_position = float(fields[5])
                    msg.input_velocity = float(fields[6])
                    msg.input_torque = float(fields[7])
                    msg.motor_torque = float(fields[8])
                    msg.motor_pos = float(fields[9])
                    msg.motor_vel = float(fields[10])
                    msg.sync_roller_pos = float(fields[11])
                    msg.sync_roller_vel = float(fields[12])
                    msg.sync_roller_raw = int(fields[13])

                    # Publish the parsed structured telemetry message
                    self.telemetry_pub.publish(msg)

                except Exception as e:
                    # If any parsing or conversion fails, warn and continue
                    self.get_logger().warn(f"❌ Telemetry parse error: {e}")



    def torque_cb(self, msg):
        self.ser.write(f"send_odrive w axis0.controller.input_torque {msg.data}\n".encode())

    def velocity_cb(self, msg):
        self.ser.write(f"send_odrive w axis0.controller.input_vel {msg.data}\n".encode())

    def position_cb(self, msg):
        self.ser.write(f"send_odrive w axis0.controller.input_pos {msg.data}\n".encode())

    def set_brake_cb(self, req, res):
        self.ser.write(f"set_brake {int(req.data)}\n".encode())
        res.success = True
        res.message = f"Brake {'engaged' if req.data else 'released'}"
        return res

    def set_idle_cb(self, req, res):
        self.ser.write(b"send_odrive w axis0.requested_state 1\n")
        res.success = True
        res.message = "Set to IDLE"
        return res

    def set_closed_loop_cb(self, req, res):
        self.ser.write(b"send_odrive w axis0.requested_state 8\n")
        res.success = True
        res.message = "Set to CLOSED_LOOP_CONTROL"
        return res

    def set_torque_mode_cb(self, req, res):
        self.ser.write(b"send_odrive w axis0.controller.config.control_mode 1\n")
        res.success = True
        res.message = "Control mode set to TORQUE"
        return res

    def set_velocity_mode_cb(self, req, res):
        self.ser.write(b"send_odrive w axis0.controller.config.control_mode 2\n")
        res.success = True
        res.message = "Control mode set to VELOCITY"
        return res

    def set_position_mode_cb(self, req, res):
        self.ser.write(b"send_odrive w axis0.controller.config.control_mode 3\n")
        res.success = True
        res.message = "Control mode set to POSITION"
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


if __name__ == '__main__':
    main()
