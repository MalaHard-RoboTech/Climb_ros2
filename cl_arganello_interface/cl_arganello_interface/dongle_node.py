#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String
import serial
import threading
import re

SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970047301-if00"
BAUDRATE = 115200

class DongleNode(Node):
    def __init__(self):
        super().__init__('dongle_node')

        # Serial init
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
            self.get_logger().info(f"✅ Serial port opened: {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to open serial: {e}")
            raise

        # Internal state
        self.lock = threading.Lock()
        self.cmd = {'sx': {}, 'dx': {}}
        self.manual_cmd = {'sx': [], 'dx': []}

        # Publishers
        self.pub = {
            'sx': self.create_publisher(Int32, 'arganello/sx/encoderCounts', 10),
            'dx': self.create_publisher(Int32, 'arganello/dx/encoderCounts', 10),
        }

        # Subscribers for command values
        for side in ['sx', 'dx']:
            for cmd_type in ['position', 'velocity', 'torque']:
                topic = f'arganello/{side}/{cmd_type}'
                self.create_subscription(Float32, topic, self.make_cmd_callback(side, cmd_type), 10)

            # Subscribers for raw string commands
            self.create_subscription(String, f'arganello/{side}/command', self.make_injection_callback(side), 10)

        # Serial loop thread
        self.thread = threading.Thread(target=self.serial_loop, daemon=True)
        self.thread.start()

    def make_cmd_callback(self, side, cmd_type):
        def callback(msg):
            with self.lock:
                self.cmd[side][cmd_type] = msg.data
            self.get_logger().info(f"📥 {side} → {cmd_type}: {msg.data}")
        return callback

    def make_injection_callback(self, side):
        def callback(msg):
            with self.lock:
                self.manual_cmd[side].append(msg.data)
            self.get_logger().info(f"🧪 Injected to {side}: '{msg.data}'")
        return callback

    def get_next_command(self, side):
        with self.lock:
            if self.manual_cmd[side]:
                return self.manual_cmd[side].pop(0)
            for key in ['torque', 'velocity', 'position']:
                if key in self.cmd[side]:
                    value = self.cmd[side].pop(key)
                    return f"{side}:{key}:{value}"
        return f"{side}:torque:0.0"

    def serial_loop(self):
        sides = ['sx', 'dx']
        index = 0
        while rclpy.ok():
            side = sides[index]
            try:
                cmd = self.get_next_command(side)
                self.ser.write((cmd + '\n').encode())
                self.get_logger().debug(f"📤 Sent to {side}: {cmd}")
                line = self.ser.readline().decode().strip()
                if line:
                    self.handle_serial_response(line)
            except Exception as e:
                self.get_logger().error(f"❌ Serial error: {e}")
            index = (index + 1) % 2

    def handle_serial_response(self, line):
        match = re.search(r"Arganello (SX|DX): ENC: (-?\d+)", line)
        if match:
            side = match.group(1).lower()
            count = int(match.group(2))
            self.pub[side].publish(Int32(data=count))
            self.get_logger().info(f"🧮 {side}/encoderCounts: {count}")
        else:
            self.get_logger().debug(f"💬 Ignored line: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = DongleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

