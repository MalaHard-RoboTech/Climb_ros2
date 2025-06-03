#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import re
import threading

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')

        # Serial communication
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.running = True

        # ROS setup
        self.publisher_ = self.create_publisher(String, 'encoder_data', 10)
        self.subscription = self.create_subscription(
            String,
            'arganello_dx_command',
            self.command_callback,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.encoder_offset = None
        self.enc_pattern = re.compile(r'ENC:\s*(-?\d+)')  # support negatives

        # Start serial reading in separate thread
        self.serial_thread = threading.Thread(target=self.read_serial_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def command_callback(self, msg: String):
        """Callback to send received command to the Arduino."""
        try:
            data = msg.data.strip() + '\n'  # ensure newline for Arduino parsing
            self.ser.write(data.encode('utf-8'))
            self.get_logger().info(f"➡️ Comando inviato all'Arduino: '{msg.data}'")
        except serial.SerialException as e:
            self.get_logger().error(f"Errore durante l'invio seriale: {str(e)}")

    def read_serial_loop(self):
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    self.publisher_.publish(String(data=line))
                    self.get_logger().info(f'📤 Pubblicato: {line}')

                    match = self.enc_pattern.search(line)
                    if match:
                        try:
                            enc_value = int(match.group(1))
                        except ValueError:
                            self.get_logger().warn(f"Valore encoder non valido: {match.group(1)}")
                            continue

                        if self.encoder_offset is None:
                            self.encoder_offset = enc_value
                            self.get_logger().info(f'Offset iniziale impostato a {self.encoder_offset}')

                        relative_pos = enc_value - self.encoder_offset
                        meters = float(relative_pos) * 0.000395

                        t = TransformStamped()
                        t.header.stamp = self.get_clock().now().to_msg()
                        t.header.frame_id = 'world'
                        t.child_frame_id = 'finecorsa'
                        t.transform.translation.x = 0.0
                        t.transform.translation.y = 0.0
                        t.transform.translation.z = meters
                        t.transform.rotation.x = 0.0
                        t.transform.rotation.y = 0.0
                        t.transform.rotation.z = 0.0
                        t.transform.rotation.w = 1.0
                        self.tf_broadcaster.sendTransform(t)
                    else:
                        self.get_logger().warn(f"⚠️ Stringa non compatibile: '{line}'")
            except serial.SerialException:
                self.get_logger().error("❌ Errore seriale! Controlla il collegamento USB.")
                self.running = False
            except Exception as e:
                self.get_logger().error(f"❗ Errore imprevisto: {str(e)}")

    def destroy_node(self):
        self.running = False
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

