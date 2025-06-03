#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import time

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        self.publisher_ = self.create_publisher(String, 'encoder_data', 10)
        # Modifica la porta seriale secondo necessità
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_serial)
        self.tf_broadcaster = TransformBroadcaster(self)

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            msg = String()
            msg.data = line
            self.publisher_.publish(msg)
            self.get_logger().info(f'Pubblicato: {line}')   

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').rstrip()
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
                self.get_logger().info(f'Pubblicato: {line}')

                # Esempio: invia sempre una posizione fissa (da personalizzare)
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'base_link'
                t.child_frame_id = 'encoder_link'
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                
                try:
                    t.transform.translation.z = float(line)
                except ValueError:
                    self.get_logger().warn(f"Ricevuto valore non numerico dalla seriale: '{line}'")
                    return  # Esci senza inviare la trasformazione
                
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0

                self.tf_broadcaster.sendTransform(t)
        except serial.SerialException:
            self.get_logger().error("Seriale disconnessa! Provo a riconnettere...")
            self.ser.close()
            self.ser = self.connect_serial()

    def destroy_node(self):
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