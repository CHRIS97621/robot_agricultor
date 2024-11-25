#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        
        # Configuración serial
        self.serial_port = '/dev/ttyUSB0'  # Asegúrate de que coincide con el puerto de tu ESP32
        self.baud_rate = 115200
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Conectado al ESP32 en {self.serial_port} a {self.baud_rate} bps")
        except serial.SerialException as e:
            self.get_logger().error(f"No se pudo conectar al ESP32: {e}")
            raise

    def listener_callback(self, msg):
        # Extraer los valores de velocidad lineal y angular
        desired_vx = msg.linear.x
        desired_vtheta = msg.angular.z
        
        # Crear el mensaje en formato "vx,vtheta\n" para enviar al ESP32
        message = f"{desired_vx},{desired_vtheta}\n"
        self.ser.write(message.encode())
        self.get_logger().info(f"Enviando al ESP32: {message}")

def main(args=None):
    rclpy.init(args=args)
    serial_bridge = SerialBridge()
    rclpy.spin(serial_bridge)
    serial_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
