#!/usr/bin/env python3
# Nodo GPS
# scripts/gps_node.py

import rclpy
from rclpy.node import Node
import serial
import re
from sensor_msgs.msg import NavSatFix  # Importa el mensaje NavSatFix

class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_node')
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 9600)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.ser = serial.Serial(port, baud, timeout=2)
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Llama a timer_callback cada 1 segundo
        self.get_logger().info("GPS Serial Opened! Baudrate=%d" % baud)

        # Variables para almacenar datos GPS
        self.lat = ''
        self.lon = ''
        self.alt = ''

    def timer_callback(self):
        if self.GPS_read():
            self.publish_gps_data()

    def publish_gps_data(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Ajusta esto según tu marco de referencia
        msg.latitude = float(self.lat) if self.lat else 0.0
        msg.longitude = float(self.lon) if self.lon else 0.0
        msg.altitude = float(self.alt) if self.alt else 0.0

        self.publisher_.publish(msg)

    def Convert_to_degrees(self, value, direction):
        try:
            if direction == 'N' or direction == 'S':
                degrees = float(value[:2])
                minutes = float(value[2:])
                decimal_degrees = degrees + minutes / 60.0
                if direction == 'S':
                    decimal_degrees = -decimal_degrees
            elif direction == 'E' or direction == 'W':
                degrees = float(value[:3])
                minutes = float(value[3:])
                decimal_degrees = degrees + minutes / 60.0
                if direction == 'W':
                    decimal_degrees = -decimal_degrees
            return decimal_degrees
        except ValueError:
            self.get_logger().error(f"Error converting to degrees: value={value}, direction={direction}")
            return 0.0

    def GPS_read(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('ascii', errors='replace').strip()
                self.get_logger().info(f"Raw GPS Data: {line}")
                if line.startswith('$GNGLL'):
                    parts = line.split(',')
                    if len(parts) > 4:
                        self.lat = self.Convert_to_degrees(parts[1], parts[2])
                        self.lon = self.Convert_to_degrees(parts[3], parts[4])
                        self.alt = parts[9] if len(parts) > 9 else '0.0'
                        return True
        except Exception as e:
            self.get_logger().error(f"Error reading GPS data: {e}")
        return False

def main(args=None):
    rclpy.init(args=args)
    gps_node = GPSNode()
    try:
        rclpy.spin(gps_node)
    except KeyboardInterrupt:
        gps_node.get_logger().info("Shutting down...")
    finally:
        gps_node.ser.close()
        gps_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
