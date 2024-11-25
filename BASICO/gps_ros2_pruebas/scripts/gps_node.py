#!/usr/bin/env python3
# Nodo GPS
# scripts/gps_node.py

import rclpy
from rclpy.node import Node
import serial
import re

class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_node')
        self.ser = serial.Serial("/dev/ttyUSB0", 9600)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.ser = serial.Serial(self.get_parameter('serial_port').get_parameter_value().string_value, 9600)

        if self.ser.isOpen():
            self.get_logger().info("GPS Serial Opened! Baudrate=9600")
        else:
            self.get_logger().error("GPS Serial Open Failed!")

        # Variables para almacenar datos GPS
        self.utctime = ''
        self.lat = ''
        self.ulat = ''
        self.lon = ''
        self.ulon = ''
        self.numSv = ''
        self.msl = ''
        self.cogt = ''
        self.cogm = ''
        self.sog = ''
        self.kph = ''
        self.gps_t = 0

        self.timer = self.create_timer(1.0, self.GPS_read)

    def GPS_read(self):
        if self.ser.inWaiting():
            if self.ser.read(1) == b'G':
                if self.ser.inWaiting():
                    if self.ser.read(1) == b'N':
                        if self.ser.inWaiting():
                            choice = self.ser.read(1)
                            if choice == b'G':
                                if self.ser.inWaiting():
                                    if self.ser.read(1) == b'G':
                                        if self.ser.inWaiting():
                                            if self.ser.read(1) == b'A':
                                                GGA = self.ser.read(70)
                                                GGA_g = re.findall(r"\w+(?=,)|(?<=,)\w+", str(GGA))
                                                if len(GGA_g) < 13:
                                                    self.get_logger().warn("GPS not found")
                                                    self.gps_t = 0
                                                    return
                                                else:
                                                    self.utctime = GGA_g[0]
                                                    self.lat = GGA_g[2][0] + GGA_g[2][1] + 'degree' + GGA_g[2][2] + GGA_g[2][3] + '.' + GGA_g[3] + '\''
                                                    self.ulat = GGA_g[4]
                                                    self.lon = GGA_g[5][0] + GGA_g[5][1] + GGA_g[5][2] + 'degree' + GGA_g[5][3] + GGA_g[5][4] + '.' + GGA_g[6] + '\''
                                                    self.ulon = GGA_g[7]
                                                    self.numSv = GGA_g[9]
                                                    self.msl = GGA_g[12] + '.' + GGA_g[13] + GGA_g[14]
                                                    self.gps_t = 1
                            elif choice == b'V':
                                if self.ser.inWaiting():
                                    if self.ser.read(1) == b'T':
                                        if self.ser.inWaiting():
                                            if self.ser.read(1) == b'G':
                                                if self.gps_t == 1:
                                                    VTG = self.ser.read(40)
                                                    VTG_g = re.findall(r"\w+(?=,)|(?<=,)\w+", str(VTG))
                                                    self.cogt = VTG_g[0] + '.' + VTG_g[1] + 'T'
                                                    if VTG_g[3] == 'M':
                                                        self.cogm = '0.00'
                                                        self.sog = VTG_g[4] + '.' + VTG_g[5]
                                                        self.kph = VTG_g[7] + '.' + VTG_g[8]
                                                    elif VTG_g[3] != 'M':
                                                        self.cogm = VTG_g[3] + '.' + VTG_g[4]
                                                        self.sog = VTG_g[6] + '.' + VTG_g[7]
                                                        self.kph = VTG_g[9] + '.' + VTG_g[10]
        if self.gps_t:
            self.get_logger().info("*********************")
            self.get_logger().info('UTC Time:' + self.utctime)
            self.get_logger().info('Latitude:' + self.lat + self.ulat)
            self.get_logger().info('Longitude:' + self.lon + self.ulon)
            self.get_logger().info('Number of satellites:' + self.numSv)
            self.get_logger().info('Altitude:' + self.msl)
            self.get_logger().info('True north heading:' + self.cogt + 'degree')
            self.get_logger().info('Magnetic north heading:' + self.cogm + 'degree')
            self.get_logger().info('Ground speed:' + self.sog + 'Kn')
            self.get_logger().info('Ground speed:' + self.kph + 'Km/h')
            self.get_logger().info("*********************")


def main(args=None):
    rclpy.init(args=args)
    gps_node = GPSNode()
    try:
        rclpy.spin(gps_node)
    except KeyboardInterrupt:
        gps_node.ser.close()
        gps_node.get_logger().info("GPS serial Close!")
    finally:
        gps_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
