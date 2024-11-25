#!/usr/bin/env python3
# Nodo GPS
# scripts/gps_node.py

import rclpy
from rclpy.node import Node
import serial
import re

class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_node2')
        self.declare_parameter('port','/dev/ttyUSB0')
        self.declare_parameter('baud',9600)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.ser = serial.Serial(port, baud, timeout=2)
        #self.timer = self.create_timer(0.5, self.timer_callback)
        #self.ser = serial.Serial(self.get_parameter('serial_port').get_parameter_value().string_value, 9600)
    

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

        #self.timer = self.create_timer(1.0, self.GPS_read)
    def timer_callback(self):
        if self.GPS_read():
            self.log_gps_data()

    def log_gps_data(self):
        self.get_logger().info("*********************")
        self.get_logger().info(f'UTC Time: {self.utctime}')
        self.get_logger().info(f'Latitude: {self.lat}{self.ulat}')
        self.get_logger().info(f'Longitude: {self.lon}{self.ulon}')
        self.get_logger().info(f'Number of satellites: {self.numSv}')
        self.get_logger().info(f'Altitude: {self.msl}')
        self.get_logger().info(f'True north heading: {self.cogt}°')
        self.get_logger().info(f'Magnetic north heading: {self.cogm}°')
        self.get_logger().info(f'Ground speed: {self.sog} Kn')
        self.get_logger().info(f'Ground speed: {self.kph} Km/h')
        self.get_logger().info("*********************")
    
    def Convert_to_degrees(self, in_data1, in_data2):
        len_data1 = len(in_data1)
        str_data2 = "%05d" % int(in_data2)
        temp_data = int(in_data1)
        symbol = 1
        if temp_data < 0:
            symbol = -1
        degree = int(temp_data / 100.0)
        str_decimal = str(in_data1[len_data1-2]) + str(in_data1[len_data1-1]) + str(str_data2)
        f_degree = int(str_decimal) / 60.0 / 100000.0
        if symbol > 0:
            result = degree + f_degree
        else:
            result = degree - f_degree
        return result

    
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
                                                GGA = self.ser.read(70) #formato de mensaje nmea
                                                GGA_g = re.findall(r"\w+(?=,)|(?<=,)\w+", str(GGA))
                                                if len(GGA_g) < 13:
                                                    self.get_logger().warn("GPS not found")
                                                    self.gps_t = 0
                                                    return 
                                                else:
                                                    self.utctime = GGA_g[0]
                                                    #self.lat = GGA_g[2][0] + GGA_g[2][1] + 'degree' + GGA_g[2][2] + GGA_g[2][3] + '.' + GGA_g[3] + '\''
                                                    self.lat = "%.8f" % self.Convert_to_degrees(str(GGA_g[2]), str(GGA_g[3]))
                                                    self.ulat = GGA_g[4]
                                                    #self.lon = GGA_g[5][0] + GGA_g[5][1] + GGA_g[5][2] + 'degree' + GGA_g[5][3] + GGA_g[5][4] + '.' + GGA_g[6] + '\''
                                                    self.lon = "%.8f" % self.Convert_to_degrees(str(GGA_g[5]), str(GGA_g[6]))
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
    def Convert_to_degrees(self, in_data1, in_data2):
        len_data1 = len(in_data1)
        str_data2 = "%05d" % int(in_data2)
        temp_data = int(in_data1)
        symbol = 1
        if temp_data < 0:
            symbol = -1
        degree = int(temp_data / 100.0)
        str_decimal = str(in_data1[len_data1-2]) + str(in_data1[len_data1-1]) + str(str_data2)
        f_degree = int(str_decimal) / 60.0 / 100000.0
        if symbol > 0:
            result = degree + f_degree
        else:
            result = degree - f_degree
        return result    
    

    

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