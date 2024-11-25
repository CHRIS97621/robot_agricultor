#!/usr/bin/env python3
# coding:UTF-8
import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        timer_period = 0.05  # Segundos entre publicaciones
        self.timer = self.create_timer(timer_period, self.GPS_read)
        port = '/dev/ttyUSB1'  # Asegúrate de que este sea el puerto correcto
        baud = 9600
        try:
            self.ser = serial.Serial(port, baud, timeout=2)
            self.get_logger().info(f"GPS Serial Opened! Baudrate={baud}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open GPS Serial: {e}")
            raise
        self.gps_t = 0

    def GPS_read(self):
        try:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='replace').strip()
                self.get_logger().info(f"Raw GPS Data: {line}")
                
                if line.startswith('$GNGGA'):
                    GGA = line.split(',')

                    if len(GGA) < 15:
                        self.get_logger().warn("GPS GGA data incomplete")
                        return

                    # Extraer y convertir latitud, longitud y altitud
                    lat = self.convert_to_degrees(GGA[2], GGA[3], True)
                    lon = self.convert_to_degrees(GGA[4], GGA[5], False)
                    alt = float(GGA[9]) if GGA[9] else 0.0
                    self.publish_gps_data(lat, lon, alt)
                else:
                    self.get_logger().warn("Non-GGA data received")
        except Exception as e:
            self.get_logger().error(f"Error processing GPS data: {e}")

    def convert_to_degrees(self, raw_value, direction, is_latitude):
        if not raw_value:
            return 0.0

        if is_latitude:
            degrees = float(raw_value[:2])
            minutes = float(raw_value[2:]) / 60.0
        else:
            degrees = float(raw_value[:3])
            minutes = float(raw_value[3:]) / 60.0

        result = degrees + minutes
        if direction in ['S', 'W']:
            result *= -1
        return result

    def publish_gps_data(self, lat, lon, alt):
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        #msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        #msg.status.status = -2  # Status no disponible
        #msg.status.service = 0
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published GPS data: lat={lat}, lon={lon}, alt={alt}")

def main(args=None):
    rclpy.init(args=args)
    try:
        gps_node = GPSNode()
        rclpy.spin(gps_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
