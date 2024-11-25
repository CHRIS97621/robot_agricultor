#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        timer_period = 1.0  # Publicar cada 1 segundo
        self.timer = self.create_timer(timer_period, self.publish_gps_data)
        self.get_logger().info('GPS Publisher Node iniciado.')

    def publish_gps_data(self):
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Aquí ajustas el frame_id a uno de los marcos de referencia que desees (por ejemplo, 'map' o 'odom')
        gps_msg.header.frame_id = 'map'  # Cambia esto a 'odom' o 'map' según lo que uses para la fusión
        
        gps_msg.latitude = -12.02453
        gps_msg.longitude = -77.04779
        gps_msg.altitude = 0.0  # Altitud fija (puedes ajustar según tus necesidades)
        gps_msg.position_covariance = [0.0] * 9  # Sin covarianza para simplificar
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.publisher_.publish(gps_msg)
        self.get_logger().info(f'Publicado GPS: lat={gps_msg.latitude}, lon={gps_msg.longitude}')

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    try:
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        gps_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
