import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import geodesy.utm # type: ignore
import math

class GpsRouteNavigationNode(Node):
    def __init__(self):
        super().__init__('gps_route_navigation_node')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        
        # Lista de puntos de la trayectoria en coordenadas GPS
        self.route_points = [
            {"lat": -12.024839, "lon": -77.047443},
            {"lat": -12.024839, "lon": -77.08989},
            {"lat": -12.082539, "lon": -77.08989},
        ]
        
        self.current_point_index = 0  # Empezamos en el primer punto
        self.initial_utm_position = None
        self.goal_reached_threshold = 1.0  # Distancia en metros para considerar que ha llegado al objetivo

    def gps_callback(self, msg):
        # Convertir la posición GPS actual a UTM
        current_utm_position = geodesy.utm.fromLatLong(msg.latitude, msg.longitude)

        if self.initial_utm_position is None:
            # Usar la primera posición GPS como origen
            self.initial_utm_position = current_utm_position

        # Obtener el siguiente punto de destino
        if self.current_point_index < len(self.route_points):
            target_gps = self.route_points[self.current_point_index]
            target_utm_position = geodesy.utm.fromLatLong(target_gps["lat"], target_gps["lon"])

            # Convertir a coordenadas locales
            x = target_utm_position.easting - self.initial_utm_position.easting
            y = target_utm_position.northing - self.initial_utm_position.northing

            # Verificar si hemos alcanzado el objetivo actual
            distance_to_goal = math.sqrt((current_utm_position.easting - target_utm_position.easting) ** 2 +
                                         (current_utm_position.northing - target_utm_position.northing) ** 2)
            if distance_to_goal < self.goal_reached_threshold:
                self.get_logger().info(f'Punto alcanzado: ({x}, {y}), avanzando al siguiente punto.')
                self.current_point_index += 1
            else:
                # Publicar el objetivo actual si aún no se ha alcanzado
                goal_msg = PoseStamped()
                goal_msg.header.frame_id = 'map'
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                goal_msg.pose.position.x = x
                goal_msg.pose.position.y = y
                goal_msg.pose.orientation.w = 1.0
                self.goal_pub.publish(goal_msg)
                self.get_logger().info(f'Objetivo de navegación publicado en (x: {x}, y: {y})')

def main(args=None):
    rclpy.init(args=args)
    node = GpsRouteNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
