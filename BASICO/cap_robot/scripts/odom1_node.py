#!/usr/bin/env python3
# solo es ejemplo de la estructura
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf_transformations import quaternion_from_euler
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Publicador de odometría
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # Parámetros de inicialización
        self.x = 0.0  # Posición en x
        self.y = 0.0  # Posición en y
        self.theta = 0.0  # Orientación (ángulo)
        self.prev_time = self.get_clock().now()

        # Suscriptor a los encoders de los motores (aquí debes definir el tópico)
        self.create_subscription(Twist, 'encoder_data', self.encoder_callback, 10)

    def encoder_callback(self, msg):
        current_time = self.get_clock().now()
        delta_time = (current_time - self.prev_time).nanoseconds / 1e9  # Convertir a segundos
        self.prev_time = current_time

        # Suponiendo que msg.linear.x es la velocidad lineal y msg.angular.z es la velocidad angular
        vx = msg.linear.x
        vth = msg.angular.z

        # Calcular el cambio en la posición
        delta_x = vx * math.cos(self.theta) * delta_time
        delta_y = vx * math.sin(self.theta) * delta_time
        delta_theta = vth * delta_time

        # Actualizar la posición y la orientación
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Crear el mensaje de odometría
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        # Posición
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientación en cuaterniones
        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        # Velocidad
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth

        # Publicar el mensaje de odometría
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
