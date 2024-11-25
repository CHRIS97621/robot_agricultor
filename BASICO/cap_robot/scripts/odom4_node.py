#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import serial
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float32MultiArray

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Publicador de odometría
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Suscriptor para recibir comandos de velocidad desde la interfaz gráfica
        self.create_subscription(Float32MultiArray, 'control_commands', self.control_callback, 10)

        # Inicializar variables de posición y orientación
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientación en radianes
        self.vx = 0.0
        self.vtheta = 0.0

        # Conexión con el ESP32
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 115200
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Conectado al ESP32 en {self.serial_port} a {self.baud_rate} bps")
        except serial.SerialException as e:
            self.get_logger().error(f"No se pudo conectar al ESP32: {e}")
            raise

        # Configuración del temporizador
        timer_period = 0.1  # Frecuencia de actualización en segundos (10 Hz)
        self.timer = self.create_timer(timer_period, self.update_odometry)

    def control_callback(self, msg):
        # Actualizar los comandos de velocidad recibidos desde la interfaz
        self.vx = msg.data[0]
        self.vtheta = msg.data[1]

        # Enviar los comandos de velocidad al ESP32 vía serial
        if self.ser.is_open:
            self.ser.write(f"{self.vx},{self.vtheta}\n".encode())

    def update_odometry(self):
        # Leer datos de velocidad actuales desde el ESP32
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('ascii').strip()
            try:
                vx, vtheta = map(float, line.split(','))
            except ValueError:
                self.get_logger().warn("Datos inválidos recibidos del ESP32.")
                return
        else:
            vx, vtheta = 0.0, 0.0

        dt = 0.1  # Intervalo de tiempo en segundos
        delta_x = vx * math.cos(self.theta) * dt
        delta_y = vx * math.sin(self.theta) * dt
        delta_theta = vtheta * dt

        # Actualizar la posición y orientación del robot
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Convertir la orientación a cuaternión
        quaternion = quaternion_from_euler(0, 0, self.theta)

        # Publicar los datos de odometría
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Posición
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        # Orientación en cuaterniones
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        # Publicar el mensaje de odometría
        self.odom_publisher.publish(odom_msg)

        # Publicar la transformación TF
        self.publish_tf(self.x, self.y, quaternion)

    def publish_tf(self, x, y, quaternion):
        # Crear y publicar la transformación desde 'odom' hasta 'base_link'
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0  # Asume que el robot se mueve en un plano

        # Orientación
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdomNode()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
