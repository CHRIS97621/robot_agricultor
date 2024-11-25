#!/usr/bin/env python3
import time
import math
import serial
import struct
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Variables globales para almacenar datos de la IMU
key = 0
buff = {}
angularVelocity = [0.0, 0.0, 0.0]  # Inicialización como float
acceleration = [0.0, 0.0, 0.0]     # Inicialización como float
magnetometer = [0.0, 0.0, 0.0]     # Inicialización como float
angle_degree = [0.0, 0.0, 0.0]     # Inicialización como float

# Funciones de procesamiento de datos
def hex_to_short(raw_data):
    """Convierte datos hexadecimales a valores enteros"""
    return list(struct.unpack("hhhh", bytearray(raw_data)))

def check_sum(list_data, check_data):
    """Verifica el checksum de los datos recibidos"""
    return sum(list_data) & 0xff == check_data

def handle_serial_data(raw_data):
    """Procesa los datos de la IMU recibidos por el puerto serial"""
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity

    buff[key] = raw_data
    key += 1

    if buff[0] != 0x55:
        key = 0
        return False

    if key < 11:
        return False
    else:
        data_buff = list(buff.values())  # Obtiene los valores del diccionario
        if buff[1] == 0x51:
            if check_sum(data_buff[0:10], data_buff[10]):
                
                acceleration = [float(hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8) for i in range(3)]
            else:
                print('0x51 Check failure')

        elif buff[1] == 0x52:
            if check_sum(data_buff[0:10], data_buff[10]):
                
                angularVelocity = [float(hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180) for i in range(3)]
            else:
                print('0x52 Check failure')

        elif buff[1] == 0x53:
            if check_sum(data_buff[0:10], data_buff[10]):
                
                angle_degree = [float(hex_to_short(data_buff[2:10])[i] / 32768.0 * 180) for i in range(3)]
                return True
            else:
                print('0x53 Check failure')

        elif buff[1] == 0x54:
            if check_sum(data_buff[0:10], data_buff[10]):
                
                magnetometer = [float(hex_to_short(data_buff[2:10])[i]) for i in range(3)]
            else:
                print('0x54 Check failure')

        buff = {}
        key = 0
        return False

def get_quaternion_from_euler(roll, pitch, yaw):
    """Convierte ángulos de Euler a una cuaternión"""
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]

class IMUDriverNode(Node):
    def __init__(self, port_name):
        super().__init__('imu_driver_node')

        # Inicializar el mensaje de IMU
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'

        # Crear publicador de IMU
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # Iniciar el hilo para el bucle de la IMU
        self.driver_thread = threading.Thread(target=self.driver_loop, args=(port_name,))
        self.driver_thread.start()

    def driver_loop(self, port_name):
        """Bucle principal de lectura de datos del puerto serial"""
        try:
            wt_imu = serial.Serial(port=port_name, baudrate=9600, timeout=0.1)
            if wt_imu.isOpen():
                self.get_logger().info("Serial port opened successfully...")
            else:
                wt_imu.open()
                self.get_logger().info("Serial port opened successfully...")
        except Exception as e:
            self.get_logger().error(f"Serial port opening failure: {e}")
            exit(0)

        while True:
            try:
                buff_count = wt_imu.inWaiting()
            except Exception as e:
                self.get_logger().error(f"IMU disconnect: {e}")
                exit(0)
            else:
                if buff_count > 0:
                    buff_data = wt_imu.read(buff_count)
                    for i in range(0, buff_count):
                        tag = handle_serial_data(buff_data[i])
                        if tag:
                            self.imu_data()

    def imu_data(self):
        """Actualiza y publica los datos de la IMU"""
        # Convertir los valores a float antes de asignarlos al mensaje Imu
        accel_x, accel_y, accel_z = float(acceleration[0]), float(acceleration[1]), float(acceleration[2])
        gyro_x, gyro_y, gyro_z = float(angularVelocity[0]), float(angularVelocity[1]), float(angularVelocity[2])

        # Configurar la orientación en cuaternión
        angle_radian = [float(angle_degree[i] * math.pi / 180) for i in range(3)]
        qua = get_quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

        # Actualizar el mensaje IMU
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.linear_acceleration.x = accel_x
        self.imu_msg.linear_acceleration.y = accel_y
        self.imu_msg.linear_acceleration.z = accel_z
        self.imu_msg.angular_velocity.x = gyro_x
        self.imu_msg.angular_velocity.y = gyro_y
        self.imu_msg.angular_velocity.z = gyro_z
        self.imu_msg.orientation.x = qua[0]
        self.imu_msg.orientation.y = qua[1]
        self.imu_msg.orientation.z = qua[2]
        self.imu_msg.orientation.w = qua[3]

        # Publicar el mensaje IMU
        self.imu_pub.publish(self.imu_msg)

def main():
    """Función principal que inicia el nodo"""
    rclpy.init()
    node = IMUDriverNode('/dev/ttyUSB0')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()