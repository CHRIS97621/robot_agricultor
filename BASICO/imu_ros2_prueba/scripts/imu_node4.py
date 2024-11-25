#!/usr/bin/env python3
# coding:UTF-8
# Version: V2.0.4
import serial                   # Comunicación por puerto serial
import rclpy                    # Paquete de python para ROS2
from rclpy.node import Node     # Importamos la clase nodo
from geometry_msgs.msg import Quaternion # geometry mensaje
from sensor_msgs.msg import Imu # sensor mensaje imu
import tf_transformations       # Para manejar las transformaciones de ángulos a cuaterniones

# Almacenamiento de la data cruda
ACCData = [0.0] * 8             # 8 bytes que representan la lectura en los ejes x,y,z del acelerómetro
GYROData = [0.0] * 8            # 8 bytes que representan la lectura en los ejes x,y,z del giroscopio
AngleData = [0.0] * 8           # 8 bytes que representan la lectura en los ángulos de ejes
FrameState = 0                  # Estado de la lectura de datos
Bytenum = 0                     # Contador de bytes leídos
CheckSum = 0                    # Bit de verificación de datos recibidos


# Variables globales para almacenar los valores procesados
acc = None
gyro = None
angle = None


class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.ser = serial.Serial(port, baud, timeout=0.5)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Serial is Opened: %s" % self.ser.is_open)

    def timer_callback(self):
        datahex = self.ser.read(33)
        self.DueData(datahex)

        # Verifica si todos los datos (acc, gyro, angle) están disponibles
        if acc is not None and gyro is not None and angle is not None:
            quaternion = self.compute_quaternion_from_angle(angle)  # Calcula el cuaternión desde los ángulos de Euler
            print(
                "acc:      %10.3f  %10.3f  %10.3f\n"
                "gyro:     %10.3f  %10.3f  %10.3f\n"
                "angle:    %10.3f  %10.3f  %10.3f\n"
                "quaternion: %10.3f  %10.3f  %10.3f  %10.3f\n" %
                (acc[0], acc[1], acc[2], 
                 gyro[0], gyro[1], gyro[2], 
                 angle[0], angle[1], angle[2],
                 quaternion[0], quaternion[1], quaternion[2], quaternion[3])
            )

    def compute_quaternion_from_angle(self, angle):
        # Convierte los ángulos de Euler (roll, pitch, yaw) a un cuaternión
        quaternion = tf_transformations.quaternion_from_euler(
            angle[0], angle[1], angle[2]
        )
        return quaternion

    def DueData(self, inputdata):
        global FrameState, Bytenum, CheckSum, acc, gyro, angle
        for data in inputdata:  # Recorre los datos de entrada
            if FrameState == 0:  # Estado inicial
                if data == 0x55 and Bytenum == 0:  # Primer byte 0x55
                    CheckSum = data
                    Bytenum = 1
                    continue
                elif data == 0x51 and Bytenum == 1:  # Identificación de la trama de acelerómetro
                    CheckSum += data
                    FrameState = 1
                    Bytenum = 2
                elif data == 0x52 and Bytenum == 1:  # Identificación de la trama de giroscopio
                    CheckSum += data
                    FrameState = 2
                    Bytenum = 2
                elif data == 0x53 and Bytenum == 1:  # Identificación de la trama de ángulo
                    CheckSum += data
                    FrameState = 3
                    Bytenum = 2
            elif FrameState == 1:  # Acelerómetro
                if Bytenum < 10:  # Lee 8 bytes de datos
                    ACCData[Bytenum - 2] = data
                    CheckSum += data
                    Bytenum += 1
                else:
                    if data == (CheckSum & 0xff):  # Verifica el bit de comprobación
                        acc = self.get_acc(ACCData)
                    self.reset_frame()
            elif FrameState == 2:  # Giroscopio
                if Bytenum < 10:
                    GYROData[Bytenum - 2] = data
                    CheckSum += data
                    Bytenum += 1
                else:
                    if data == (CheckSum & 0xff):
                        gyro = self.get_gyro(GYROData)
                    self.reset_frame()
            elif FrameState == 3:  # Ángulo
                if Bytenum < 10:
                    AngleData[Bytenum - 2] = data
                    CheckSum += data
                    Bytenum += 1
                else:
                    if data == (CheckSum & 0xff):
                        angle = self.get_angle(AngleData)
                    self.reset_frame()

    def reset_frame(self):
        global FrameState, Bytenum, CheckSum
        FrameState = 0
        Bytenum = 0
        CheckSum = 0

    def get_acc(self, datahex):
        axl = datahex[0]
        axh = datahex[1]
        ayl = datahex[2]
        ayh = datahex[3]
        azl = datahex[4]
        azh = datahex[5]
        k_acc = 16.0
        acc_x = (axh << 8 | axl) / 32768.0 * k_acc
        acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
        acc_z = (azh << 8 | azl) / 32768.0 * k_acc
        if acc_x >= k_acc:
            acc_x -= 2 * k_acc
        if acc_y >= k_acc:
            acc_y -= 2 * k_acc
        if acc_z >= k_acc:
            acc_z -= 2 * k_acc
        return acc_x, acc_y, acc_z

    def get_gyro(self, datahex):
        wxl = datahex[0]
        wxh = datahex[1]
        wyl = datahex[2]
        wyh = datahex[3]
        wzl = datahex[4]
        wzh = datahex[5]
        k_gyro = 2000.0
        gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
        gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
        gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
        if gyro_x >= k_gyro:
            gyro_x -= 2 * k_gyro
        if gyro_y >= k_gyro:
            gyro_y -= 2 * k_gyro
        if gyro_z >= k_gyro:
            gyro_z -= 2 * k_gyro
        return gyro_x, gyro_y, gyro_z

    def get_angle(self, datahex):
        rxl = datahex[0]
        rxh = datahex[1]
        ryl = datahex[2]
        ryh = datahex[3]
        rzl = datahex[4]
        rzh = datahex[5]
        k_angle = 180.0
        angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
        angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
        angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
        if angle_x >= k_angle:
            angle_x -= 2 * k_angle
        if angle_y >= k_angle:
            angle_y -= 2 * k_angle
        if angle_z >= k_angle:
            angle_z -= 2 * k_angle
        return angle_x, angle_y, angle_z


def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
