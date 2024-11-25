#!/usr/bin/env python3

# 1. LIBRARIES
import sys
import numpy as np
from PySide2.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton, 
                               QGroupBox, QGridLayout, QLineEdit, QLabel)
from PySide2.QtGui import QFont
from PySide2.QtCore import QTimer
import rclpy
#from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class Ventana(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("My teleoperator")
        self.setGeometry(10, 20, 300, 200)

        # Configuración de ROS2
        rclpy.init()
        self.node = rclpy.create_node('interfaz3')
        self.pub = self.node.create_publisher(Twist, 'control_commands', 10)

        self.node.declare_parameter('vx_max', 0.5)
        self.node.declare_parameter('vtheta_max', 0.8)
        
        self.vx_max = self.node.get_parameter('vx_max').value
        self.vtheta_max = self.node.get_parameter('vtheta_max').value
        
        self.node.get_logger().info(f'Maximum linear speed (vx) = {self.vx_max}')
        self.node.get_logger().info(f'Maximum angular speed (vtheta) = {self.vtheta_max}')
        
        # Inicializar comandos de control
        self.desired_vx = 0.0
        self.desired_vtheta = 0.0

        # Timer para publicar comandos a una frecuencia fija
        self.timer = QTimer()
        self.timer.setInterval(1000 / 20)
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start()

        # Configuración de la interfaz gráfica
        vbox = QVBoxLayout()
        self.createGridLayout()
        vbox.addWidget(self.groupBox)
        
        # Botón ENVIAR
        button_send = QPushButton("ENVIAR", self)
        button_send.clicked.connect(self.callback_boton)
        button_send.resize(150, 50)
        vbox.addWidget(button_send)

        # Botón STOP
        button_stop = QPushButton("STOP ROBOT", self)
        button_stop.clicked.connect(self.callback_boton_stop)
        button_stop.resize(150, 50)
        vbox.addWidget(button_stop)

        self.setLayout(vbox)
        self.show()

    def callback_boton(self):
        try:
            desired_vx = float(self.edit1.text())
            desired_vtheta = float(self.edit2.text())
        except ValueError:
            desired_vx, desired_vtheta = 0.0, 0.0
        
        # Chequeo de límites
        if abs(desired_vx) > self.vx_max:
            desired_vx = np.sign(desired_vx) * self.vx_max
            self.node.get_logger().warn(f'... Setting desired_vx to vx_max = {desired_vx:.4f}')
            self.edit1.setText(str(desired_vx))
        if abs(desired_vtheta) > self.vtheta_max:
            desired_vtheta = np.sign(desired_vtheta) * self.vtheta_max
            self.node.get_logger().warn(f'... Setting desired_vtheta to vtheta_max = {desired_vtheta:.4f}')
            self.edit2.setText(str(desired_vtheta))
        
        self.desired_vx = desired_vx
        self.desired_vtheta = desired_vtheta
    
    def callback_boton_stop(self):
        self.desired_vx = 0.0
        self.desired_vtheta = 0.0
        self.node.get_logger().info('... Setting desired_vx and desired_vtheta to 0')
        self.edit1.setText("0.0")
        self.edit2.setText("0.0")
    
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.desired_vx
        msg.angular.z = self.desired_vtheta

        self.pub.publish(msg)

    def createGridLayout(self):
        gridLayout = QGridLayout()
        
        # Entrada para desired_vx
        label1 = QLabel("Velocidad lineal (m/s)", self)
        gridLayout.addWidget(label1, 0, 0)
        self.edit1 = QLineEdit(str(self.desired_vx))
        gridLayout.addWidget(self.edit1, 0, 1)
        
        # Entrada para desired_vtheta
        label2 = QLabel("Velocidad angular (rad/s)", self)
        gridLayout.addWidget(label2, 1, 0)
        self.edit2 = QLineEdit(str(self.desired_vtheta))
        gridLayout.addWidget(self.edit2, 1, 1)
        
        self.groupBox = QGroupBox("Comandos de velocidad")
        self.groupBox.setFont(QFont("Sanserif", 15))
        self.groupBox.setLayout(gridLayout)

if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    window = Ventana()
    myapp.exec_()
    sys.exit()
