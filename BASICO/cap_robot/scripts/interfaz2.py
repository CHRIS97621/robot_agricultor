#!/usr/bin/env python3
#---------------------------------------------------------------------------#
# Creating a Simple PySide2 Dialog Application for Control Commands
#---------------------------------------------------------------------------#

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import sys
import numpy as np
# 1.2. PYSIDE2 LIBRARIES
from PySide2.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton, 
                               QGroupBox, QGridLayout, QLineEdit, QLabel)
from PySide2.QtGui import QIcon, QFont
from PySide2.QtCore import QTimer
# 1.3. ROS2 LIBRARIES
import rclpy
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory


#---------------------------------------------------------------------------#
# 2. GLOBAL VARIABLES
#---------------------------------------------------------------------------#
# Get the file path of the ROS2 package
package_path = get_package_share_directory('cap_topicos')
print("package_path:", package_path)


#---------------------------------------------------------------------------#
# 3. CREATE OUR CLASS
#---------------------------------------------------------------------------#
class Ventana(QWidget):
    def __init__(self, callback=None):
        super().__init__()
        # Initial configuration
        self.setWindowTitle("My teleoperator")
        self.setGeometry(10, 20, 300, 200)

        # Configure ROS2
        rclpy.init()
        self.node = rclpy.create_node('teleop_qt')
        self.pub = self.node.create_publisher(Float32MultiArray, 'control_commands', 10)  # Cambiamos el tópico a control_commands
        
        # Declare and read parameters for maximum command limits
        self.node.declare_parameter('vx_max', 0.5)  # Máxima velocidad lineal
        self.node.declare_parameter('vtheta_max', 0.8)  # Máxima velocidad angular
        self.node.declare_parameter('cmd_freq', 20)  # frecuencia de comando
        
        self.vx_max = self.node.get_parameter('vx_max').value
        self.vtheta_max = self.node.get_parameter('vtheta_max').value
        self.cmd_freq = self.node.get_parameter('cmd_freq').value
        
        self.node.get_logger().info(f'Maximum linear speed (vx) = {self.vx_max}')
        self.node.get_logger().info(f'Maximum angular speed (vtheta) = {self.vtheta_max}')
        self.node.get_logger().info(f'frecuencia de comando(cmd_freq)= {self.cmd_freq}')
        
        # Set initial control commands
        self.vx = 0.0  # Velocidad lineal deseada
        self.vtheta = 0.0  # Velocidad angular deseada
        
        # Timer to publish commands at a fixed rate
        self.timer = QTimer()
        self.timer.setInterval(1000 / self.cmd_freq)  # 20 Hz de frecuencia de comando
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start()

        # Configure GUI layout
        vbox = QVBoxLayout()
        self.createGridLayout()
        vbox.addWidget(self.groupBox)
        
        # Add ENVIAR button
        button_send = QPushButton("ENVIAR", self)
        button_send.clicked.connect(self.callback_boton)
        button_send.setIcon(QIcon(package_path + "/resources" + "/robot.png"))
        button_send.resize(150, 50)
        vbox.addWidget(button_send)

        # Add STOP button
        button_stop = QPushButton("STOP ROBOT", self)
        button_stop.clicked.connect(self.callback_boton_stop)
        button_stop.setIcon(QIcon(package_path + "/resources" + "/robot.png"))
        button_stop.resize(150, 50)
        vbox.addWidget(button_stop)

        # Set layout and show window
        self.setLayout(vbox)
        self.show()

    # Callback for ENVIAR button
    def callback_boton(self):
        try:
            vx = float(self.edit1.text())  # Velocidad lineal deseada
            vtheta = float(self.edit2.text())  # Velocidad angular deseada
        except ValueError:
            vx, vtheta = 0.0, 0.0
        
        # Check limits
        if abs(vx) > self.vx_max:
            vx = np.sign(vx) * self.vx_max
            self.node.get_logger().warn(f'... Setting vx to vx_max = {vx:.4f}')
            self.edit1.setText(str(vx))
        if abs(vtheta) > self.vtheta_max:
            vtheta = np.sign(vtheta) * self.vtheta_max
            self.node.get_logger().warn(f'... Setting vtheta to vtheta_max = {vtheta:.4f}')
            self.edit2.setText(str(vtheta))
        
        # Save the values
        self.vx = vx
        self.vtheta = vtheta
    
    # Callback for STOP button
    def callback_boton_stop(self):
        # Set velocities to zero
        self.vx = 0.0
        self.vtheta = 0.0
        self.node.get_logger().info('... Setting vx and vtheta to 0')
        self.edit1.setText("0.0")
        self.edit2.setText("0.0")
    
    # Timer callback to publish control commands
    def timer_callback(self):
        # Create and publish message with control commands
        msg = Float32MultiArray()
        msg.data = [self.vx, self.vtheta]  # Enviar los valores de vx y vtheta deseados
        self.pub.publish(msg)

    # Create grid layout for the input fields
    def createGridLayout(self):
        gridLayout = QGridLayout()
        
        # Linear command input for vx
        label1 = QLabel("Velocidad lineal (vx) (m/s)", self)
        gridLayout.addWidget(label1, 0, 0)
        self.edit1 = QLineEdit(str(self.vx))
        gridLayout.addWidget(self.edit1, 0, 1)
        
        # Angular command input for vtheta
        label2 = QLabel("Velocidad angular (vtheta) (rad/s)", self)
        gridLayout.addWidget(label2, 1, 0)
        self.edit2 = QLineEdit(str(self.vtheta))
        gridLayout.addWidget(self.edit2, 1, 1)
        
        # Group box for layout
        self.groupBox = QGroupBox("Comandos de velocidad")
        self.groupBox.setFont(QFont("Sanserif", 15))
        self.groupBox.setLayout(gridLayout)


#---------------------------------------------------------------------------#
# 4. MAIN FUNCTION
#---------------------------------------------------------------------------#
if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    window = Ventana()
    myapp.exec_()
    sys.exit()
