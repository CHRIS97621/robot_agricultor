#!/usr/bin/env python3
#---------------------------------------------------------------------------#
# Creating a Simple PySide2 Dialog Application
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
package_path = get_package_share_directory('cap_robot')
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
        self.pub = self.node.create_publisher(Float32MultiArray, 'set_speed', 10)
        
        # Declare and read parameters
        self.node.declare_parameter('v_max', 0.5)
        self.node.declare_parameter('w_max', 0.8)
        self.node.declare_parameter('cmd_freq', 20)
        
        self.v_max = self.node.get_parameter('v_max').value
        self.w_max = self.node.get_parameter('w_max').value
        self.cmd_freq = self.node.get_parameter('cmd_freq').value
        
        self.node.get_logger().info(f'cmd_freq = {self.cmd_freq}')
        self.node.get_logger().info(f'Maximum velocities (v, w) = ({self.v_max}, {self.w_max})')
        
        # Set initial velocities
        self.v = 0.0
        self.w = 0.0
        
        # Timer to publish at a fixed rate
        self.timer = QTimer()
        self.timer.setInterval(1000 / self.cmd_freq)
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
            v = float(self.edit1.text())
            w = float(self.edit2.text())
        except ValueError:
            v, w = 0.0, 0.0
        
        # Check limits
        if abs(v) > self.v_max:
            v = np.sign(v) * self.v_max
            self.node.get_logger().warn(f'... Setting v to v_max = {v:.4f}')
            self.edit1.setText(str(v))
        if abs(w) > self.w_max:
            w = np.sign(w) * self.w_max
            self.node.get_logger().warn(f'... Setting w to w_max = {w:.4f}')
            self.edit2.setText(str(w))
        
        # Save the values
        self.v = v
        self.w = w
    
    # Callback for STOP button
    def callback_boton_stop(self):
        self.v = 0.0
        self.w = 0.0
        self.node.get_logger().info('... Setting v and w to 0')
        self.edit1.setText("0.0")
        self.edit2.setText("0.0")
    
    # Timer callback to publish velocities
    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [self.v, self.w]
        self.pub.publish(msg)

    # Create grid layout for the input fields
    def createGridLayout(self):
        gridLayout = QGridLayout()
        
        # Velocity input
        label1 = QLabel("Ingresa v(m/s)", self)
        gridLayout.addWidget(label1, 0, 0)
        self.edit1 = QLineEdit(str(self.v))
        gridLayout.addWidget(self.edit1, 0, 1)
        
        # Angular velocity input
        label2 = QLabel("Ingresa w(rad/s)", self)
        gridLayout.addWidget(label2, 1, 0)
        self.edit2 = QLineEdit(str(self.w))
        gridLayout.addWidget(self.edit2, 1, 1)
        
        # Group box for layout
        self.groupBox = QGroupBox("Velocidades")
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
