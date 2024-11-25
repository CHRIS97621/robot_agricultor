#!/usr/bin/env python3
#---------------------------------------------------------------------------#
# Creating a Simple PySide2 Dialog Application
#	https://codeloop.org/python-gui-gridlayout-with-pyside2-qt-for-python/
#---------------------------------------------------------------------------#

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
#  1.1. PYTHON LIBRARIES
import sys
import numpy as np
#  1.2. PYSIDE2 LIBRARIES
from PySide2.QtWidgets import (QApplication, QWidget,
    QVBoxLayout, QPushButton, QGroupBox, QGridLayout, QLineEdit, QLabel)
from PySide2.QtGui import QIcon, QFont
from PySide2.QtCore import QTimer
#  1.3. ROS LIBRARIES
import rclpy
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory


#---------------------------------------------------------------------------#
# 2. GLOBAL VARIABLES
#---------------------------------------------------------------------------#
#  GET FILE PATH OF OUR ROS2 PACKAGE
package_path = get_package_share_directory('cap_topicos')
print("package_path:", package_path)


#---------------------------------------------------------------------------#
# 3. CREATE OUR CLASS
#---------------------------------------------------------------------------#
class Ventana(QWidget):
  #------------------------------------------------------------------#
  # CONSTRUCTOR
  #------------------------------------------------------------------#
  def __init__(self, callback=None):
    #------------------------------------------------------------#
    # INITIAL CONFIGURATION
    #------------------------------------------------------------#
    super().__init__()
    self.setWindowTitle("My teleoperator")
    self.setGeometry(10,20,300,200)

    #------------------------------------------------------------#
    # CONFIGURE ROS2
    #------------------------------------------------------------#
    rclpy.init()
    self.node = rclpy.create_node('teleop_qt')
    self.pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
    # Declare and parameters
    self.node.declare_parameter('v_max', 0.5)
    self.node.declare_parameter('w_max', 0.8)
    self.node.declare_parameter('cmd_freq', 20)
    # Read parameters
    self.v_max = self.node.get_parameter('v_max').value
    self.w_max = self.node.get_parameter('w_max').value
    self.cmd_freq = self.node.get_parameter('cmd_freq').value
    self.node.get_logger().info('cmd_freq = %d' %(self.cmd_freq))
    self.node.get_logger().info('Maximum velocities (v,w) = (%.3f, %.3f)' 
        %(self.v_max, self.w_max))
    # CREATE TIMER TO PUBLISH
    # No se puede usar el timer de ROS2 ya que no estamos usando el spin()
    # https://www.pythonguis.com/tutorials/multithreading-pyside-applications-qthreadpool/
    self.timer = QTimer()
    self.timer.setInterval(1000/self.cmd_freq)   # ms
    self.timer.timeout.connect(self.timer_callback)
    self.timer.start()
    # SET INITIAL VELOCITIES
    self.v = 0.0
    self.w = 0.0

    #------------------------------------------------------------#
    # CONFIGURE GUI
    #------------------------------------------------------------#
    # CREATE VERTICAL LAYOUT
    vbox = QVBoxLayout()
    # Add grid layout
    self.createGridLayout()
    vbox.addWidget(self.groupBox)
    # Add button
    button = QPushButton("ENVIAR", self)
    button.clicked.connect(self.callback_boton)
    button.setIcon(QIcon(package_path + "/resources" + "/robot.png"))
    button.resize(150, 50) 
    vbox.addWidget(button)
    # Add STOP button
    button = QPushButton("STOP ROBOT", self)
    button.clicked.connect(self.callback_boton_stop)
    button.setIcon(QIcon(package_path + "/resources" + "/robot.png"))
    button.resize(150, 50) 
    vbox.addWidget(button)
    # SET AND SHOW LAYOUT OF DIALOG
    self.setLayout(vbox)
    self.show()

  #------------------------------------------------------------------#
  # QT CALLBACKS
  #------------------------------------------------------------------#
  def callback_boton(self):
    # READ VELOCITIES
    v = float(self.edit1.text())
    w = float(self.edit2.text())
    # CHECK LIMITS
    print('---') 
    if(abs(v)>self.v_max):
      v = np.sign(v)*self.v_max
      self.node.get_logger().warn('... Setting v to v_max=%.4f' % (v))
      # Reset text
      self.edit1.setText( str(v) )
    if(abs(w)>self.w_max):
      w = np.sign(w)*self.w_max
      self.node.get_logger().warn('... Setting w to w_max=%.4f' % (w))
      # Reset text
      self.edit2.setText( str(w) )
    # SAVE
    self.v = v
    self.w = w
  
  def callback_boton_stop(self):
    # READ VELOCITIES
    v = 0.0
    w = 0.0
    self.v = v
    self.w = w
    # RESET EDIT TEXT BOXES
    self.node.get_logger().info('... Setting v and w to 0')
    self.edit1.setText( str(v) )
    self.edit2.setText( str(w) )
    
  def timer_callback(self):
    # SEND DATA
    msg = Twist()
    msg.linear.x  = self.v 
    msg.angular.z = self.w 
    self.pub.publish(msg)


  #------------------------------------------------------------------#
  # GRID LAYOUT
  #------------------------------------------------------------------#
  def createGridLayout(self):
    # CREATE GRID LAYOUT
    gridLayout = QGridLayout()
    # First row
    label1 = QLabel("Ingresa v(m/s)", self)
    gridLayout.addWidget(label1, 0,0)
    self.edit1 = QLineEdit( str(self.v) )   # Save edit1 resource
    gridLayout.addWidget(self.edit1, 0,1)
    # Second row
    label2= QLabel("Ingresa w(rad/s)", self)
    gridLayout.addWidget(label2, 1,0)
    self.edit2 = QLineEdit( str(self.w) )   # Save edit1 resource
    gridLayout.addWidget(self.edit2, 1,1)
    # SET GROUP
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