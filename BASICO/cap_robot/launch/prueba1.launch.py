#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de odometría
        Node(
            package='cap_robot',
            executable='odom5_node.py',
            name='odom_node',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'baud_rate': 115200}]
        ),
        
        # Nodo de la interfaz gráfica
        Node(
            package='cap_robot',
            executable='interfaz3.py',
            name='interfaz1',
            output='screen'
        ),
    ])
