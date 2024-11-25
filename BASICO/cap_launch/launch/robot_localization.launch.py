#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable
import os
#import yaml
#import pathlib

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_ros2_prueba',
            executable='imu_node5.py',
            name='imu_node',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0', 'baud': 9600}]
        ),
        Node(
            package='gps_ros2_pruebas',
            executable='gps_node7r.py',
            name='gps_node',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB1', 'baud': 9600}]
        ),
        #Node(
            #package='imu_ros2_prueba',
            #executable='odom_node.py',  # Nodo de odometría simulada
            #name='odometry_node',
            #output='screen'
        #),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters = [os.path.join(get_package_share_directory("cap_launch"), 'config', 'ekf_filter_node.yaml')]
        ),
        Node(
            name='rqt_graph',
            package='rqt_graph', 
            executable='rqt_graph',
            output='screen'
        ),
    ])
