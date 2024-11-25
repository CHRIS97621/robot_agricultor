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
            package='cap_robot',
            executable='imu1_node.py',
            name='imu_node',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0', 'baud': 9600}]
        ),
        Node(
            package='cap_robot',
            executable='gps1_node.py',
            name='gps_node',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB1', 'baud': 9600}]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_navsat',
            output='screen',
            parameters = [os.path.join(get_package_share_directory("cap_robot"), 'config', 'ekf_navsat.yaml')]
        ),
        
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='ekf_navsat',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("cap_robot"), 'config', 'ekf_navsat.yaml')],
           
        ),
        #Node(
            #package='rviz2',
            #executable='rviz2',
            #name='rviz2',
            #output='screen',
        #),
        Node(
            name='rqt_graph',
            package='rqt_graph', 
            executable='rqt_graph',
            output='screen'
        ),
    ])
