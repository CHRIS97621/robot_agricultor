#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_ros2_prueba',
            executable='imu_node5.py',
            name='imu_node',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB1', 'baud': 9600}]
        ),
        Node(
            package='gps_ros2_pruebas',
            executable='gps_node7r.py',
            name='gps_node',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0', 'baud': 9600}]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_imu_gps2',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("cap_launch"), 'config', 'ekf_imu_gps2.yaml')]
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_gps',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("cap_launch"), 'config', 'navsat_gps.yaml')]
        ),
        Node(
            name='rqt_graph',
            package='rqt_graph', 
            executable='rqt_graph',
            output='screen'
        ),
        #Node(
            #package='rviz2',
            #executable='rviz2',
            #name='rviz2',
            #output='screen',
            #arguments=['-d', os.path.join(get_package_share_directory("cap_launch"), 'config', 'robot_visualization.rviz')]
        #),
    ])
