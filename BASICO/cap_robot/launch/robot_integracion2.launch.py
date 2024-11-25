#---------------------------------------------------------------------------#
# 1. LIBRERÍAS
#---------------------------------------------------------------------------#
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

#from ament_index_python.packages import get_package_share_directory
#from launch import LaunchDescription
#from launch_ros.actions import Node
#import os

def generate_launch_description():
    #---------------------------------------------------------------#
	# CREATE LAUNCH CONFIGURATION VARIABLES
	#---------------------------------------------------------------#
    # Ruta de archivo rviz
    rviz_config_dir = os.path.join(get_package_share_directory('cap_robot'), 'rviz', 'robot1.rviz')

    # Ruta a los archivos de configuración YAML para EKF y navsat_transform
    ekf_config_path = os.path.join(get_package_share_directory("cap_robot"), 'config', 'ekf_gps_imu.yaml')
    navsat_config_path = os.path.join(get_package_share_directory("cap_robot"), 'config', 'gps_navsat.yaml')

    return LaunchDescription([
        # Nodo IMU
        Node(
            package='cap_robot',
            executable='imu2_node.py',
            name='imu_node',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0', 'baud': 9600}]
        ),
        # Nodo GPS
        Node(
            package='cap_robot',
            executable='gps3_node.py',
            name='gps_node',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB1', 'baud': 9600}]
        ),
        # Nodo EKF para fusionar IMU y GPS
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_gps_imu',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[
                ('/imu/data', '/imu/data'),
                ('/gps/fix', '/gps/fix'),
                ('odometry/filtered', 'odometry/global')  # Solo usamos la salida de odometría filtrada global
            ]
        ),
        # Nodo navsat_transform para transformar los datos GPS
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='gps_navsat',
            output='screen',
            parameters=[navsat_config_path],
            remappings=[
                ('imu', '/imu/data'),       # Remapea el tópico de IMU
                ('gps/fix', '/gps/fix'),    # Remapea el tópico de GPS
                ('gps/filtered', 'gps/filtered'),
                ('odometry/filtered', 'odometry/global')  # Publicación de la odometría global filtrada
            ]
        ),
        # Nodo rqt_graph para visualizar el grafo de nodos
        Node(
            package='rqt_graph', 
            executable='rqt_graph',
            name='rqt_graph',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir]
        ),

    ])
