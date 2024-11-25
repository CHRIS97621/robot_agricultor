#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Paths for robot_localization config files
    ekf_config_path = os.path.join(get_package_share_directory('cap_robot'), 'config', 'ekf_gps_imu_odom.yaml')
    navsat_config_path = os.path.join(get_package_share_directory('cap_robot'), 'config', 'gps_navsat.yaml')

    return LaunchDescription([
        # Node for odometry
        Node(
            package='cap_robot',
            executable='odom2_node.py',
            name='odom_node',
            output='screen'
        ),
        
        # Node for GUI teleoperator
        Node(
            package='cap_robot',
            executable='interfaz1.py',
            name='teleoperator_gui',
            output='screen'
        ),
        
        # Node for robot_localization EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            output='screen',
            parameters=[ekf_config_path]
        ),
        
        # Node for navsat_transform for GPS data transformation
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='gps_navsat_transform',
            output='screen',
            parameters=[navsat_config_path],
            remappings=[
                ('/gps/fix', '/gps/fix'),
                ('/imu/data', '/imu/data'),
                ('/odom', '/odom')
            ]
        )
    ])
