from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_and_imu_node = Node(
        package='cap_robot',
        executable='wit_ros2_imu2.py',  # Usar el nombre sin '.py' si está registrado en CMakeLists.txt
        name='imu',
        remappings=[('/wit/imu', '/imu/data')],
        parameters=[{'port': '/dev/ttyUSB0'},  # Cambié el puerto a '/dev/ttyUSB0'
                    {'baud': 9600}],
        output="screen"
    )

    
    rviz_display_node = Node(
        package='rviz2',
        executable="rviz2",
        output="screen"
    )

    return LaunchDescription(
        [
            rviz_and_imu_node,
            rviz_display_node  
        ]
    )
