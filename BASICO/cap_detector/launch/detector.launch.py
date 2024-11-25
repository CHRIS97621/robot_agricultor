from launch import LaunchDescription
from launch_ros.actions import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor

def generate_launch_description():
    return LaunchDescription([
        # Nodo para la primera cámara
        Node(
            package='cap_detector',
            executable='detector_node5.py',
            name='cacao_detector1',
            output='screen',
            parameters=[
                {'camera_device': '/dev/video4'}
            ]
        ),
        # Nodo para la segunda cámara
        Node(
            package='cap_detector',
            executable='detector_node5.py',
            name='cacao_detector2',
            output='screen',
            parameters=[
                {'camera_device': '/dev/video2'}
            ]
        )
    ])

if __name__ == '__main__':
    rclpy.init()
    executor = MultiThreadedExecutor()
    ld = generate_launch_description()
    # Lanzar nodos con el executor multihilo
    for node in ld.entities:
        executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
