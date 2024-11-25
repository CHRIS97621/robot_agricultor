#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import random

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odometry/simulated', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Odom Publisher Node has been started.")
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def timer_callback(self):
        msg = Odometry()
        
        # Simulación de la odometría
        self.x += random.uniform(-0.1, 0.1)  # Simula movimiento en x
        self.y += random.uniform(-0.1, 0.1)  # Simula movimiento en y
        self.theta += random.uniform(-0.01, 0.01)  # Simula rotación
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Posición
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # Velocidad
        msg.twist.twist.linear.x = random.uniform(-0.1, 0.1)
        msg.twist.twist.linear.y = random.uniform(-0.1, 0.1)
        msg.twist.twist.angular.z = random.uniform(-0.01, 0.01)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: x={self.x}, y={self.y}, theta={self.theta}')

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
