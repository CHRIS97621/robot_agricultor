#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point  # PointStamped 
from tf2_geometry_msgs import PointStamped


#---------------------------------------------------------------------------#
# 2. PUBLISHER NODE
#---------------------------------------------------------------------------#
class PointPublisher(Node):
  def __init__(self):
    # Call base class constructor
    super().__init__('pub_point')
    # Create a publisher (msg_type, topic, )
    qos = QoSProfile(depth=10)
    self.publisher_ = self.create_publisher(msg_type=PointStamped, 
                                            topic='point_stamped',
                                            qos_profile=qos)
    # Create a timer
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):
    #  CREATE A "PointStamped" MESSAGE
    # https://github.com/ros2/common_interfaces/blob/jazzy/geometry_msgs/msg/PointStamped.msg
    msg = PointStamped()
    msg.header.frame_id = 'sensor'
    msg.header.stamp    = self.get_clock().now().to_msg()
    msg.point = Point()
    msg.point.x = 0.4
    msg.point.y = 0.2
    msg.point.z = 0.0
    # Publish message
    self.publisher_.publish(msg)


#---------------------------------------------------------------------------#
# 3. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main(args=None):
  """
  Run a minimal_publisher node standalone.
  """
  # INIT ROS2 AND CREATE A NODE
  rclpy.init(args=args)
  minimal_publisher = PointPublisher()
  # B. SPIN
  rclpy.spin(minimal_publisher)
  # C. CLOSE ALL
  minimal_publisher.destroy_node() # Not necessary
  rclpy.shutdown()

if __name__ == '__main__':
  main()