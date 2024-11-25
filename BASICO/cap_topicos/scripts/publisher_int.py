#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from example_interfaces.msg import Int32, Float32


#---------------------------------------------------------------------------#
# 2. PUBLISHER NODE
#---------------------------------------------------------------------------#
class MinimalPublisher(Node):
  def __init__(self):
    # Call base class constructor
    super().__init__('pub')
    # Create a publisher (msg_type, topic, )
    qos = QoSProfile(depth=10)
    self.publisher_ = self.create_publisher(msg_type=Int32, 
                                            topic='topic',
                                            qos_profile=qos)
    # Create a timer
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    # Counter
    self.i = 0

  def timer_callback(self):
    # Create message
    msg = Int32()
    msg.data = self.i
    # Publish message
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%d"' % msg.data)
    # Update counter
    self.i += 1


#---------------------------------------------------------------------------#
# 3. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main(args=None):
  """
  Run a minimal_publisher node standalone.
  """
  # INIT ROS2 AND CREATE A NODE
  rclpy.init(args=args)
  minimal_publisher = MinimalPublisher()
  # B. SPIN
  rclpy.spin(minimal_publisher)
  # C. CLOSE ALL
  minimal_publisher.destroy_node() # Not necessary
  rclpy.shutdown()

if __name__ == '__main__':
  # Runs the node when this script is run directly (not through an entrypoint)
  main()