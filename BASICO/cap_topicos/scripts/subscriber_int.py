#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32


#---------------------------------------------------------------------------#
# 2. SUBSCRIBER NODE
#---------------------------------------------------------------------------#
class MinimalSubscriber(Node):
  def __init__(self):
    # Call base class constructor
    super().__init__('sub')
    # Create subscriber (msg_type, topic, callback, )
    self.subscription = self.create_subscription(msg_type=Int32,
                                                 topic='topic',
                                                 callback=self.listener_callback,
                                                 qos_profile=10)
    self.subscription  # prevent unused variable warning

  def listener_callback(self, msg):
    self.get_logger().info('I heard: "%d"' % msg.data)


#---------------------------------------------------------------------------#
# 3. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main(args=None):
  """
  Run a minimal_subscriber node standalone.
  """
  # INIT ROS2 AND CREATE A NODE
  rclpy.init(args=args)
  minimal_subscriber = MinimalSubscriber()
  # B. SPIN
  rclpy.spin(minimal_subscriber)
  # C. CLOSE ALL
  minimal_subscriber.destroy_node() # Not necessary
  rclpy.shutdown()

if __name__ == '__main__':
  main()