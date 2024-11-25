#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
# 1.1. STANDARD
import sys
import threading
import time
# 1.2. ROS 2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile
from example_interfaces.msg import String


#---------------------------------------------------------------------------#
# 2. SUBSCRIBER NODE
#---------------------------------------------------------------------------#
class MinimalSubscriber(Node):
  def __init__(self, name):
    # Call base class constructor
    super().__init__(name)
    # Create a subscriber (msg_type, topic, callback, qos)
    self.sub_cb_group = MutuallyExclusiveCallbackGroup()
    self.subscription = self.create_subscription(
      String,
      'topic',
      callback = self.listener_callback, 
      qos_profile = QoSProfile(depth=10),
      callback_group = self.sub_cb_group)
    self.subscription  # prevent unused variable warning
    # Create timer
    self.timer_cb_group = MutuallyExclusiveCallbackGroup()
    self.timer = self.create_timer(2.0, self.timer_callback,
                                   callback_group = self.timer_cb_group)

  def listener_callback(self, msg):
    #self.get_logger().info('I heard: "%s"' % msg.data)
    self.get_logger().info('sub cb thread %s, msg: %s' %( threading.current_thread(), msg.data) )

  def timer_callback(self):
    # Create message
    self.get_logger().info('timer cb thread %s' %( threading.current_thread()) )
    # Do some work
    time.sleep(1.5)


#---------------------------------------------------------------------------#
# 2. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main(args=None):
  """
  Run a minimal_subscriber node standalone.
  """
  #------------------------------------------------------------------#
  # A. INITIALIZE ROS2
  #------------------------------------------------------------------#
  rclpy.init(args=args)
  print('---')
  print("Python MAIN thread %s:" % ( threading.current_thread()))
  print('---')

  #------------------------------------------------------------------#
  # B. CREATE A NODE AND EXECUTE IT USING A EXECUTOR
  #------------------------------------------------------------------#
  try:
    minimal_subscriber = MinimalSubscriber('sub')
    #executor = SingleThreadedExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(minimal_subscriber)
    # Execute callbacks for both nodes as they become ready
    try:
      executor.spin()
    finally:
      executor.shutdown()
      minimal_subscriber.destroy_node()
  except KeyboardInterrupt:
    pass
  except ExternalShutdownException:
    sys.exit(1)
  finally:
    rclpy.shutdown()


if __name__ == '__main__':
  # Runs the node when this script is run directly (not through an entrypoint)
  main()