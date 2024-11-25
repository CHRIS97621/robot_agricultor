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
from example_interfaces.msg import String


#---------------------------------------------------------------------------#
# 2. PUBLISHER NODE
#---------------------------------------------------------------------------#
class MinimalPublisher(Node):
  def __init__(self, name):
    # Call base class constructor
    super().__init__(name)
    # Create publisher (msg_type, topic, )
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    # Create a timer
    # This type of callback group only allows one callback to be executed at a time
    self.timer1_cb_group = MutuallyExclusiveCallbackGroup()
    timer_period = 1.0  # seconds
    self.timer = self.create_timer(timer_period, 
                                   self.timer1_callback,
                                   callback_group=self.timer1_cb_group)
    self.i = 1
    # Another timer
    self.timer2_cb_group = MutuallyExclusiveCallbackGroup()
    self.timer2 = self.create_timer(timer_period_sec=2.0,
                                    callback=self.timer2_callback,
                                    callback_group=self.timer2_cb_group)

  def timer1_callback(self):
    # Create message
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    # Publish message
    self.publisher_.publish(msg)
    self.get_logger().info('timer1 cb thread %s, data:%d' %( threading.current_thread(), self.i) )
    # Update counter
    self.i += 1

  def timer2_callback(self):
    # Create message
    self.get_logger().info('timer2 cb thread %s' %( threading.current_thread()) )
    # Simulate some work
    time.sleep(1.5)


#---------------------------------------------------------------------------#
# 3. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main(args=None):
  """
  Run a minimal_publisher node standalone.
  """
  #------------------------------------------------------------------#
  # A. INITIALIZE ROS
  #------------------------------------------------------------------#
  rclpy.init(args=args)
  print('---')
  print("Python MAIN thread %s:" % ( threading.current_thread()))
  print('---')

  #------------------------------------------------------------------#
  # B. CREATE A NODE AND EXECUTE IT USING A EXECUTOR
  #------------------------------------------------------------------#
  try:
    minimal_publisher = MinimalPublisher('pub')
    #executor = SingleThreadedExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(minimal_publisher)
    # Execute callbacks for both nodes as they become ready
    try:
      executor.spin()
    finally:
      executor.shutdown()
      minimal_publisher.destroy_node()
  except KeyboardInterrupt:
      pass
  except ExternalShutdownException:
      sys.exit(1)
  finally:
    rclpy.shutdown()


if __name__ == '__main__':
  # Runs the node when this script is run directly (not through an entrypoint)
  main()