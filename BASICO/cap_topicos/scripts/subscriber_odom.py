#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
# 1.1. STANDARD
import sys
# 1.2. ROS 2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


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
      Odometry,
      'odometry',
      callback = self.odometry_callback, 
      qos_profile = QoSProfile(depth=10),
      callback_group = self.sub_cb_group)
    self.subscription  # prevent unused variable warning
    # Create timer
    self.timer_cb_group = MutuallyExclusiveCallbackGroup()
    self.timer = self.create_timer(1.0, self.timer_callback,
                                   callback_group = self.timer_cb_group)
    # Odometry inicial value
    self.x = 0
    self.y = 0
    self.yaw = 0

  def odometry_callback(self, msg):
    #------------------------------------------------------------#
    # GET POSE 
    #------------------------------------------------------------#
    pose = msg.pose.pose
    # Get position
    x = pose.position.x
    y = pose.position.y
    # Get Euler angles
    quaternion = (pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    roll  = euler[0]
    pitch = euler[1]
    yaw   = euler[2]

    #------------------------------------------------------------------#
    # SAVE DATA
    #------------------------------------------------------------------#
    self.x = x
    self.y = y
    self.yaw = yaw
    return

  def timer_callback(self):
    # Create message
    self.get_logger().info('x:%f, y:%f, yaw:%f' %(self.x, self.y, self.yaw))


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