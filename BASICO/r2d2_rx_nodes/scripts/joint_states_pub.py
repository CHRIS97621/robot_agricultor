#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. IMPORT LIBRARIES
#---------------------------------------------------------------------------#
#  1.1. STANDARD LIBRARIES
#  1.2. ROS LIBRARIES
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import JointState


#---------------------------------------------------------------------------#
# 3. OUR CLASS
#---------------------------------------------------------------------------#
class JointPublisher(Node):
  def __init__(self):
    super().__init__('odom_broadcaster')
    #-----------------------------------------------------------#
    # SET PARAMETERS
    #-----------------------------------------------------------#
    self.declare_parameter('topic_joint', '/joint_states')
    self.topic_joint  = self.get_parameter('topic_joint').value
    #-----------------------------------------------------------#
    # SUBSCRIBER AND PUBLISHER
    #-----------------------------------------------------------#
    self.joint_sub = self.create_subscription(
      JointState, self.topic_joint + '_gz', self.joint_callback, 10)
    self.joint_pub = self.create_publisher(JointState, self.topic_joint, 10)
    self.MESSAGES = 0
    #-----------------------------------------------------------#
    # CREATE TIMER TO PUBLISH JOINT
    #-----------------------------------------------------------#
    timer_period = 0.05  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def joint_callback(self, msg):
    self.joint_msg = msg
    self.MESSAGES = 1

  def timer_callback(self):
    if self.MESSAGES==1:
      self.joint_pub.publish(self.joint_msg)


#---------------------------------------------------------------------------#
# 4. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main(args=None):
  #-----------------------------------------------------------#
  # B. INITIALIZE ROS AND CREATE NODE
  #-----------------------------------------------------------#
  rclpy.init()
  node = JointPublisher()

  #-----------------------------------------------------------#
  # C. SPIN NODE
  #-----------------------------------------------------------#
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  rclpy.shutdown()

if __name__ == '__main__':
  main()