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
from sensor_msgs.msg import JointState


#---------------------------------------------------------------------------#
# 2. OUR CLASS
# https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html
#---------------------------------------------------------------------------#
class OdomPublisher(Node):
  def __init__(self):
    super().__init__('odom_broadcaster')
    # Declare parameters
    self.declare_parameter('topic_odom',  '/r1_odom')
    self.declare_parameter('base_frame',  '/r1_odom_tf')
    self.declare_parameter('child_frame', '/r1_base_link')
    # Set parameter
    self.topic_odom   = self.get_parameter('topic_odom').value
    self.base_frame   = self.get_parameter('base_frame').value
    self.child_frame  = self.get_parameter('child_frame').value
    # Create a "TransformBroadcaster" object
    self.tf_broadcaster = TransformBroadcaster(self)
    # Create a subscriber (msg_type, topic, callback, )
    self.odom_sub = self.create_subscription(
      Odometry, self.topic_odom, self.odom_callback, 10)
    self.odom_sub  # prevent unused variable warning

  def odom_callback(self, msg):
    #------------------------------------------------------------------#
    # POSE INFO
    #------------------------------------------------------------------#
    position    = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    #-----------------------------------------------------------#
    # CREATE A "TransformStamped" OBJECT WHICH WILL BE THE
    # MESSAGE WE WILL SEND OVER
    # https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html
    #-----------------------------------------------------------#
    ts = TransformStamped()
    # SET "header" INFORMATION
    ts.header.stamp    = self.get_clock().now().to_msg()
    ts.header.frame_id = self.base_frame
    # SET "frame_id" OF THE CHILD FRAME
    ts.child_frame_id = self.child_frame
    #  SET "transform" INFORMATION
    #   -> Set position of the child frame
    ts.transform.translation.x = position.x
    ts.transform.translation.y = position.y
    ts.transform.translation.z = position.z
    #   -> Set rotation of the child frame
    ts.transform.rotation.x = orientation.x
    ts.transform.rotation.y = orientation.y
    ts.transform.rotation.z = orientation.z
    ts.transform.rotation.w = orientation.w
    #----------------------------------------------------#
    # SEND THE TRANSFORM USING THE "broadcaster"
    #----------------------------------------------------#
    self.tf_broadcaster.sendTransform(ts)


#---------------------------------------------------------------------------#
# 3. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main(args=None):
  #-----------------------------------------------------------#
  # A. INITIALIZE ROS AND CREATE NODE
  #-----------------------------------------------------------#
  rclpy.init()
  node = OdomPublisher()
  #-----------------------------------------------------------#
  # B. SPIN NODE
  #-----------------------------------------------------------#
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  rclpy.shutdown()

if __name__ == '__main__':
  main()