#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. IMPORT LIBRARIES
#---------------------------------------------------------------------------#
#  1.1. STANDARD LIBRARIES
import sys          # to get commandline arguments
import math
import numpy as np
#  1.2. ROS LIBRARIES
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
#from tf_transformations import quaternion_from_euler, euler_from_quaternion


#---------------------------------------------------------------------------#
# 2. AUXILIAR FUNCTION
#---------------------------------------------------------------------------#
def quaternion_from_euler(ai, aj, ak):
  ai /= 2.0
  aj /= 2.0
  ak /= 2.0
  ci = math.cos(ai)
  si = math.sin(ai)
  cj = math.cos(aj)
  sj = math.sin(aj)
  ck = math.cos(ak)
  sk = math.sin(ak)
  cc = ci*ck
  cs = ci*sk
  sc = si*ck
  ss = si*sk

  q = np.empty((4, ))
  q[0] = cj*sc - sj*cs
  q[1] = cj*ss + sj*cc
  q[2] = cj*cs - sj*sc
  q[3] = cj*cc + sj*ss

  return q


#---------------------------------------------------------------------------#
# 3. OUR CLASS
#---------------------------------------------------------------------------#
class StaticFramePublisher(Node):
  def __init__(self):
    super().__init__('static_broadcaster')
    # Declare parameters
    self.declare_parameter('base_frame', 'laser_link')
    self.declare_parameter('child_frame', 'sensor')
    self.declare_parameter('x', 0.0)
    self.declare_parameter('y', 0.0)
    self.declare_parameter('yaw', 0.0)
    self.base_frame  = self.get_parameter('base_frame').value
    self.child_frame = self.get_parameter('child_frame').value
    self.x   = self.get_parameter('x').value
    self.y   = self.get_parameter('y').value
    self.yaw = self.get_parameter('yaw').value
    # Create a "StaticTransformBroadcaster" object
    self.tf_static_broadcaster = StaticTransformBroadcaster(self)
    # Publish static transforms once at startup
    self.make_transforms()


  def make_transforms(self):
    #-----------------------------------------------------------#
    # GET ORIENTATION
    #-----------------------------------------------------------#
    quat = quaternion_from_euler(0,0,self.yaw)
    #-----------------------------------------------------------#
    # CREATE A "TransformStamped" OBJECT WHICH WILL BE THE
    # MESSAGE WE WILL SEND OVER
    #-----------------------------------------------------------#
    # CREATE A "TransformStamped" OBJECT
    t = TransformStamped()
    # SET "header" INFORMATION
    t.header.stamp    = self.get_clock().now().to_msg()
    t.header.frame_id = self.base_frame
    # SET "frame_id" OF THE CHILD FRAME
    t.child_frame_id = self.child_frame
    #  SET "transform" INFORMATION
    #   -> Set position of the child frame
    t.transform.translation.x = 0.0 + self.x
    t.transform.translation.y = 0.0 + self.y
    t.transform.translation.z = 0.0
    #   -> Set rotation of the child frame (First get quaternion)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    #-----------------------------------------------------------#
    # SEND THE TRANSFORM USING THE "broadcaster"
    #-----------------------------------------------------------#
    self.tf_static_broadcaster.sendTransform(t)
    self.get_logger().info("Spinning until killed publishing %s to %s"\
      %(self.base_frame, self.child_frame) )


#---------------------------------------------------------------------------#
# 4. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main(args=None):
  #-----------------------------------------------------------#
  # B. INITIALIZE ROS AND CREATE NODE
  #-----------------------------------------------------------#
  rclpy.init()
  node = StaticFramePublisher()

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