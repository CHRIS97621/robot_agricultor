#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. IMPORT LIBRARIES
#---------------------------------------------------------------------------#
#  1.1. STANDARD LIBRARIES
import sys
import math
#  1.2. ROS LIBRARIES
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


#---------------------------------------------------------------------------#
# 2. OUR CLASS
#---------------------------------------------------------------------------#
class FramePublisher(Node):
  def __init__(self):
    super().__init__('dynamic_br')
    #-----------------------------------------------------------#
    # A. READ PARAMETERS
    #-----------------------------------------------------------#
    self.declare_parameter('parent', 'world')
    self.declare_parameter('child', 'base_link')
    self.parent = self.get_parameter('parent').value
    self.child  = self.get_parameter('child').value
    #-----------------------------------------------------------#
    # B. CREATE TRANSFORM BROADCASTERS
    #-----------------------------------------------------------#
    self.tf_broadcaster = TransformBroadcaster(self)
    self.t = TransformStamped()
    self.t.header.frame_id = self.parent
    self.t.child_frame_id  = self.child
    #-----------------------------------------------------------#
    # C. CREATE TIMER TO PUBLISH DYNAMIC FRAME
    #-----------------------------------------------------------#
    timer_period = 0.005  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    #-----------------------------------------------------------#
    # D. SAVE INITIAL TIME
    #  https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/time.py
    #  https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/duration.py
    #-----------------------------------------------------------#
    self.init_time = self.get_clock().now() # <class 'rclpy.time.Time'>

  def timer_callback(self):
    #----------------------------------------------------#
    # COMPUTE ELAPSED TIME SINCE THE BEGINNING
    #----------------------------------------------------#
    te = self.get_clock().now() - self.init_time
    tsec = float(te.nanoseconds/1e9)
    x = (math.pi/4.0)*tsec  # "x=(pi/4)*t"
    #----------------------------------------------------#
    # SET THE DYNAMIC FRAME
    #----------------------------------------------------#
    t = self.t
    #  SET "timestamp"
    t.header.stamp = self.get_clock().now().to_msg()
    #  SET POSITION OF CHILD FRAME
    t.transform.translation.x = 2*math.cos(x)
    t.transform.translation.y = 2*math.sin(x)
    t.transform.translation.z = 0.0
    #  SET ROTATION OF CHILD FRAME
    yaw = x
    quat = quaternion_from_euler(0,0,yaw)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    #----------------------------------------------------#
    # SEND THE TRANSFORM USING THE "broadcaster"
    #----------------------------------------------------#
    self.tf_broadcaster.sendTransform(t)
    

#---------------------------------------------------------------------------#
# 3. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main():
  #-----------------------------------------------------------#
  # INITIALIZE ROS AND CREATE NODE
  #-----------------------------------------------------------#
  rclpy.init()
  node = FramePublisher()

  #-----------------------------------------------------------#
  # SPIN NODE
  #-----------------------------------------------------------#
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  rclpy.shutdown()

if __name__ == '__main__':
  main()