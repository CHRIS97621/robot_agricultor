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
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler


#---------------------------------------------------------------------------#
# 2. OUR CLASS
#---------------------------------------------------------------------------#
class FramePublisher(Node):
  """
  Broadcast dynamic transforms.
  """
  def __init__(self):
    super().__init__('dynamic_tf_broadcaster')
    #-----------------------------------------------------------#
    # A. READ PARAMETERS
    #-----------------------------------------------------------#
    self.declare_parameter('sensor_pos_x', 0.2)
    self.declare_parameter('sensor_pos_y', 0.0)
    self.sensor_pos_x = self.get_parameter('sensor_pos_x').value
    self.sensor_pos_y = self.get_parameter('sensor_pos_y').value
    self.get_logger().info("sensor_pos_x: %.4f, sensor_pos_y %.4f"\
      %(self.sensor_pos_x, self.sensor_pos_y))
    #-----------------------------------------------------------#
    # B. CREATE TRANSFORM BROADCASTERS
    #-----------------------------------------------------------#
    self.tf_broadcaster        = TransformBroadcaster(self)
    self.tf_static_broadcaster = StaticTransformBroadcaster(self)
    #-----------------------------------------------------------#
    # C. CREATE "TransformStamped" OBJECTS
    #-----------------------------------------------------------#
    # FOR THE STATIC FRAME
    self.t1 = TransformStamped()
    self.t1.header.frame_id = 'base_link'
    self.t1.child_frame_id  = 'sensor'
    # FOR THE DYNAMIC FRAME
    self.t2 = TransformStamped()
    self.t2.header.frame_id = 'map'
    self.t2.child_frame_id  = 'base_link'
    #-----------------------------------------------------------#
    # D. PUBLISH STATIC FRAME
    #-----------------------------------------------------------#
    self.publish_static_tf()
    #-----------------------------------------------------------#
    # E. CREATE TIMER TO PUBLISH DYNAMIC FRAME
    #-----------------------------------------------------------#
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    #-----------------------------------------------------------#
    # F. SAVE INITIAL TIME
    #  https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/time.py
    #  https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/duration.py
    #-----------------------------------------------------------#
    self.tt = self.get_clock().now() # <class 'rclpy.time.Time'>


  def publish_static_tf(self):
    #-----------------------------------------------------------#
    # SET THE STATIC TRANSFORM
    #-----------------------------------------------------------#
    t = self.t1
    #  SET "timestamp"
    t.header.stamp = self.get_clock().now().to_msg()
    #  SET POSITION OF CHILD FRAME
    t.transform.translation.x = self.sensor_pos_x
    t.transform.translation.y = self.sensor_pos_y
    t.transform.translation.z = 0.0
    #  SET ROTATION OF CHILD FRAME
    quat = quaternion_from_euler(0.0, 0.0, 0.0)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    #-----------------------------------------------------------#
    # SEND THE TRANSFORM USING THE "broadcaster"
    #-----------------------------------------------------------#
    self.tf_static_broadcaster.sendTransform(t)
    self.get_logger().info("Static transform published")


  def timer_callback(self):
    #----------------------------------------------------#
    # COMPUTE ELAPSED TIME SINCE THE BEGINNING
    #----------------------------------------------------#
    te = self.get_clock().now() - self.tt
    #print(type(te))     # <class 'rclpy.duration.Duration'>
    tsec = float(te.nanoseconds/1e9)
    x = (math.pi/4.0)*tsec  # "x=(pi/4)*t"
    #----------------------------------------------------#
    # SET THE DYNAMIC FRAME
    #----------------------------------------------------#
    t = self.t2
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
  print('sys.argv:',sys.argv)
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