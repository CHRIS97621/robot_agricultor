#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. IMPORT LIBRARIES
#---------------------------------------------------------------------------#
#  1.1. STANDARD LIBRARIES
import numpy as np
import time
#  1.2. ROS LIBRARIES
import rclpy
import rclpy.duration
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point  # PointStamped 
from tf2_geometry_msgs import PointStamped
from tf_transformations import quaternion_matrix


#---------------------------------------------------------------------------#
# 2. AUXILIARY FUNCTIONS
#---------------------------------------------------------------------------#
def convert_point(trans, p):
  #-----------------------------------------------------------#
  # A. INITIAL CONFIGURATION
  #-----------------------------------------------------------#
  # CHECK IF POINT IS ALREADY IN THE FRAME
  if trans.header.frame_id == p.header.frame_id:
    print("convert_point: Point is already in the desired frame")
    return p, np.identity(4)
  # CHECK IF CHILD FRAME IS CORRECT
  if trans.child_frame_id != p.header.frame_id:
    print("convert_point: Child frame is incorrect for the given transform")
    return None, np.identity(4)
  #-----------------------------------------------------------#
  # B. COMPUTE HOMOGENEOUS TRANSFORMATION
  #-----------------------------------------------------------#
  # GET ROTATION MATRIX
  quat = [trans.transform.rotation.x, \
    trans.transform.rotation.y,
    trans.transform.rotation.z,
    trans.transform.rotation.w]
  mat = quaternion_matrix(quat)
  # ADD POSITION INFORMATION TO THE 4x4 HOMOGENEUS MATRIX
  mat[0,3] = trans.transform.translation.x
  mat[1,3] = trans.transform.translation.y
  mat[2,3] = trans.transform.translation.z
  #-----------------------------------------------------------#
  # C. DO CONVERSION USING HOMOGENEOUS TRANSFORM
  #-----------------------------------------------------------#
  pt = [p.point.x, p.point.y, p.point.z, 1.0]
  pt = np.dot(mat, pt)
  return pt, mat


#---------------------------------------------------------------------------#
# 3. OUR CLASS
#---------------------------------------------------------------------------#
class PointPublisher(Node):
  def __init__(self):
    super().__init__('point_pub')
    #-----------------------------------------------------------#
    # A. READ PARAMETERS
    #-----------------------------------------------------------#
    self.declare_parameter('sensor_pos_x', 0.50)
    self.declare_parameter('sensor_pos_y', 0.25)
    self.sensor_pos_x = self.get_parameter('sensor_pos_x').value
    self.sensor_pos_y = self.get_parameter('sensor_pos_y').value
    # 'turtlename' PARAMETER
    self.turtlename = self.declare_parameter(
      'turtlename', 'turtle1').get_parameter_value().string_value
    self.get_logger().info('turtlename: %s' % self.turtlename)
    #-----------------------------------------------------------#
    # B. POINT SENSOR SUBSCRIBER
    #-----------------------------------------------------------#
    self.subscription = self.create_subscription(msg_type=PointStamped,
                                                 topic='point_stamped',
                                                 callback=self.point_listener_cb,
                                                 qos_profile=10)
    #-----------------------------------------------------------#
    # C. CREATE TRANSFORM LISTENER AND ITS BUFFER
    #-----------------------------------------------------------#
    self.tf_buffer   = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
 
  def point_listener_cb(self, msg):
    #-----------------------------------------------------------#
    # CONVERT POINT TO 'world' FRAME
    #-----------------------------------------------------------#
    # GET THE TRANSFORM BETWEEN "sensorframe wrt world"
    try:
      t1 = self.tf_buffer.lookup_transform(
        target_frame = 'world',
        source_frame = msg.header.frame_id,
        time         = rclpy.time.Time(), 
        timeout      = rclpy.duration.Duration(seconds=0.1))
    except TransformException as ex:
      self.get_logger().info(
        f'Could not transform turtlename wrt world: {ex}')
      return
    # COMPUTE POINT USING OUR METHOD
    pt, m = convert_point(t1, msg)
    self.get_logger().info('----------------------')
    self.get_logger().info(' POINT IN world FRAME ')
    self.get_logger().info('pt=[%2.4f,%2.4f,%2.4f]' %(pt[0],pt[1],pt[2]))
    self.get_logger().info('msg.stamp=[%d,%d]' %(msg.header.stamp.sec,msg.header.stamp.nanosec))
    #-----------------------------------------------------------#
    # USING 'transform'
    # https://github.com/ros2/geometry2/blob/jazzy/tf2_ros_py/tf2_ros/buffer_interface.py#L74
    # https://github.com/ros2/geometry2/issues/589 
    #   target_frame:
    #     - 'sensor' works ok
    #     - self.turtlename  works ok
    #     - world FAILS!!!!
    msg.header.stamp = rclpy.time.Time()  # Hay que hacer esto para que funcione
    try:
      pp = self.tf_buffer.transform(object_stamped=msg,
                                    target_frame='world',
                                    timeout=rclpy.duration.Duration(seconds=0.1))
    except TransformException as e:
      self.get_logger().warn('transform failed: %s' %e)
      return
    self.get_logger().info('pp(header)=%s, pp(stamp)=%d, %d' %(pp.header.frame_id, 
                                                               pp.header.stamp.sec, 
                                                               pp.header.stamp.nanosec))
    self.get_logger().info('pp=[%2.4f,%2.4f,%2.4f]' %(pp.point.x,pp.point.y,pp.point.z))
    

#---------------------------------------------------------------------------#
# 3. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main():
  #-----------------------------------------------------------#
  # A. INITIALIZE NODE
  #-----------------------------------------------------------#
  rclpy.init()
  node = PointPublisher()
  #-----------------------------------------------------------#
  # C. GIVE CONTROL TO ROS
  #-----------------------------------------------------------#
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  rclpy.shutdown()

if __name__ == '__main__':
  main()