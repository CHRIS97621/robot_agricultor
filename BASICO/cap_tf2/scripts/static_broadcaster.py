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
from tf_transformations import quaternion_from_euler

#---------------------------------------------------------------------------#
# 2. AUXILIAR FUNCTION (por si no se tiene el paquete tf_transformations)
#---------------------------------------------------------------------------#
'''
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
'''

#---------------------------------------------------------------------------#
# 3. OUR CLASS
#---------------------------------------------------------------------------#
class StaticFramePublisher(Node):
  """
  Broadcast transforms that never change.
  """

  def __init__(self, transformada):
    super().__init__('static_py_broadcaster')
    # Create a "StaticTransformBroadcaster" object
    self.tf_static_broadcaster = StaticTransformBroadcaster(self)
    # Publish static transforms once at startup
    self.make_transforms(transformada)

  def make_transforms(self, transformada):
    #-----------------------------------------------------------#
    # CREATE A "TransformStamped" OBJECT WHICH WILL BE THE
    # MESSAGE WE WILL SEND OVER
    #-----------------------------------------------------------#
    # CREATE A "TransformStamped" OBJECT
    t = TransformStamped()
    # SET "header" INFORMATION
    t.header.stamp    = self.get_clock().now().to_msg()
    t.header.frame_id = transformada[1]
    # SET "frame_id" OF THE CHILD FRAME
    t.child_frame_id = transformada[2]
    #  SET "transform" INFORMATION
    #   -> Set position of the child frame
    t.transform.translation.x = float(transformada[3])
    t.transform.translation.y = float(transformada[4])
    t.transform.translation.z = float(transformada[5])
    #   -> Set rotation of the child frame (First get quaternion)
    quat = quaternion_from_euler(
        float(transformada[6]), float(transformada[7]), float(transformada[8]))
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    #-----------------------------------------------------------#
    # SEND THE TRANSFORM USING THE "broadcaster"
    #-----------------------------------------------------------#
    self.tf_static_broadcaster.sendTransform(t)
    self.get_logger().info("Spinning until killed publishing %s wrt %s"\
        %(transformada[2], transformada[1]))


#---------------------------------------------------------------------------#
# 4. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main(args=None):
  #-----------------------------------------------------------#
  # A. CHECK COMMAND LINE ARGUMENTS
  #-----------------------------------------------------------#
  logger = rclpy.logging.get_logger('logger')
  print('sys.argv[0]:',sys.argv[0])
  print('sys.argv:',sys.argv)
  if len(sys.argv) <  9:
    logger.info( 'number of parameters %d' % len(sys.argv))
    logger.info('Invalid number of parameters. Usage: \n'
                '$ ros2 run cap_tf2 static_broadcaster.py '
                'frame_name child_frame_name  x y z  roll pitch yaw')
    sys.exit(1)

  #-----------------------------------------------------------#
  # B. INITIALIZE ROS AND CREATE NODE
  #-----------------------------------------------------------#
  rclpy.init()
  node = StaticFramePublisher(sys.argv)

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