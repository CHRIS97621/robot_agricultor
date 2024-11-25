#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import rclpy
from rclpy.node import Node


#---------------------------------------------------------------------------#
# 2. CLASS TO MANAGE OUR NODE
#---------------------------------------------------------------------------#
class MyNode(Node):
  # Constructor
  def __init__(self):
    # Call base class constructor
    super().__init__('my_node_py')
    # Declare parameters
    self.declare_parameters(
      namespace='',
      parameters=[
       ('odom_frequency', 100),
       ('port_md', '/dev/ttyACM0'),
       ('type_robot', 'R2D2-01'),
       ('gains', [2.0, 3.0]),
       ('enable', False)
       ])
    # Read parameters
    TYPE_ROBOT = self.get_parameter('type_robot')
    PORT_MD    = self.get_parameter('port_md')
    ODOM_FREQ  = self.get_parameter('odom_frequency')
    GAINS      = self.get_parameter('gains')
    self.get_logger().info("TYPE_ROBOT: %s, type: %s" %
                           (str(TYPE_ROBOT.value),
                            str(TYPE_ROBOT._type_),))
    self.get_logger().info("PORT_MD: %s, type: %s" %
                           (str(PORT_MD.value),
                            str(PORT_MD._type_),))
    self.get_logger().info("ODOM_FREQ: %s, type: %s" %
                           (str(ODOM_FREQ.value),
                            str(ODOM_FREQ._type_),))
    self.get_logger().info("GAINS: %s, type: %s" %
                           (str(GAINS.value),
                            str(GAINS._type_),))

#---------------------------------------------------------------------------#
# 3. FUNCION MAIN
#---------------------------------------------------------------------------#
def main(args=None):
  # INIT ROS 2 AND OUR NODE
  rclpy.init(args=args)
  node = MyNode()
  # SPIN AND CLOSE ALL
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()