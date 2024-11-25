#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

#---------------------------------------------------------------------------#
# 2. MY NODE CLASS
#---------------------------------------------------------------------------#
class MyNode(Node):
  def __init__(self, name):
    super().__init__(name)
    self.get_logger().info('Creando nodo:%s' %(name))
    # Declare parameters
    self.declare_parameter('my_str', 'ivan')
    self.declare_parameter(name='my_int', value=2)
    self.declare_parameter('my_double_array', value=[4.4, 5.5, 6.6])


#---------------------------------------------------------------------------#
# 3. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main(args=None):
  # Init ROS2 and create a node
  rclpy.init(args=args)
  node1 = MyNode('nodo1_py')
  # Create an executor and add nodes
  executor = SingleThreadedExecutor()
  executor.add_node(node1)
  try:
    executor.spin()
  finally:
    executor.shutdown()
    node1.destroy_node()
  # Close all
  rclpy.shutdown()

if __name__ == '__main__':
  main()