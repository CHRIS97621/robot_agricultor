#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

#---------------------------------------------------------------------------#
# 2. MY NODE CLASS
#---------------------------------------------------------------------------#
class MyNode(Node):
  def __init__(self, name):
    super().__init__(name,
                     allow_undeclared_parameters=True,
                     automatically_declare_parameters_from_overrides=True)
    self.get_logger().info(f'Creando nodo: {name}')
    # Check if a parameter is declared. 
    #   If yes, the parameter is retrieved. 
    #   If not, you can specify a default parameter.
    param_str = self.get_parameter_or('my_str', 
                                      Parameter('str', Parameter.Type.STRING, 'Hi'))
    param_int = self.get_parameter_or('my_int', 
                                      Parameter('abc', Parameter.Type.INTEGER, 8))
    param_double_array = self.get_parameter_or('my_double_array', 
                                               Parameter('def', Parameter.Type.DOUBLE_ARRAY, [1.1, 2.2]))
    # Show info
    self.get_logger().info("Params: %s, %s, %s" %
                           (str(param_int.value),
                            str(param_str.value),
                            str(param_double_array.value),))


#---------------------------------------------------------------------------#
# 3. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main(args=None):
  # Init ROS2 and create a node
  rclpy.init(args=args)
  node1 = MyNode('nodo1_py')
  # Create and executor and add nodes
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