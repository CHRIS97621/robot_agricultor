#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import SingleThreadedExecutor

#---------------------------------------------------------------------------#
# 2. MY NODE CLASS
#---------------------------------------------------------------------------#
class MyNode(Node):
  def __init__(self, name):
    super().__init__(name)
    self.get_logger().info(f'Creando nodo: {self.get_name()}')
    # Declare parameters
    self.declare_parameter('camera_device_port', '/dev/ttyACM0')
    self.declare_parameter('simulation_mode', False)
    self.declare_parameter('battery_percentage_warning', 15.0)
    # Read parameters
    self.camera_device_port_          = self.get_parameter('camera_device_port').value
    self.simulation_mode_             = self.get_parameter('simulation_mode').value
    self.battery_percentage_warning_  = self.get_parameter('battery_percentage_warning').value
    self.get_logger().info("Parameters - cam_port: %s, sim_mode:%s, battery:%s" %
                           (str(self.camera_device_port_),
                            str(self.simulation_mode_),
                            str(self.battery_percentage_warning_),))
    # Callback to be called every time a parameter’s value has been changed 
    # from the outside.
    self.add_on_set_parameters_callback(self.parameter_callback)
  
  def parameter_callback(self, params):
    success = False
    for param in params:
      if param.name == "camera_device_port":
        if param.type_ == Parameter.Type.STRING:
          if param.value.startswith('/dev/tty'):
            success = True
            self.camera_device_port_ = param.value # Recien hago el cambio
            self.restart_camera = True
      if param.name == "battery_percentage_warning":
        if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
          if param.value >= 0.0 and param.value < 100.0:
            success = True
            self.battery_percentage_warning_ = param.value
      if param.name == "simulation_mode":
        if param.type_ == Parameter.Type.BOOL:
          success = True
          self.simulation_mode_ = param.value
    # Return result
    if not success:
      self.get_logger().warn('parameter_callback: There was an error, parameter not changed')
    return SetParametersResult(successful=success)

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