#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult


#---------------------------------------------------------------------------#
# 2. PUBLISHER NODE
#---------------------------------------------------------------------------#
class MinimalPublisher(Node):
  def __init__(self, name):
    # Call base class constructor
    super().__init__(name)
    # Declare and read parameters
    self.declare_parameter('lin_vel', 0.0)
    self.declare_parameter('ang_vel', 0.0)
    self.v = self.get_parameter('lin_vel').value
    self.w = self.get_parameter('ang_vel').value
    # Create publisher (msg_type, topic, )
    #qos = qos_profile_sensor_data # Turtlesim don't work with this
    qos = QoSProfile(depth=10,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     reliability=QoSReliabilityPolicy.RELIABLE)
    self.publisher_ = self.create_publisher(Twist, 'cmd_vel', qos)
    # Create a timer
    timer_period = 0.5  # seconds
    self.timer_ = self.create_timer(timer_period, self.timer_callback)
    # Parameter change callback
    self.add_on_set_parameters_callback(self.parameters_callback)

  def timer_callback(self):
    # Create message reading parameter data
    msg = Twist()
    msg.linear.x  = self.v
    msg.angular.z = self.w
    # Publish message
    self.publisher_.publish(msg)

  def parameters_callback(self, params):
    success = False
    for param in params:
      if param.name == "lin_vel":
        if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
          if param.value >= 0.0 and param.value < 1.0:
            success = True
            self.v = param.value
      if param.name == "ang_vel":
        if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
          if param.value >= -2.0 and param.value < 2.0:
            success = True
            self.w = param.value
    return SetParametersResult(successful=success)


#---------------------------------------------------------------------------#
# 3. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main(args=None):
  """
  Run a teleop node
  """
  #------------------------------------------------------------------#
  # A. INITIALIZE ROS
  #------------------------------------------------------------------#
  rclpy.init(args=args)

  #------------------------------------------------------------------#
  # B. CREATE A NODE AND EXECUTE IT
  #------------------------------------------------------------------#
  try:
    minimal_publisher = MinimalPublisher('teleop')
    executor = SingleThreadedExecutor()
    executor.add_node(minimal_publisher)
    try:
      executor.spin()
    finally:
      executor.shutdown()
      minimal_publisher.destroy_node()
  except KeyboardInterrupt:
    pass
  except ExternalShutdownException:
    sys.exit(1)
  finally:
    rclpy.shutdown()


if __name__ == '__main__':
  # Runs the node when this script is run directly (not through an entrypoint)
  main()