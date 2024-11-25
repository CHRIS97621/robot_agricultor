#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
# 1.1. STANDARD
import sys
import numpy as np
# 1.2. ROS 2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default, qos_profile_default
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from example_interfaces.msg import Int32MultiArray, MultiArrayDimension


#---------------------------------------------------------------------------#
# 2. PUBLISHER NODE
#---------------------------------------------------------------------------#
class MinimalPublisher(Node):
  def __init__(self, name):
    # Call base class constructor
    super().__init__(name)
    # Declare parameters
    self.declare_parameters(namespace='',
                            parameters=[('rows', 2), 
                                        ('cols', 3), ])
    self.ROWS = self.get_parameter('rows').value
    self.COLS = self.get_parameter('cols').value
    self.get_logger().info('rows:%d' % self.ROWS)
    print("cols:", self.COLS)
    # Create QOS profile publisher
    #qos = QoSProfile(depth=10)
    #qos = QoSProfile(depth=10,
    #                 history=QoSHistoryPolicy.KEEP_LAST,
    #                 durability=QoSDurabilityPolicy.VOLATILE,
    #                reliability=QoSReliabilityPolicy.BEST_EFFORT)
    qos = qos_profile_sensor_data
    # Create a publisher (msg_type, topic, )
    self.publisher_ = self.create_publisher(Int32MultiArray, 
                                            topic='topic', 
                                            qos_profile=qos)
    # Create a timer
    timer_period = 2.0  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)


  def timer_callback(self):
    #-------------------------------------------------------------#
    # a. SET VARIABLES
    #-------------------------------------------------------------#
    rows = self.ROWS
    cols = self.COLS

    #-------------------------------------------------------------#
    # b. CREATE MATRIX TO TRANSMIT
    #-------------------------------------------------------------#
    matrix = np.random.randint(5, size=(rows,cols) )

    #-------------------------------------------------------------#
    # c. SET Int32MultiArray MESSAGE
    #-------------------------------------------------------------#
    mat = Int32MultiArray()
    #  -> Specification of data layout 
    #   Dimensions are ordered from outer most to inner most.
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim[0].label = "height"
    mat.layout.dim[1].label = "width"
    mat.layout.dim[0].size  = rows
    mat.layout.dim[1].size  = cols
    mat.layout.dim[0].stride = rows*cols  # just the size of the array
    mat.layout.dim[1].stride = cols
    mat.layout.data_offset   = 0
    #  -> Configure data
    mat.data = [0]*(rows*cols)
    dstride1 = mat.layout.dim[1].stride
    offset   = mat.layout.data_offset
    #  -> Set data
    for i in range(rows):
        for j in range(cols):
            mat.data[offset + dstride1*i + j] = matrix[i,j]

    #-------------------------------------------------------------#
    # d. PUBLISH MESSAGE
    #-------------------------------------------------------------#
    self.publisher_.publish(mat)
    print('---')
    print("published array:")
    print(matrix)



#---------------------------------------------------------------------------#
# 3. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main(args=None):
  """
  Run a minimal_publisher node standalone.
  """
  #------------------------------------------------------------------#
  # A. INITIALIZE ROS
  #------------------------------------------------------------------#
  rclpy.init(args=args)

  #------------------------------------------------------------------#
  # B. CREATE A NODE AND EXECUTE IT
  #------------------------------------------------------------------#
  try:
    minimal_publisher = MinimalPublisher('pub')
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