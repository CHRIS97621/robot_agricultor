#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from example_interfaces.msg import Int32MultiArray


#---------------------------------------------------------------------------#
# 2. SUBSCRIBER NODE
#---------------------------------------------------------------------------#
class MinimalSubscriber(Node):
  def __init__(self, name):
    # Call base class constructor
    super().__init__(name)
    # Create a subscriber (msg_type, topic, callback, )
    qos = QoSProfile(depth=10)
    qos = qos_profile_sensor_data
    self.subscription = self.create_subscription(
      Int32MultiArray, 'topic', self.listener_callback, qos)
    self.subscription  # prevent unused variable warning


  def listener_callback(self, msg):
    #------------------------------------------------------------------#
    # GET SPECIFICATION OF DATA LAYOUT
    #------------------------------------------------------------------#
    print('---')
    print('layout.dim[0]:')
    print(msg.layout.dim[0])
    print('layout.dim[1]:')
    print(msg.layout.dim[1])
    print('layout.data_offset:', msg.layout.data_offset)
    # GET INFO
    ROWS     = msg.layout.dim[0].size
    COLS     = msg.layout.dim[1].size
    dstride0 = msg.layout.dim[0].stride # dim[0] stride is just size of the array
    dstride1 = msg.layout.dim[1].stride
    offset   = msg.layout.data_offset
    print('ROWS:%d, COLS:%d' %(ROWS, COLS))

    #------------------------------------------------------------------#
    # GET DATA
    #------------------------------------------------------------------#
    data   = [0]*(ROWS*COLS)
    matrix = np.zeros( (ROWS, COLS), dtype=np.int32 )
    for i in range(ROWS):
      for j in range(COLS):
        data[i*COLS + j] = msg.data[offset + dstride1*i + j] # Lo guardo 1D
        matrix[i, j]     = msg.data[offset + dstride1*i + j] # Lo guardo 2D
    # Print results
    print('data:', data)
    print('matrix:')
    print(matrix)



#---------------------------------------------------------------------------#
# 3. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main(args=None):
  """
  Run a minimal_subscriber node standalone.
  """
  #------------------------------------------------------------------#
  # A. INITIALIZE ROS2
  #------------------------------------------------------------------#
  rclpy.init(args=args)

  #------------------------------------------------------------------#
  # B. CREATE A NODE AND EXECUTE IT
  #------------------------------------------------------------------#
  try:
    minimal_subscriber = MinimalSubscriber('sub')
    executor = SingleThreadedExecutor()
    executor.add_node(minimal_subscriber)
    try:
      executor.spin()
    finally:
      executor.shutdown()
      minimal_subscriber.destroy_node()
  except KeyboardInterrupt:
    pass
  except ExternalShutdownException:
    sys.exit(1)
  finally:
    rclpy.shutdown()


if __name__ == '__main__':
  # Runs the node when this script is run directly (not through an entrypoint)
  main()