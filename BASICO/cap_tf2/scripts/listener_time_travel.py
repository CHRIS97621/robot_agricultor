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
import rclpy.time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from tf2_ros import TransformException

#---------------------------------------------------------------------------#
# 2. OUR CLASS
#---------------------------------------------------------------------------#
class FramePublisher(Node):
  def __init__(self):
    super().__init__('turtle_time_travel')
    #-----------------------------------------------------------#
    # A. READ PARAMETERS
    #-----------------------------------------------------------#
    # 'turtlename' PARAMETER
    self.turtlename = self.declare_parameter(
      'turtlename', 'turtle1').get_parameter_value().string_value
    self.get_logger().info('turtlename: %s' % self.turtlename)
    #-----------------------------------------------------------#
    # B. CREATE TRANSFORM LISTENER AND ITS BUFFER
    #-----------------------------------------------------------#
    self.tf_buffer   = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    #------------------------------------------------------------------#
    # C. CREATE A CLIENT AND WAIT FOR SERVICE 'spawn'
    #------------------------------------------------------------------#
    # CREATE A CLIENT (type, name)
    client = self.create_client(Spawn, 'spawn')
    # WAIT FOR THE SERVICE
    self.get_logger().info('...waiting for service: spawn')
    while not client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')
    self.get_logger().info('service available')
    # CREATE REQUEST
    req = Spawn.Request()
    req.name = self.turtlename
    req.x = 1.0
    req.y = 2.0
    # CALL SERVICE AND WAIT FOR THE RESULT
    future = client.call_async(req)
    rclpy.spin_until_future_complete(self, future)
    res = future.result()
    self.get_logger().info('spawn turtle: %s' %( res.name ) )
    #------------------------------------------------------------------#
    # D. CREATE A PUBLISHER FOR THE NEW TURTLE
    #------------------------------------------------------------------#
    self.publisher = self.create_publisher(Twist, '%s/cmd_vel' % self.turtlename, 1)
    #-----------------------------------------------------------#
    # E. CREATE TIMER TO LISTEN TRANSFORMS
    #-----------------------------------------------------------#
    self.timer = self.create_timer(0.1, self.on_timer)

  def on_timer(self):
    #-----------------------------------------------------------#
    # Look up for the transformation between turtle1 wrt turtle2
    #-----------------------------------------------------------#
    past = self.get_clock().now() - rclpy.duration.Duration(seconds=5.0) #5
    try:
      t = self.tf_buffer.lookup_transform_full(
        target_frame = self.turtlename,
        target_time  = rclpy.time.Time(),
        source_frame = 'turtle1',
        source_time  = past,     # rclpy.time.Time() also works
        fixed_frame  = 'world', # frame that does not change over time
        timeout      = rclpy.duration.Duration(seconds=1.0))
    except TransformException as ex:
      self.get_logger().info(f'Could not transform : {ex}')
      return
    print('---')
    print('sensor wrt world')
    print('header.stamp:', t.header.stamp)
    print('header.frame_id:', t.header.frame_id)
    print('child_frame_id:', t.child_frame_id)
    print(t.transform.translation)
    print(t.transform.rotation)
    #-----------------------------------------------------------#
    # COMPUTE VELOCITIES TO MAKE "turtle2 FOLLOW TURTLE1
    # USING THE TRANSFORM BETWEEN TWO FRAMES
    #-----------------------------------------------------------#
    #  SET "Twist" OBJECT
    msg = Twist()
    #  SET ANGULAR VELOCITY PROPORTIONAL TO THE ANGLE
    msg.angular.z = 4.0*math.atan2(t.transform.translation.y,
                                    t.transform.translation.x)
    #  SET LINEAR VELOCITY PROPORTIONAL TO THE DISTANCE
    msg.linear.x = 0.5*math.sqrt(t.transform.translation.x ** 2 +
            t.transform.translation.y ** 2)
    # PUBLISH VELOCITY
    self.publisher.publish(msg)


#---------------------------------------------------------------------------#
# 3. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main():
  rclpy.init()
  node = FramePublisher()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  rclpy.shutdown()

if __name__ == '__main__':
  main()