#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. IMPORT LIBRARIES
#---------------------------------------------------------------------------#
#  1.1. STANDARD LIBRARIES
#  1.2. ROS LIBRARIES
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
from tf_transformations import quaternion_from_euler


#---------------------------------------------------------------------------#
# 2. OUR CLASS
#---------------------------------------------------------------------------#
class TurtlesimPublisher(Node):
  def __init__(self):
    super().__init__('turtle_br')
    #-----------------------------------------------------------#
    # A. READ turtlename' PARAMETER
    #-----------------------------------------------------------#
    self.turtlename = self.declare_parameter(
      'turtlename', 'turtle1').get_parameter_value().string_value
    self.get_logger().info('turtlename: %s' % self.turtlename)
    #-----------------------------------------------------------#
    # B. TURTLESIM CONFIGURATION
    #-----------------------------------------------------------#
    # SUBSCRIBE TO turtlename/pose TOPIC
    self.subscription = self.create_subscription(
      Pose,
      f'/{self.turtlename}/pose',
      self.handle_turtle_pose,
      1)
    #self.subscription  # prevent unused variable warning
    #-----------------------------------------------------------#
    # C. CREATE TRANSFORM OBJECTS
    #-----------------------------------------------------------#
    self.tf_broadcaster = TransformBroadcaster(self)
    self.t = TransformStamped()
    self.t.header.frame_id = 'world'
    self.t.child_frame_id  = self.turtlename
    
  def handle_turtle_pose(self, msg):
    #-----------------------------------------------------------#
    # SET THE DYNAMIC FRAME
    #-----------------------------------------------------------#
    t = self.t
    # SET "timestamp"
    t.header.stamp = self.get_clock().now().to_msg()
    # SET POSITION OF CHILD FRAME
    #   Turtle only exists in 2D, thus we get x and y translation
    #   coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    # SET ROTATION OF CHILD FRAME
    #   For the same reason, turtle can only rotate around one axis
    #   and this why we set rotation in x and y to 0 and obtain
    #   rotation in z axis from the message
    q = quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    #----------------------------------------------------#
    # SEND THE TRANSFORM USING THE "broadcaster"
    #----------------------------------------------------#
    self.tf_broadcaster.sendTransform(t)


#---------------------------------------------------------------------------#
# 3. MAIN FUNCTION
#---------------------------------------------------------------------------#
def main():
  rclpy.init()
  node = TurtlesimPublisher()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  rclpy.shutdown()

if __name__ == '__main__':
  main()