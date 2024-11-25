#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. IMPORT LIBRARIES
#---------------------------------------------------------------------------#
#  1.1. STANDARD LIBRARIES
#  1.2. ROS LIBRARIES
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler


#---------------------------------------------------------------------------#
# 2. OUR CLASS
#---------------------------------------------------------------------------#
class FramePublisher(Node):
  def __init__(self):
    super().__init__('turtle_tf2_br')
    #-----------------------------------------------------------#
    # A. READ PARAMETERS
    #-----------------------------------------------------------#
    self.declare_parameter('sensor_pos_x', 0.50)
    self.declare_parameter('sensor_pos_y', 0.25)
    self.sensor_pos_x = self.get_parameter('sensor_pos_x').value
    self.sensor_pos_y = self.get_parameter('sensor_pos_y').value
    # 'turtlename' PARAMETER
    self.turtlename = self.declare_parameter(
      'turtlename', 'turtle1').get_parameter_value().string_value
    self.get_logger().info('turtlename: %s' % self.turtlename)
    #-----------------------------------------------------------#
    # B. CREATE TRANSFORM BROADCASTERS
    #-----------------------------------------------------------#
    self.tf_static_broadcaster = StaticTransformBroadcaster(self)
    #-----------------------------------------------------------#
    # C. CREATE "TransformStamped" OBJECTS
    #-----------------------------------------------------------#
    self.ts = TransformStamped()
    self.ts.header.frame_id = self.turtlename
    self.ts.child_frame_id  = 'sensor'
    #-----------------------------------------------------------#
    # D. PUBLISH STATIC FRAME
    #-----------------------------------------------------------#
    self.publish_static_tf()
    #-----------------------------------------------------------#
    # E. CREATE TRANSFORM LISTENER AND ITS BUFFER
    #-----------------------------------------------------------#
    self.tf_buffer   = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    #-----------------------------------------------------------#
    # F. CREATE TIMER TO LISTEN TRANSFORMS
    #-----------------------------------------------------------#
    self.timer = self.create_timer(0.5, self.on_timer)
  
  def publish_static_tf(self):
    #-----------------------------------------------------------#
    # SET THE STATIC TRANSFORM
    #-----------------------------------------------------------#
    t = self.ts
    #  SET "timestamp"
    t.header.stamp = self.get_clock().now().to_msg()
    #  SET POSITION OF CHILD FRAME
    t.transform.translation.x = self.sensor_pos_x
    t.transform.translation.y = self.sensor_pos_y
    t.transform.translation.z = 0.0
    #  SET ROTATION OF CHILD FRAME
    quat = quaternion_from_euler(0.0, 0.0, 0.0)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    #-----------------------------------------------------------#
    # SEND THE TRANSFORM USING THE "broadcaster"
    #-----------------------------------------------------------#
    self.tf_static_broadcaster.sendTransform(t)

  def on_timer(self):
    #-----------------------------------------------------------#
    # GET THE TRANSFORM BETWEEN "turtlename wrt world"
    #-----------------------------------------------------------#
    try:
      t1 = self.tf_buffer.lookup_transform(
        target_frame = 'world',
        source_frame = self.turtlename,
        time         = rclpy.time.Time())
    except TransformException as ex:
      self.get_logger().info(
        f'Could not transform turtlename wrt world: {ex}')
      return
    self.get_logger().info('---')
    self.get_logger().info('turtlename wrt world')
    self.get_logger().info('header.stamp(nsec): %d' %(t1.header.stamp.nanosec))
    self.get_logger().info('header.frame_id: %s' %(t1.header.frame_id))
    self.get_logger().info('child_frame_id: %s' %(t1.child_frame_id))
    self.get_logger().info('pos=%2.4f,%2.4f,%2.4f' %(t1.transform.translation.x, 
                                                  t1.transform.translation.y,
                                                  t1.transform.translation.z))
    self.get_logger().info('rot=%2.4f,%2.4f,%2.4f,%2.4f' %(t1.transform.rotation.x, 
                                                  t1.transform.rotation.y,
                                                  t1.transform.rotation.z,
                                                  t1.transform.rotation.w))
    #-----------------------------------------------------------#
    # GET THE TRANSFORM BETWEEN "sensor wrt world"
    #-----------------------------------------------------------#
    try:
      t2 = self.tf_buffer.lookup_transform(
        target_frame = 'world',
        source_frame = 'sensor',
        time         = rclpy.time.Time(),
        timeout      = rclpy.duration.Duration(seconds=0.1)) # How long to block before failing
    except TransformException as ex:
      self.get_logger().info(
        f'Could not transform sensor wrt world: {ex}')
      return
    self.get_logger().info('sensor wrt world')
    self.get_logger().info('header.stamp(nsec): %ld' %(t2.header.stamp.nanosec))
    self.get_logger().info('header.frame_id: %s' %(t2.header.frame_id))
    self.get_logger().info('child_frame_id: %s' %(t2.child_frame_id))
    self.get_logger().info('pos=%2.4f,%2.4f,%2.4f' %(t2.transform.translation.x, 
                                                  t2.transform.translation.y,
                                                  t2.transform.translation.z))
    self.get_logger().info('rot=%2.4f,%2.4f,%2.4f,%2.4f' %(t2.transform.rotation.x, 
                                                  t2.transform.rotation.y,
                                                  t2.transform.rotation.z,
                                                  t2.transform.rotation.w))


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