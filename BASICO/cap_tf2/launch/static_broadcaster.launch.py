#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

#---------------------------------------------------------------------------#
# 2. MAIN CODE
#---------------------------------------------------------------------------#
def generate_launch_description():
  PI = math.pi
  #---------------------------------------------------------------#
	# CREATE LAUNCH CONFIGURATION VARIABLES
	#---------------------------------------------------------------#
  parent_frame = LaunchConfiguration('parent_frame')
  child_frame  = LaunchConfiguration('child_frame')
  x           = LaunchConfiguration('x')
  y           = LaunchConfiguration('y')
  z           = LaunchConfiguration('z')
  roll        = LaunchConfiguration('roll')
  pitch       = LaunchConfiguration('pitch')
  yaw         = LaunchConfiguration('yaw')
  parent_frame_arg = DeclareLaunchArgument('parent_frame', default_value='world')
  child_frame_arg  = DeclareLaunchArgument('child_frame', default_value='turtle')
  x_arg            = DeclareLaunchArgument('x', default_value='1.0')
  y_arg            = DeclareLaunchArgument('y', default_value='0.5')
  z_arg            = DeclareLaunchArgument('z', default_value='0.0')
  roll_arg         = DeclareLaunchArgument('roll',  default_value=str(2*PI/4))
  pitch_arg        = DeclareLaunchArgument('pitch', default_value=str(1*PI/4))
  yaw_arg          = DeclareLaunchArgument('yaw',   default_value=str(0*PI/4))
  # FOR RVIZ
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  FLAG_RVIZ = LaunchConfiguration('FLAG_RVIZ')
  FLAG_RVIZ_arg = DeclareLaunchArgument('FLAG_RVIZ', default_value='1',
    description='flag to launch RVIZ2')
  # RVIZ CONFIG FILE
  rviz_config_file = os.path.join(
    get_package_share_directory('cap_tf2'), 'rviz', 'static_frames.rviz')

  #---------------------------------------------------------------#
  # SET OUR NODES 
  #---------------------------------------------------------------#
  static_br_node = Node(name='static_br',
    package='cap_tf2', 
    executable='static_broadcaster.py', 
    arguments = [parent_frame, child_frame, x, y, z, roll, pitch, yaw],
    output='screen',
    prefix='xterm -e'
    )

  #---------------------------------------------------------------#
  # RVIZ2 NODE
  #---------------------------------------------------------------#
  rviz2_node = Node(
    condition = IfCondition(PythonExpression([FLAG_RVIZ,'==1'])),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_file],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen')

  #---------------------------------------------------------------#
  # CREATE THE LAUNCH DESCRIPTION AND POPULATE
  #---------------------------------------------------------------#
  ld = LaunchDescription()
  ld.add_action(FLAG_RVIZ_arg)
  ld.add_action(parent_frame_arg)
  ld.add_action(child_frame_arg)
  ld.add_action(x_arg)
  ld.add_action(y_arg)
  ld.add_action(z_arg)
  ld.add_action(roll_arg)
  ld.add_action(pitch_arg)
  ld.add_action(yaw_arg)
  ld.add_action(static_br_node)
  ld.add_action(rviz2_node)
  return ld