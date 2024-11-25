#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import os
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
  #---------------------------------------------------------------#
	# CREATE LAUNCH CONFIGURATION VARIABLES
	#---------------------------------------------------------------#
  # FOR RVIZ
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  FLAG_RVIZ = LaunchConfiguration('FLAG_RVIZ')
  FLAG_RVIZ_arg = DeclareLaunchArgument(
    'FLAG_RVIZ', default_value='1', description='flag to launch RVIZ2')
  # RVIZ CONFIG FILE
  rviz_config_dir = os.path.join(
    get_package_share_directory('cap_tf2'), 'rviz', 'listener_time_travel.rviz')

  #---------------------------------------------------------------#
  # SET OUR NODES 
  #---------------------------------------------------------------#
  turtlesim_node = Node(
    package='turtlesim',
    executable='turtlesim_node',
    name='turtlesim',
    )

  turtlesim_teleop_node = Node(
    package='turtlesim',
    executable='turtle_teleop_key',
    name='turtlesim_teleop',
    output='screen',
    prefix='xterm -e',       # Solo funciona bien con esto
    )

  turtle1_br = Node(name='turtle1_br',
    package='cap_tf2', 
    executable='turtlesim_pose_br.py',
    parameters = [
      {'turtlename': 'turtle1'}],
    output='screen',
    )
  
  turtle2_br = Node(name='turtle2_br',
    package='cap_tf2', 
    executable='turtlesim_pose_br.py',
    parameters = [
      {'turtlename': 'turtle2'}],
    output='screen',
    )
  
  listener_time_travel_node = Node(name='listener_time_travel',
    package='cap_tf2', 
    executable='listener_time_travel.py',
    parameters = [
      {'turtlename': 'turtle2'}],
    #output='screen',
    prefix='xterm -e',
    )

  #---------------------------------------------------------------#
  # RVIZ2 NODE
  #---------------------------------------------------------------#
  rviz2_node = Node(
    condition = IfCondition(PythonExpression([FLAG_RVIZ,'==1'])),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_dir],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen')

  #---------------------------------------------------------------#
  # CREATE THE LAUNCH DESCRIPTION AND POPULATE
  #---------------------------------------------------------------#
  ld = LaunchDescription()
  ld.add_action(FLAG_RVIZ_arg)
  ld.add_action(turtlesim_node)
  ld.add_action(turtlesim_teleop_node)
  ld.add_action(turtle1_br)
  ld.add_action(turtle2_br)
  ld.add_action(listener_time_travel_node)
  ld.add_action(rviz2_node)
  return ld