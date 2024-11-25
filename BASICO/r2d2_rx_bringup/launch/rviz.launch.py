#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#---------------------------------------------------------------------------#
# 2. MAIN CODE
#---------------------------------------------------------------------------#
def generate_launch_description():
  pkg= 'r2d2_rx_bringup'
  #---------------------------------------------------------------#
	# CREATE LAUNCH CONFIGURATION VARIABLES
	#---------------------------------------------------------------#
  use_sim_time     = LaunchConfiguration('use_sim_time', default='False')
  urdf_file_name   = LaunchConfiguration('urdf_file_name', 
                                         default='robot_skeed_basico.xacro')

  #---------------------------------------------------------------#
  # STATE PUBLISHER NODE
  #---------------------------------------------------------------#
  rsp = IncludeLaunchDescription(
    launch_description_source = PythonLaunchDescriptionSource([
      PathJoinSubstitution([FindPackageShare(pkg) ,'launch','robot_description.launch.py']) 
      ]),
    launch_arguments={'urdf_file_name': urdf_file_name,
                      'use_sim_time': use_sim_time,}.items(),  # Our sdf file
  )
  jsp = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    output='screen',
    remappings=[
      ('/robot_description', '/r1_robot_description'),
      ('/joint_states', '/r1_joint_states'),
      ], 
    )

  #---------------------------------------------------------------#
  # RVIZ
  #---------------------------------------------------------------#
  rviz_config_dir = PathJoinSubstitution([FindPackageShare(pkg),
                                          'rviz','rviz.rviz'])
  rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_dir],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen')

  #---------------------------------------------------------------#
  # RETURN DESCRIPTION
  #---------------------------------------------------------------#
  return LaunchDescription([
    rsp,
    jsp,
    rviz,
  ])