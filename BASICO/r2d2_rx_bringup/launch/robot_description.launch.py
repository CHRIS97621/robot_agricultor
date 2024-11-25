#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


#---------------------------------------------------------------------------#
# 2. MAIN CODE
#---------------------------------------------------------------------------#
def generate_launch_description():
  #---------------------------------------------------------------#
	# CREATE LAUNCH CONFIGURATION VARIABLES
	#---------------------------------------------------------------#
  use_sim_time      = LaunchConfiguration('use_sim_time')
  urdf_file_name    = LaunchConfiguration('urdf_file_name')
  use_sim_time_arg = DeclareLaunchArgument(name='use_sim_time',
    default_value='True', description='Use simulation (Gazebo) clock if True')
  urdf_file_name_arg = DeclareLaunchArgument(name='urdf_file_name',
    default_value='robot_skeed_basico.xacro', description='Defaul urdf model')

  #---------------------------------------------------------------#
	# SET URDF FILE
	#---------------------------------------------------------------#
  urdf = PathJoinSubstitution([
    FindPackageShare('r2d2_rx_description'),'urdf',urdf_file_name])

  #---------------------------------------------------------------#
  # STATE PUBLISHER NODE
  #---------------------------------------------------------------#
  rsp = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[
        {'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', urdf])} ],
    remappings=[
        ('/robot_description', '/r1_robot_description'),
        ('/joint_states', '/r1_joint_states'),
        ],
    arguments=[])
  
  #---------------------------------------------------------------#
  # RETURN DESCRIPTION
  #---------------------------------------------------------------#
  return LaunchDescription([
    use_sim_time_arg,
    urdf_file_name_arg,
    rsp,
  ])