#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import math
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


#---------------------------------------------------------------------------#
# 2. MAIN CODE
#---------------------------------------------------------------------------#
def generate_launch_description():
  pkg    = 'r2d2_rx_bringup'
  #---------------------------------------------------------------#
	# ROBOT INICIAL POSITION IN THE WORLD
	#---------------------------------------------------------------#
  x_robot   = 1.0
  y_robot   = 2.0
  yaw_robot = 0*math.pi/2

  #---------------------------------------------------------------#
	# CREATE LAUNCH CONFIGURATION VARIABLES
	#---------------------------------------------------------------#
  prefix_robot    = LaunchConfiguration('prefix_robot',   default='r1_')
  use_sim_time    = LaunchConfiguration('use_sim_time',   default='False')
  urdf_file       = LaunchConfiguration('urdf_file_name', default='robot_skeed_basico.xacro')
  # Get value of LaunchConfiguration as a variable
  context= LaunchContext()
  prefix = context.perform_substitution(prefix_robot)
  # Driver variables
  PORT            = LaunchConfiguration('PORT',           default='/dev/ttyUSB0')
  D               = LaunchConfiguration('D',              default='0.1250')
  B               = LaunchConfiguration('B',              default='0.2030')
  cmd_vel_topic   = LaunchConfiguration('cmd_vel_topic',  default=str(prefix+'cmd_vel'))
  odom_topic      = LaunchConfiguration('odom_topic',     default=str(prefix+'odom'))
  odom_frame      = LaunchConfiguration('odom_frame',     default=str(prefix+'odom_tf'))
  base_link_frame = LaunchConfiguration('base_link_frame',default=str(prefix+'base_link'))

  #---------------------------------------------------------------#
  # STATE PUBLISHER NODE
  #---------------------------------------------------------------#
  rsp = IncludeLaunchDescription(
    launch_description_source = PythonLaunchDescriptionSource([
      PathJoinSubstitution([FindPackageShare(pkg) ,'launch','robot_description.launch.py']) 
      ]),
    launch_arguments={'urdf_file_name': urdf_file,
                      'use_sim_time': use_sim_time,}.items(),  # Our sdf file
  )
  
  #---------------------------------------------------------------#
  # DRIVER NODE
  #---------------------------------------------------------------#
  driver_node = IncludeLaunchDescription(
    launch_description_source = PythonLaunchDescriptionSource([
      PathJoinSubstitution([FindPackageShare('r2d2_rx_driver') ,
                            'launch','driver_r2d2_rx.launch.py']) 
      ]),
    launch_arguments={
      'PORT': PORT, 'D': D, 'B': B,
      'cmd_vel_topic':cmd_vel_topic, 'odom_topic':odom_topic, 
      'odom_frame':odom_frame, 'base_link_frame':base_link_frame,
      }.items(),  # Our sdf file
  )
  
  #---------------------------------------------------------------#
  # RVIZ
  #---------------------------------------------------------------#
  rviz_config_dir = PathJoinSubstitution([FindPackageShare(pkg),
                                          'rviz','robot.rviz'])
  rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_dir],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen')
  
  #---------------------------------------------------------------#
  # ADDITIONAL STATE RELATED NODES
  #---------------------------------------------------------------#
  joint_node = Node(
    name='joint_states_pub',
    package='r2d2_rx_driver', 
    executable='publisher_joint_states_diff.py', 
    parameters = [
      {'topic_joint': '/r1_joint_states'},],
    output='screen',
    )
  odom_br = Node(
    name='odom_br',
    package='r2d2_rx_nodes', 
    executable='odom_broadcaster.py', 
    parameters = [
      {'topic_odom': odom_topic},
      {'topic_joint': '/r1_joint_states'},
      {'base_frame': odom_frame},
      {'child_frame': base_link_frame},
      {'use_sim_time': use_sim_time},
      ],
    output='screen',
    )
  # Odom_tf wrs world (This is replaced by a localization algorithm)
  odom_tf = Node(
    name='odom_tf',
    package='r2d2_rx_nodes', 
    executable='static_broadcaster.py', 
    parameters = [
      {'base_frame': '/world'},
      {'child_frame': odom_frame},
      {'x':   x_robot},
      {'y':   y_robot},
      {'yaw': yaw_robot},
      {'use_sim_time': use_sim_time}],
    output='screen',
    )
  
  #---------------------------------------------------------------#
  # TELEOP
  #---------------------------------------------------------------#
  teleop_node = Node(
    name='teleop',
    package='cap_topicos', 
    executable='teleop_qt.py', 
    output='screen',
    remappings=[('cmd_vel', cmd_vel_topic)],
    parameters = [{'v_max': 0.6},
                  {'w_max': 0.8},
                  {'cmd_freq': 10}],
    )

  #---------------------------------------------------------------#
  # RETURN DESCRIPTION
  #---------------------------------------------------------------#
  return LaunchDescription([
    rsp,
    driver_node,
    rviz,
    joint_node,
    odom_br,
    odom_tf,
    teleop_node,
  ])