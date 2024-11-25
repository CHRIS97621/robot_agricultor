#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import math
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


#---------------------------------------------------------------------------#
# 2. MAIN CODE
#---------------------------------------------------------------------------#
def generate_launch_description():
  world_sdf_file = os.path.join(
    get_package_share_directory('r2d2_rx_description'), 'urdf', 'ground.sdf')
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
  prefix_robot = LaunchConfiguration('prefix_robot', default='r1_')
  use_sim_time = LaunchConfiguration('use_sim_time', default='True')
  urdf_file    = LaunchConfiguration('urdf_file_name', default='robot_skeed_basico.xacro')
  FLAG_GAZEBO  = LaunchConfiguration('FLAG_GAZEBO', default=1)
  # Get value of LaunchConfiguration as a variable
  context= LaunchContext()
  prefix = context.perform_substitution(prefix_robot)

  #---------------------------------------------------------------#
  # STATE PUBLISHER NODE ( robot state publisher)
  #---------------------------------------------------------------#
  rsp = IncludeLaunchDescription(
    launch_description_source = PythonLaunchDescriptionSource([
      PathJoinSubstitution([FindPackageShare(pkg) ,'launch','robot_description.launch.py']) 
      ]),
    launch_arguments={'urdf_file_name': urdf_file,
                      'use_sim_time': use_sim_time,}.items(),  # Our sdf file
  )
  
  #---------------------------------------------------------------#
  # GAZEBO NODES
  #---------------------------------------------------------------#
  pkg_cap_xacro = get_package_share_directory('ros_gz_sim')
  gazebo = IncludeLaunchDescription(
    launch_description_source = PythonLaunchDescriptionSource(
      os.path.join(pkg_cap_xacro, 'launch', 'gz_sim.launch.py') ),
    launch_arguments={'gz_args': '-r ' + world_sdf_file,
                      'use_sim_time': use_sim_time,}.items(),  # Our sdf file
  )
  # Spawn (generamos los parametros del robot en gazebo)
  spawn = Node(
    package='ros_gz_sim', 
    executable='create',
    arguments=[
      '-name', 'r1_robot',
      '-x', str(x_robot), 
      '-y', str(y_robot),
      '-z', '0.5',
      '-R', '0.0',
      '-P', '0.0',
      '-Y', str(yaw_robot),
      '-topic', str('/' + prefix + 'robot_description'),
      #'-topic', '/r1_robot_description'
      ],
    output='screen',
    condition = IfCondition(PythonExpression([FLAG_GAZEBO,'==1'])),
    )
  # Bridge
  bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
      '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
      '/world/r1_robot/model/r1_robot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
      '/model/r1_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
      '/model/r1_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
      '/'+ prefix +'lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
      '/'+ prefix +'imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
      '/'+ prefix +'navsat@gps_msgs/msg/GPSFix[gz.msgs.NavSat',
      '/'+ prefix +'camera@sensor_msgs/msg/Image[gz.msgs.Image'
      ],
    remappings=[
      ('/world/r1_robot/model/r1_robot/joint_state',  '/'+ prefix +'joint_states_gz'),
      ('/model/r1_robot/odometry',                      '/'+ prefix +'odom'),
      ('/r1_lidar',                                     '/'+ prefix +'scan'),
      ('/model/r1_robot/cmd_vel',                       '/'+ prefix +'cmd_vel'),
    ],
    output='screen',
    condition = IfCondition(PythonExpression([FLAG_GAZEBO,'==1'])),
  )
  
  #---------------------------------------------------------------#
  # RVIZ
  #---------------------------------------------------------------#
  rviz_config_dir = PathJoinSubstitution([FindPackageShare(pkg),
                                          'rviz','gazebo.rviz'])
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
    package='r2d2_rx_nodes', 
    executable='joint_states_pub.py', 
    parameters = [
      {'topic_joint': '/r1_joint_states'},],
    output='screen',
    )
  odom_br = Node(
    name='odom_br',
    package='r2d2_rx_nodes', 
    executable='odom_broadcaster.py', 
    parameters = [
      {'topic_odom': '/r1_odom'},
      {'topic_joint': '/r1_joint_states'},
      {'base_frame': '/r1_odom_tf'},
      {'child_frame': '/r1_base_link'},
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
      {'child_frame': '/r1_odom_tf'},
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
    remappings=[('cmd_vel', 'r1_cmd_vel')],
    parameters = [{'v_max': 0.6},
                  {'w_max': 0.8},
                  {'cmd_freq': 10}],
    )

  #---------------------------------------------------------------#
  # RETURN DESCRIPTION
  #---------------------------------------------------------------#
  return LaunchDescription([
    rsp,
    gazebo,
    spawn,
    bridge,
    rviz,
    joint_node,
    odom_br,
    odom_tf,
    teleop_node,
  ])