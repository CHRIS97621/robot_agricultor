#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


#---------------------------------------------------------------------------#
# 2. MAIN CODE
#---------------------------------------------------------------------------#
def generate_launch_description():
  #---------------------------------------------------------------#
  # CREATE LAUNCH CONFIGURATION VARIABLES
  #---------------------------------------------------------------#
  FLAG_TURTLESIM     = LaunchConfiguration('FLAG_TURTLESIM')
  FLAG_TURTLESIM_arg = DeclareLaunchArgument('FLAG_TURTLESIM',default_value='1')
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')

  #---------------------------------------------------------------#
  # CONFIG VARIABLES
  #---------------------------------------------------------------#
  pkg_cap_topicos = get_package_share_directory('cap_topicos')
  rviz_file = os.path.join(pkg_cap_topicos,'rviz','diff_drive.rviz')

  #---------------------------------------------------------------#
  # SET NODES
  #---------------------------------------------------------------#
  teleop_node = Node(name='teleop',
                     package='cap_topicos', 
                     executable='teleop.py', 
                     output='screen',
                     remappings=[('cmd_vel', '/model/vehicle_blue/cmd_vel')],)
  
  odom_node = Node(name='odom',
                   package='cap_topicos', 
                   executable='subscriber_odom.py', 
                    output='screen',
                    remappings=[('odometry', '/model/vehicle_blue/odometry')],)

  rqt_reconfigure_node = Node(name='rqt_reconfigure',
                              package='rqt_reconfigure', 
                              executable='rqt_reconfigure',)
  
  # Rviz
  rviz2 = Node(name='rviz2',
               package='rviz2', 
              executable='rviz2',
              arguments=['-d', rviz_file],
              parameters=[{'use_sim_time': use_sim_time}],
              output='screen')
  

  #---------------------------------------------------------------#
  # CREATE THE LAUNCH DESCRIPTION AND POPULATE
  #---------------------------------------------------------------#
  return LaunchDescription([
    FLAG_TURTLESIM_arg,
    teleop_node,
    odom_node,
    rqt_reconfigure_node,
    rviz2
  ])