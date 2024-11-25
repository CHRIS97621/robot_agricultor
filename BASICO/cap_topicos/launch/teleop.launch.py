#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
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
  FLAG_TURTLESIM     = LaunchConfiguration('FLAG_TURTLESIM')
  FLAG_TURTLESIM_arg = DeclareLaunchArgument('FLAG_TURTLESIM',default_value='1')

  #---------------------------------------------------------------#
  # SET NODES AND PUT THEM IN A GROUP
  #---------------------------------------------------------------#
  teleop_node = Node(name='teleop',
                     package='cap_topicos', 
                     executable='teleop.py', 
                     output='screen',
                     remappings=[('cmd_vel', '/turtle1/cmd_vel')],)

  rqt_reconfigure_node = Node(name='rqt_reconfigure',
                              package='rqt_reconfigure', 
                              executable='rqt_reconfigure',)
  
  rqt_plot_node = Node(name='rqt_plot',
                       package='rqt_plot', 
                       executable='rqt_plot',
                       arguments=["/turtle1/cmd_vel/linear/x", "/turtle1/cmd_vel/angular/z"],
                       output='screen',)
  
  turtlesim_node = Node(condition = IfCondition(PythonExpression([FLAG_TURTLESIM,'==1'])),
                        name='turtlesim',
                        package='turtlesim',
                        executable='turtlesim_node', )
  

  #---------------------------------------------------------------#
  # CREATE THE LAUNCH DESCRIPTION AND POPULATE
  #---------------------------------------------------------------#
  return LaunchDescription([
    FLAG_TURTLESIM_arg,
    turtlesim_node,
    teleop_node,
    rqt_reconfigure_node,
    rqt_plot_node,
  ])