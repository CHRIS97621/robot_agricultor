#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

#---------------------------------------------------------------------------#
# 2. MAIN CODE
#---------------------------------------------------------------------------#
def generate_launch_description():
  #---------------------------------------------------------------#
  # SET NODES
  #---------------------------------------------------------------#
  nodes1 = GroupAction(
    actions=[
      PushRosNamespace(namespace = 'ivan'),
      Node(name='pub',
                package='cap_topicos', 
                executable='publisher_int.py', 
                output='screen',
                prefix='xterm -e'),
      Node(name='sub',
            package='cap_topicos', 
            executable='subscriber_int.py', 
            output='screen',
            prefix='xterm -e'),
                ])

  rqt_graph_node = Node(name='rqt_graph',
                  package='rqt_graph', 
                  executable='rqt_graph', 
                  output='screen',)

  #---------------------------------------------------------------#
  # CREATE THE LAUNCH DESCRIPTION AND POPULATE
  #---------------------------------------------------------------#
  return LaunchDescription([
    nodes1,
    rqt_graph_node,
  ])