#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
from launch import LaunchDescription
from launch_ros.actions import Node


#---------------------------------------------------------------------------#
# 2. MAIN CODE
#---------------------------------------------------------------------------#
def generate_launch_description():
  #---------------------------------------------------------------#
  # SET NODES AND PUT THEM IN A GROUP
  #---------------------------------------------------------------#
  pub1_node = Node(name='pub1',
                  package='cap_topicos', 
                  executable='publisher_array.py', 
                  output='screen',
                  prefix='xterm -e',
                  )
  pub2_node = Node(name='pub2',
                  package='cap_topicos', 
                  executable='publisher_array.py', 
                  output='screen',
                  prefix='xterm -e',
                  )

  sub_node = Node(name='sub',
                  package='cap_topicos', 
                  executable='subscriber_array.py', 
                  output='screen',
                  prefix='xterm -e',
                  )

  #---------------------------------------------------------------#
  # CREATE THE LAUNCH DESCRIPTION AND POPULATE
  #---------------------------------------------------------------#
  return LaunchDescription([
    pub1_node,
    pub2_node,
    sub_node,
  ])