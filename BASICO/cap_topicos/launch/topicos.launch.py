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
  pub_node = Node(name='pub',
                  package='cap_topicos', 
                  executable='publisher_int.py', 
                  output='screen',
                  prefix='xterm -e',
                  )

  sub_node = Node(name='sub',
                  package='cap_topicos', 
                  executable='subscriber_int.py', 
                  output='screen',
                  prefix='xterm -e',
                  )

  #---------------------------------------------------------------#
  # CREATE THE LAUNCH DESCRIPTION AND POPULATE
  #---------------------------------------------------------------#
  return LaunchDescription([
    pub_node,
    sub_node,
  ])