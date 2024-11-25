#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
from launch import LaunchDescription
from launch_ros.actions import Node


#---------------------------------------------------------------------------#
# 2. INITIALIZE AND PRINT MESSAGES
#---------------------------------------------------------------------------#
def generate_launch_description():
  #---------------------------------------------------------------#
  # SET NODES 
  #---------------------------------------------------------------#
  # Prepare nodes in the namespace "ivan"
  turtle_node = Node(name='turtle',
                          package='turtlesim', 
                          executable='turtlesim_node', 
                          namespace = 'ivan',
                          output='screen')
  teleop_node = Node(name='teleop',
                          package='turtlesim', 
                          executable='turtle_teleop_key',
                          namespace = 'ivan', 
                          output='screen', prefix='xterm -e')

  # Prepare nodes in the namespace "r2d2"
  turtle_node_r2d2 = Node(name='turtle',
                          package='turtlesim', 
                          executable='turtlesim_node', 
                          namespace = 'chris', 
                          output='screen')
  teleop_node_r2d2 = Node(name='teleop',
                          package='turtlesim', 
                          executable='turtle_teleop_key',
                          namespace = 'chris', 
                          output='screen', prefix='xterm -e')
  
  # rqt_graph node
  rqt_graph_node = Node(name='rqt_graph',
                        package='rqt_graph', 
                        executable='rqt_graph')
	
  #---------------------------------------------------------------#
	# ADD ACTIONS TO THE LAUNCH DESCRIPTION
	#---------------------------------------------------------------#
  ld = LaunchDescription()
  ld.add_action(turtle_node)
  ld.add_action(teleop_node)
  ld.add_action(turtle_node_r2d2)
  ld.add_action(teleop_node_r2d2)
  ld.add_action(rqt_graph_node)
  return ld