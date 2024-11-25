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
  # turtlesim node
  turtle_node = Node(name='turtle',
                     package='turtlesim', 
                     executable='turtlesim_node', 
                     output='screen')
  # teleop node
  teleop_node = Node(name='teleop',
                     package='turtlesim', 
                     executable='turtle_teleop_key',
                     output='screen', prefix='xterm -e')
  # rqt_graph node
  rqt_graph_node = Node(name='rqt_graph',
                        package='rqt_graph', 
                        executable='rqt_graph')

	#---------------------------------------------------------------#
	# ADD ACTIONS TO THE LAUNCH DESCRIPTION
	# The order they are added reflects the order in which they 
	# will be executed
	#---------------------------------------------------------------#
  ld = LaunchDescription()
  ld.add_action(turtle_node)
  ld.add_action(teleop_node)
  ld.add_action(rqt_graph_node)
  return ld