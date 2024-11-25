#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


#---------------------------------------------------------------------------#
# 2. INITIALIZE AND PRINT MESSAGES
#---------------------------------------------------------------------------#
def generate_launch_description():
  #---------------------------------------------------------------#
  # SET NODES
  #---------------------------------------------------------------#
  nodes1 = GroupAction(
    actions=[Node(name='turtle',
                  package='turtlesim', 
                  executable='turtlesim_node', 
                  output='screen'),
            Node(name='teleop',
                 package='turtlesim', 
                 executable='turtle_teleop_key',
                 output='screen', prefix='xterm -e')])
  # Prepare nodes in the namespace "ivan"
  nodes2 = GroupAction(
    actions=[PushRosNamespace(namespace = 'ivan'),
             Node(name='turtle',
                  package='turtlesim', 
                  executable='turtlesim_node', 
                  output='screen'),
            Node(name='teleop',
                 package='turtlesim',
                 executable='turtle_teleop_key',
                 output='screen', prefix='xterm -e')])
  
  # rqt_graph node
  rqt_graph_node = Node(name='rqt_graph',
                        package='rqt_graph', 
                        executable='rqt_graph')
  
  #---------------------------------------------------------------#
	# ADD ACTIONS TO THE LAUNCH DESCRIPTION
	#---------------------------------------------------------------#
  ld = LaunchDescription()
  ld.add_action(nodes1)
  ld.add_action(nodes2)
  ld.add_action(rqt_graph_node)
  return ld