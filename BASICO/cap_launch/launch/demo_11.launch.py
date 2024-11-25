#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart, OnShutdown


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
	# RETURN LAUNCH DESCRIPTION
	#---------------------------------------------------------------#
  #ld.add_action(rqt_graph_node)
  return LaunchDescription([
    RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=teleop_node,
                on_start=[turtle_node],
            )),
    teleop_node,
    rqt_graph_node,
    # Message when launch file is shutting down
    RegisterEventHandler(
      event_handler=OnShutdown(
        on_shutdown=[LogInfo(msg=['Launch file is shutting down'])])),
  ])