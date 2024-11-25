#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


#---------------------------------------------------------------------------#
# 2. INITIALIZE AND PRINT MESSAGES
#---------------------------------------------------------------------------#
def generate_launch_description():
  #---------------------------------------------------------------#
  # LAUNCH CONFIGURATION VARIABLES
  #---------------------------------------------------------------#
  FLAG_RQT = LaunchConfiguration('FLAG_RQT', default='1')
	
  #---------------------------------------------------------------#
	# DECLARE LAUNCH ARGUMENTS
	#---------------------------------------------------------------#
  declare_load_nodes_cmd = DeclareLaunchArgument(
      name = 'load_nodes',
      default_value = '1',
      description = 'flag para lanzar el turtlesim')
  
  #---------------------------------------------------------------#
  # INCLUDE ANOTHER LAUNCH DESCRIPTION
  #---------------------------------------------------------------#
  # Find package
  pkg_cap_launch = get_package_share_directory('cap_launch')
  # Set action
  include_launch = IncludeLaunchDescription(
    launch_description_sources.PythonLaunchDescriptionSource(
      pkg_cap_launch + '/launch/demo_01.launch.py'),
      condition=IfCondition(LaunchConfiguration('load_nodes')),
      )
  
  #---------------------------------------------------------------#
  # SET NODES
  #---------------------------------------------------------------#
  rqt_graph_node = Node(name='rqt_graph',
                        package='rqt_graph', 
                        executable='rqt_graph',
                        condition = IfCondition(PythonExpression([FLAG_RQT,'==1'])),
                        )
  
  #---------------------------------------------------------------#
	# RETURN LAUNCH DESCRIPTION
	#---------------------------------------------------------------#
  return LaunchDescription([
    declare_load_nodes_cmd,
    include_launch,
    rqt_graph_node,
  ])