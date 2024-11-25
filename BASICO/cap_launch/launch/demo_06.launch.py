#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
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
  # INCLUDE ANOTHER LAUNCH DESCRIPTION
  #---------------------------------------------------------------#
  # Find package
  pkg_cap_launch = get_package_share_directory('cap_launch')
  print('---')
  print('pkg_cap_launch:', pkg_cap_launch)
  print('---')
  # Set action
  include_launch = IncludeLaunchDescription(
    launch_description_sources.PythonLaunchDescriptionSource(
      pkg_cap_launch + '/launch/demo_01.launch.py'))
  
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
    include_launch,
    rqt_graph_node,
  ])