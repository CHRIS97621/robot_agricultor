#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
#obetener la ruta del directorio share de un paquete en ros2#

from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


#---------------------------------------------------------------------------#
# 2. INITIALIZE AND PRINT MESSAGES
#---------------------------------------------------------------------------#
def generate_launch_description():
  #---------------------------------------------------------------#
  # INCLUDE ANOTHER LAUNCH DESCRIPTION
  #---------------------------------------------------------------#
  # Find package
  # pkg_cap_launch es una variable que almacena la ruta absoluta del directorio share #
  pkg_cap_launch = get_package_share_directory('cap_launch')
  print('---')
  print('pkg_cap_launch:', pkg_cap_launch)
  print('---')
  # Set action
  include_launch = IncludeLaunchDescription(
    launch_description_source=launch_description_sources.PythonLaunchDescriptionSource(
      pkg_cap_launch + '/launch/demo_01.launch.py'))
  
  #---------------------------------------------------------------#
  # SET NODES
  #---------------------------------------------------------------#
  rqt_graph_node = Node(name='rqt_graph',
                        package='rqt_graph', 
                        executable='rqt_graph')
  
  #---------------------------------------------------------------#
	# RETURN LAUNCH DESCRIPTION
	#---------------------------------------------------------------#
  return LaunchDescription([
    include_launch,
    rqt_graph_node,
  ])