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
  # SET NODES
  #---------------------------------------------------------------#
  robot_node = Node(
    name='robot',
    package='cap_nodos', 
    executable='params_02.py',
    output='screen', 
    parameters = [
      {'camera_device_port': '/dev/ttyACM2'},
      {'simulation_mode': False},
      {'battery_percentage_warning': 50.0}],
    )
  
  #---------------------------------------------------------------#
  # SET NODES
  #---------------------------------------------------------------#
  rqt_graph_node = Node(name='rqt_reconfigure',
                        package='rqt_reconfigure', 
                        executable='rqt_reconfigure',
                        condition = IfCondition(PythonExpression([FLAG_RQT,'==1'])),
                        )
  
  #---------------------------------------------------------------#
	# RETURN LAUNCH DESCRIPTION
	#---------------------------------------------------------------#
  return LaunchDescription([
    robot_node,
    rqt_graph_node,
  ])