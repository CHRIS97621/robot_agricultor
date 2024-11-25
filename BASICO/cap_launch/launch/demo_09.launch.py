#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
  # GET PATH TO "parametros.yaml"
  #---------------------------------------------------------------#
  pkg_cap_launch = get_package_share_directory('cap_launch')
  param_config = os.path.join(pkg_cap_launch, 'config', 'parametros2.yaml')
  print('param_config:', param_config)
  
  #---------------------------------------------------------------#
  # SET NODES
  #---------------------------------------------------------------#
  robot_node = Node(
    name='robot2',
    package='cap_nodos', 
    executable='params_03.py',
    output='screen', 
    parameters = [param_config], # se cofigura con los paraemetros2.yaml ---#
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