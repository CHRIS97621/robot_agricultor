#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription

#---------------------------------------------------------------------------#
# 2. INITIALIZE AND PRINT MESSAGES
#---------------------------------------------------------------------------#
def generate_launch_description():
  #---------------------------------------------------------------#
  # INCLUDE ANOTHER LAUNCH DESCRIPTION
  #---------------------------------------------------------------#
  # Find package
  pkg_cap_launch = get_package_share_directory('cap_launch')
  # Set action
  include_launch = IncludeLaunchDescription(
    launch_description_sources.PythonLaunchDescriptionSource(
      pkg_cap_launch + '/launch/demo_07.launch.py'),
      launch_arguments={'load_nodes': str(1), }.items(),
      )
  
  #---------------------------------------------------------------#
	# RETURN LAUNCH DESCRIPTION
	#---------------------------------------------------------------#
  return LaunchDescription([
    include_launch,
  ])