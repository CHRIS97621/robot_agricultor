#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

#---------------------------------------------------------------------------#
# 2. INITIALIZE AND PRINT MESSAGES
#---------------------------------------------------------------------------#
def generate_launch_description():
  #---------------------------------------------------------------#
  # SET WORLD PATH
  #---------------------------------------------------------------#
  # GET PACKAGE
  pkg_cap_launch = get_package_share_directory('cap_launch')
  # SET GAZEBO WORLD
  GAZEBO_WORLD = 0
  if(GAZEBO_WORLD==0):
    world_path = os.path.join(pkg_cap_launch, 
                              'worlds', 'world_01.sdf')
  elif(GAZEBO_WORLD==1):
    world_path = os.path.join(pkg_cap_launch, 
                              'worlds', 'world_02.sdf')
  print('---')
  print('world_path:', world_path)

  #---------------------------------------------------------------#
  # EXECUTE IGNITION HARMONIC v8.3
  #---------------------------------------------------------------#
  exe_action = ExecuteProcess(
     cmd=['gz sim', world_path],
     shell=True,
     output='screen')

	#---------------------------------------------------------------#
	# CREATE THE LAUNCH DESCRIPTION AND POPULATE
	#---------------------------------------------------------------#
  ld = LaunchDescription()
  ld.add_action(exe_action)
  return ld