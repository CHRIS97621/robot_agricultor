#!/usr/bin/env python3

#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


#---------------------------------------------------------------------------#
# 2. MAIN CODE
#---------------------------------------------------------------------------#
def generate_launch_description():
  #---------------------------------------------------------------#
  # CREATE LAUNCH CONFIGURATION VARIABLES
  #---------------------------------------------------------------#
  FLAG_GAZEBO     = LaunchConfiguration('FLAG_GAZEBO')
  FLAG_GAZEBO_arg = DeclareLaunchArgument('FLAG_GAZEBO',
                                          default_value='1',
                                          description='flag to launch GAZEBO')
  
  #---------------------------------------------------------------#
  # SET WORLD FILE
  #---------------------------------------------------------------#
  # GET PACKAGE
  pkg_cap_launch = get_package_share_directory('cap_launch')
  # SET GAZEBO WORLD
  world_path = os.path.join(pkg_cap_launch,'worlds', 'world_01.sdf')
  print('---')
  print('world_path:', world_path)

  #---------------------------------------------------------------#
  # GAZEBO "HARMONIC" NODE
  #---------------------------------------------------------------#
  pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
  gazebo = IncludeLaunchDescription(
    launch_description_source = PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py') ),
    launch_arguments={'gz_args': '-r ' + world_path}.items(),
    condition = IfCondition(PythonExpression([FLAG_GAZEBO,'==1'])),
  )

  #---------------------------------------------------------------#
  # RETURN DESCRIPTION
  #---------------------------------------------------------------#
  return LaunchDescription([
    FLAG_GAZEBO_arg,
    gazebo,
  ])