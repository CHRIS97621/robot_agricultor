#---------------------------------------------------------------------------#
# 1. LIBRARIES
#---------------------------------------------------------------------------#
import launch
from launch_ros.actions import Node


#---------------------------------------------------------------------------#
# 2. INITIALIZE AND PRINT MESSAGES
#---------------------------------------------------------------------------#
def generate_launch_description():
  return launch.LaunchDescription([
    # Action that executes a ROS node
    Node(name='turtleivan',
         package='turtlesim', 
         executable='turtlesim_node', 
         output='screen'),
         
    
    Node(name='teleopcrhis',
         package='turtlesim', 
         executable='turtle_teleop_key', 
         output='screen',
         prefix='xterm -e'),
         ])
         


