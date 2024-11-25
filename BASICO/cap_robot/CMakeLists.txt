cmake_minimum_required(VERSION 3.8)
project(cap_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
#find_package(tf2_ros_py REQUIRED)
find_package(turtlesim REQUIRED)

#-------------#
#  C++ NODES  #
#-------------#

#----------------#
#  Python NODES  #
#----------------#
# Install module
ament_python_install_package(${PROJECT_NAME})
# Install py executables
install(PROGRAMS
  scripts/wit_ros2_imu2.py
  scripts/imu1_node.py
  scripts/gps1_node.py
  scripts/imu2_node.py
  scripts/gps2_node.py
  scripts/gps3_node.py
  scripts/esp32_gui.py
  scripts/odom2_node.py
  scripts/odom3_node.py
  scripts/interfaz2.py
  scripts/interfaz3.py
  scripts/odom4_node.py
  scripts/odom5_node.py
  scripts/node_enlace.py
  scripts/gps4_nodexel.py
  

  
  DESTINATION lib/${PROJECT_NAME}
)

#-------------------------#
#  INSTALL LAUNCH FILES  #
#-------------------------#
install(DIRECTORY
launch rviz config
DESTINATION share/${PROJECT_NAME}/
)

ament_package()