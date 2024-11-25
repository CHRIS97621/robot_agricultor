cmake_minimum_required(VERSION 3.8)
project(gps_ros2_pruebas)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


#----------------#
#  C++ programs  #
#----------------#
# Create executable nodes and link dependencies.
#add_executable(demo src/demo.cpp)
#ament_target_dependencies(demo rclcpp)
# Install nodes
#install(
#  TARGETS demo
#  DESTINATION lib/${PROJECT_NAME}
#)

#-------------------#
#  Python programs  #
#-------------------#
# Install module
ament_python_install_package(${PROJECT_NAME})
# Install py executables
install(PROGRAMS
  scripts/gps_node.py
  scripts/gps_node2.py
  scripts/gps_node3.py
  scripts/gps_node4r.py
  scripts/gps_node5r.py
  scripts/gps_node6r.py
  scripts/gps_node7r.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
