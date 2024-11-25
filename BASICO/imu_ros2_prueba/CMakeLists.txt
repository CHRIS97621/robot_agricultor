cmake_minimum_required(VERSION 3.8)
project(imu_ros2_prueba)

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
  scripts/imu_node.py
  scripts/imu_node2.py
  scripts/imu_node3.py
  scripts/imu_node4.py
  scripts/imu_node5.py
  scripts/imu_node6.py
  scripts/odom_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
