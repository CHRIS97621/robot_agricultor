cmake_minimum_required(VERSION 3.8)
project(cap_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

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
# ament_python_install_package(${PROJECT_NAME})
# Install py executables
install(PROGRAMS
  scripts/detector_node.py
  scripts/detector_node2.py
  scripts/detector_node3.py
  scripts/detector_node4.py
  scripts/detector_node5.py
  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
