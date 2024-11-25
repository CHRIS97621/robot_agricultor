cmake_minimum_required(VERSION 3.8)
project(cap_topicos)

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
find_package(cap_msgs REQUIRED)

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

#----------------#
#  Python NODES  #
#----------------#
# Install module
ament_python_install_package(${PROJECT_NAME})
# Install py executables
install(PROGRAMS
  scripts/publisher_array.py
  scripts/publisher_int.py
  scripts/publisher_string.py
  scripts/subscriber_array.py
  scripts/subscriber_int.py
  scripts/subscriber_odom.py
  scripts/subscriber_string.py
  scripts/teleop_qt.py
  scripts/teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

#-----------------------#
#  INSTALL DIRECTORIES  #
#-----------------------#
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/

)

ament_package()
