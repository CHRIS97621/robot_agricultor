cmake_minimum_required(VERSION 3.8)
project(cap_nodos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)


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
  scripts/params_01.py
  scripts/params_02.py
  scripts/params_03.py
  scripts/params_04.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
