cmake_minimum_required(VERSION 3.8)
project(cap_tf2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
  scripts/static_broadcaster.py
  scripts/dynamic_broadcaster.py
  #scripts/dynamic_publisher_tf.py
  scripts/turtlesim_pose_br.py
  scripts/listener.py
  scripts/publisher_point.py
  scripts/listener_point_transform.py
  scripts/listener_time_travel.py
  DESTINATION lib/${PROJECT_NAME}
)

#-------------------------#
#  INSTALL LAUNCH FILES  #
#-------------------------#
install(DIRECTORY
  launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()