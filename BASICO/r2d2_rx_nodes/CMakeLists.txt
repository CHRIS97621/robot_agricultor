cmake_minimum_required(VERSION 3.8)
project(r2d2_rx_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

#----------------#
#  Python NODES  #
#----------------#
# Install module
ament_python_install_package(${PROJECT_NAME})
# Install py executables
install(PROGRAMS
  scripts/dynamic_broadcaster.py
  scripts/joint_states_pub.py
  scripts/static_broadcaster.py
  scripts/odom_broadcaster.py
  scripts/navegation.py
  scripts/navegation2.py
  scripts/mapvizdemo.py

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
