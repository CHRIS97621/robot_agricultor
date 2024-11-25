# Copyright 2018 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
import launch.actions


def generate_launch_description():
    # Define el directorio del paquete r2d2_rx_bringup
    gps_wpf_dir = get_package_share_directory("r2d2_rx_bringup")
    rl_params_file = os.path.join(gps_wpf_dir, "config", "ekf_global.yaml")  # Asegúrate de tener este archivo

    return LaunchDescription(
        [
            # Declarar los argumentos de lanzamiento
            DeclareLaunchArgument(
                "output_final_position", default_value="false"
            ),
            DeclareLaunchArgument(
                "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
            ),
            
            # Nodo de filtro EKF para odometría (local)
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                remappings=[
                    ("odometry/filtered", "r1_odom")  # Asegúrate de que el tópico de salida sea el correcto
                ],
            ),
            
            # Nodo de filtro EKF para mapa (global)
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                remappings=[
                    ("odometry/filtered", "r1_odom_global")  # Asegúrate de que el tópico de salida sea el correcto
                ],
            ),
            
            # Nodo de transformación NavSat (fusión de GPS)
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                remappings=[
                    ("imu/data", "r1_imu/data"),  # Ajusta el nombre del tópico IMU si es diferente
                    ("gps/fix", "r1_gps/fix"),  # Remapea el tópico GPS según el nombre de tu nodo GPS
                    ("gps/filtered", "r1_gps/filtered"),  # Usa el nombre correcto para el GPS filtrado
                    ("odometry/gps", "r1_odom_gps"),  # Ajusta según el nombre de tu odometría GPS
                    ("odometry/filtered", "r1_odom_global")  # Remapea al filtro global
                ],
            ),
        ]
    )
