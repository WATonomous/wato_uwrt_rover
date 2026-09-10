# Copyright (c) 2025-present WATonomous. All rights reserved.
#
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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    ld = LaunchDescription()  # Begin building a launch description

    # Shared parameter: odom_topic (used by multiple nodes)
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odom/filtered",
        description="Odometry topic name used by control, map_memory, and planner nodes",
    )
    ld.add_action(odom_topic_arg)

    #################### State Manager Node #####################
    # Owns the rover's WAIT / CONTROL state. Launched first so /rover_state
    # exists (transient-local) before any autonomy node comes up looking for it.
    state_manager_pkg_prefix = get_package_share_directory("state_manager")
    state_manager_param_file = os.path.join(
        state_manager_pkg_prefix, "config", "params.yaml"
    )

    state_manager_param = DeclareLaunchArgument(
        "state_manager_param_file",
        default_value=state_manager_param_file,
        description="Path to config file for state manager node",
    )
    state_manager_node = Node(
        package="state_manager",
        name="state_manager_node",
        executable="state_manager_node",
        parameters=[LaunchConfiguration("state_manager_param_file")],
    )
    ld.add_action(state_manager_param)
    ld.add_action(state_manager_node)

    #################### Costmap Node #####################
    costmap_pkg_prefix = get_package_share_directory("costmap")
    costmap_param_file = os.path.join(costmap_pkg_prefix, "config", "params.yaml")

    costmap_param = DeclareLaunchArgument(
        "costmap_param_file",
        default_value=costmap_param_file,
        description="Path to config file for producer node",
    )
    costmap_node = Node(
        package="costmap",
        name="costmap_node",
        executable="costmap_node",
        parameters=[LaunchConfiguration("costmap_param_file")],
    )
    ld.add_action(costmap_param)
    ld.add_action(costmap_node)

    #################### Map Memory Node #####################
    map_memory_pkg_prefix = get_package_share_directory("map_memory")
    map_memory_param_file = os.path.join(map_memory_pkg_prefix, "config", "params.yaml")

    map_memory_param = DeclareLaunchArgument(
        "map_memory_param_file",
        default_value=map_memory_param_file,
        description="Path to config file for producer node",
    )
    map_memory_node = Node(
        package="map_memory",
        name="map_memory_node",
        executable="map_memory_node",
        parameters=[
            LaunchConfiguration("map_memory_param_file"),
            {"odom_topic": LaunchConfiguration("odom_topic")},
        ],
    )
    ld.add_action(map_memory_param)
    ld.add_action(map_memory_node)

    ##################### Planner Node #####################
    planner_pkg_prefix = get_package_share_directory("planner")
    planner_param_file = os.path.join(planner_pkg_prefix, "config", "params.yaml")

    planner_param = DeclareLaunchArgument(
        "planner_param_file",
        default_value=planner_param_file,
        description="Path to config file for producer node",
    )
    planner_node = Node(
        package="planner",
        name="planner_node",
        executable="planner_node",
        parameters=[
            LaunchConfiguration("planner_param_file"),
            {"odom_topic": LaunchConfiguration("odom_topic")},
        ],
    )
    ld.add_action(planner_param)
    ld.add_action(planner_node)

    ##################### Control Node #####################
    control_pkg_prefix = get_package_share_directory("control")
    control_param_file = os.path.join(control_pkg_prefix, "config", "params.yaml")

    control_param = DeclareLaunchArgument(
        "control_param_file",
        default_value=control_param_file,
        description="Path to config file for producer node",
    )
    control_node = Node(
        package="control",
        name="control_node",
        executable="control_node",
        parameters=[
            LaunchConfiguration("control_param_file"),
            {"odom_topic": LaunchConfiguration("odom_topic")},
        ],
    )
    ld.add_action(control_param)
    ld.add_action(control_node)

    #################### Odometry Spoof Node #####################
    odometry_spoof_node = Node(
        package="odometry_spoof",
        name="odometry_spoof",
        executable="odometry_spoof",
    )
    ld.add_action(odometry_spoof_node)

    #################### GPS Simulation Node #####################
    gps_sim_pkg_prefix = get_package_share_directory("gps_sim")
    gps_sim_param_file = os.path.join(gps_sim_pkg_prefix, "config", "params.yaml")

    gps_sim_param = DeclareLaunchArgument(
        "gps_sim_param_file",
        default_value=gps_sim_param_file,
        description="Path to config file for GPS simulation node",
    )
    gps_sim_node = Node(
        package="gps_sim",
        name="gps_sim_node",
        executable="gps_sim_node",
        parameters=[LaunchConfiguration("gps_sim_param_file")],
    )
    ld.add_action(gps_sim_param)
    ld.add_action(gps_sim_node)

    #################### IMU Simulation Node #####################
    imu_sim_pkg_prefix = get_package_share_directory("imu_sim")
    imu_sim_param_file = os.path.join(imu_sim_pkg_prefix, "config", "params.yaml")

    imu_sim_param = DeclareLaunchArgument(
        "imu_sim_param_file",
        default_value=imu_sim_param_file,
        description="Path to config file for IMU simulation node",
    )
    imu_sim_node = Node(
        package="imu_sim",
        name="imu_sim_node",
        executable="imu_sim_node",
        parameters=[LaunchConfiguration("imu_sim_param_file")],
    )
    ld.add_action(imu_sim_param)
    ld.add_action(imu_sim_node)

    #################### Localization Node (EKF) #####################
    localization_pkg_prefix = get_package_share_directory("localization")
    localization_param_file = os.path.join(
        localization_pkg_prefix, "config", "params.yaml"
    )

    localization_param = DeclareLaunchArgument(
        "localization_param_file",
        default_value=localization_param_file,
        description="Path to config file for localization node",
    )
    localization_node = Node(
        package="localization",
        name="localization_node",
        executable="localization_node",
        parameters=[LaunchConfiguration("localization_param_file")],
    )
    ld.add_action(localization_param)
    ld.add_action(localization_node)

    return ld
