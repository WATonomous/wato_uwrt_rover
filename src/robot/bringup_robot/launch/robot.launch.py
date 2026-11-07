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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODES = [
    ("localization", "localization_node", "localization"),
    ("visual_odometry", "visual_odometry_node", "visual_odometry"),
    ("enav", "enav_node", "enav"),
    ("mobility", "mobility_node", "mobility"),
    ("global_localization", "global_localization_node", "global_localization"),
]
# Only meaningful in simulation: they read Gazebo ground truth.
SIM_NODES = [
    ("sim_sensors", "sun_sensor_node", "sun_sensor"),
    ("eval", "eval_node", "eval"),
]


def nodes(context):
    config = os.path.join(get_package_share_directory("bringup_robot"), "config")
    scenario = os.path.join(
        config, "scenarios", LaunchConfiguration("scenario").perform(context) + ".yaml"
    )
    if not os.path.isfile(scenario):
        raise FileNotFoundError(f"unknown scenario: {scenario}")
    params = [os.path.join(config, "params.yaml"), scenario, {"use_sim_time": True}]
    out = [
        Node(package=pkg, executable=exe, name=name, parameters=params, output="screen")
        for pkg, exe, name in NODES
    ]
    out += [
        Node(
            package=pkg,
            executable=exe,
            name=name,
            parameters=params,
            output="screen",
            condition=IfCondition(LaunchConfiguration("sim")),
        )
        for pkg, exe, name in SIM_NODES
    ]
    return out


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="traverse",
                description="config/scenarios/<name>.yaml (traverse, rock_field, sand_trap)",
            ),
            DeclareLaunchArgument(
                "sim",
                default_value="true",
                description="also run sim-only nodes (sun sensor, eval)",
            ),
            OpaqueFunction(function=nodes),
        ]
    )
