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

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

# Gazebo -> ROS topics the rover sees. Ground truth goes to /sim/* and is only used by
# simulated sensors and the eval node, never by the autonomy stack directly.
BRIDGED_TOPICS = [
    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
    "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
    "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
    "/world/mars/model/rover/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
    "/navcam/image@sensor_msgs/msg/Image[ignition.msgs.Image",
    "/navcam/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
    "/navcam/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
    "/sim/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
]


def generate_launch_description():
    description = get_package_share_directory("rover_description")
    sim_share = get_package_share_directory("rover_sim")
    world = os.path.join(description, "worlds", "mars.sdf")

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH", os.path.join(description, "models")
            ),
            SetEnvironmentVariable(
                "IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
                os.path.join(get_package_prefix("rover_sim"), "lib"),
            ),
            ExecuteProcess(
                cmd=["ign", "gazebo", "-s", "-r", "-v", "3", world], output="screen"
            ),
            ExecuteProcess(
                cmd=["ign", "launch", os.path.join(sim_share, "launch", "sim.ign")],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=BRIDGED_TOPICS,
                remappings=[("/world/mars/model/rover/joint_state", "/joint_states")],
                output="screen",
            ),
        ]
    )
