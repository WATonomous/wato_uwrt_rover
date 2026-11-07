#!/usr/bin/env python3
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
"""Publish the simulated terrain once, latched, as a Foxglove 3D model on /sim/terrain.

This is the simulator's true ground, for people watching in Foxglove. The rover never sees it.
It goes out a single time (transient local), so it costs nothing after a client connects.
"""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from foxglove_msgs.msg import ModelPrimitive, SceneEntity, SceneUpdate
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock


class TerrainViz(Node):
    def __init__(self):
        super().__init__("terrain_viz")
        path = os.path.join(
            get_package_share_directory("rover_description"), "viz", "terrain.glb"
        )
        with open(path, "rb") as f:
            self.model = f.read()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(SceneUpdate, "/sim/terrain", qos)
        # Stamp with sim time like every other topic: take one /clock message, then stop
        # listening so the node sits idle instead of waking at the clock rate.
        self.clock_sub = self.create_subscription(Clock, "/clock", self.publish_once, 1)

    def publish_once(self, clock):
        self.destroy_subscription(self.clock_sub)
        model = ModelPrimitive(media_type="model/gltf-binary", data=self.model)
        model.pose.orientation.w = 1.0
        model.scale.x = model.scale.y = model.scale.z = 1.0
        entity = SceneEntity(
            frame_id="map", id="terrain", frame_locked=True, models=[model]
        )
        entity.timestamp = clock.clock
        self.pub.publish(SceneUpdate(entities=[entity]))
        self.get_logger().info(
            f"Published terrain model ({len(self.model) / 1e6:.1f} MB) on /sim/terrain"
        )


def main():
    rclpy.init()
    rclpy.spin(TerrainViz())


if __name__ == "__main__":
    main()
