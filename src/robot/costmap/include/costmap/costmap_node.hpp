// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <memory>
#include <string>

#include "costmap/costmap_core.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class CostmapNode : public rclcpp::Node
{
public:
  // Costmap Node constructor
  CostmapNode();

  // Retrieves all the parameters and their values in params.yaml
  void processParameters();

  // NOTE: neither callback is const any more. Both mutate costmap_, and the
  // core's update methods are non-const, so a const member function cannot
  // call them.
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  // Fetches both transforms the 2.5D path needs, AT THE GIVEN STAMP.
  //
  //   sensor_to_chassis  cloud frame -> chassis_frame_. Static in practice, but
  //                      looked up anyway so a URDF change needs no code change.
  //   chassis_to_world   chassis_frame_ -> world_frame_. Only the roll/pitch of
  //                      its rotation is used; its translation is discarded.
  //
  // Looked up at the cloud's stamp, not TimePointZero: at 30 Hz with the rover
  // pitching, the latest transform is a frame stale, which reintroduces exactly
  // the tilt error that leveling exists to remove.
  //
  // Returns false (and logs, throttled) if either lookup throws, so the callback
  // drops the frame rather than publishing a costmap built on a bad pose.
  bool lookupTransforms(
    const rclcpp::Time & stamp,
    const std::string & cloud_frame,
    geometry_msgs::msg::TransformStamped & sensor_to_chassis,
    geometry_msgs::msg::TransformStamped & chassis_to_world);

  robot::CostmapCore costmap_;

  // --- TF ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Body-fixed. Pitches and rolls with the rover, which is why heights cannot be
  // classified in it directly -- the core levels it first.
  //
  // This is also what the published costmap is stamped with. Strictly the grid
  // lives in the LEVELED frame, not this one, but nothing downstream reads the
  // frame_id (map_memory uses the odom pose and hardcodes "sim_world" on its own
  // output), and naming a frame that no one broadcasts would make Foxglove drop
  // the message. Cost of the shortcut: Foxglove renders the grid tilted by the
  // current roll/pitch. Only the rendering leans; the data is level.
  std::string chassis_frame_;

  // Any GRAVITY-ALIGNED frame -- "sim_world" in sim, "odom" from the EKF on the
  // real rover. Used only as a reference for how tilted the chassis currently is.
  std::string world_frame_;

  // How long lookupTransform may block waiting for a transform at the stamp.
  // Small on purpose: a late cloud is better dropped than delayed.
  double tf_timeout_ = 0.05;

  // --- ROS constructs ---
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  std::string laserscan_topic_;
  std::string pointcloud_topic_;
  std::string costmap_topic_;

  // --- Grid geometry ---
  double resolution_;
  int width_;
  int height_;
  geometry_msgs::msg::Pose origin_;
  double inflation_radius_;

  // Band and range tuning, filled from params in processParameters() and handed
  // to the core via setTerrainFilter(). One member rather than ten loose doubles.
  robot::TerrainFilter filter_;
};

#endif
