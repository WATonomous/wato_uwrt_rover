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

#include <string>

#include "costmap/costmap_core.hpp"
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

  // Given a laserscan, it will send it into CostmapCore for processing and then
  // retrieve the costmap
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const;

  // Given a point cloud, it will send it into CostmapCore for processing and then
  // retrieve the costmap
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;

private:
  //for 2.5D mapping
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string target_frame_;   // e.g. "sim_world" or "robot/chassis"

  robot::CostmapCore costmap_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  std::string laserscan_topic_;
  std::string pointcloud_topic_;
  std::string costmap_topic_;

  double resolution_;
  int width_;
  int height_;
  geometry_msgs::msg::Pose origin_;
  double inflation_radius_;
};

#endif
