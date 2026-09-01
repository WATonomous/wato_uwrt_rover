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

#include "costmap/costmap_node.hpp"

#include <chrono>
#include <memory>
#include <string>

CostmapNode::CostmapNode()
: Node("costmap")
, costmap_(robot::CostmapCore(this->get_logger()))
{
  // load ROS2 yaml parameters
  processParameters();

  //transform buffer(queue of past states) and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // Subscribe to point cloud from RGBD camera
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  pointcloud_topic_, 10, std::bind(&CostmapNode::pointCloudCallback, this, std::placeholders::_1));

  // Keep laser scan subscription for backwards compatibility (optional)
  // laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
  //   laserscan_topic_, 10,
  //   std::bind(
  //     &CostmapNode::laserScanCallback, this,
  //     std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Initialized ROS Constructs");
  RCLCPP_INFO(this->get_logger(), "Subscribed to point cloud topic: %s", pointcloud_topic_.c_str());

  costmap_.initCostmap(resolution_, width_, height_, origin_, inflation_radius_);

  RCLCPP_INFO(this->get_logger(), "Initialized Costmap Core");
}
void CostmapNode::processParameters()
{
  // Declare all ROS2 Parameters
  this->declare_parameter<std::string>("laserscan_topic", "/lidar");
  this->declare_parameter<std::string>("pointcloud_topic", "/camera/points");
  this->declare_parameter<std::string>("costmap_topic", "/costmap");
  this->declare_parameter<double>("costmap.resolution", 0.1);
  this->declare_parameter<int>("costmap.width", 100);
  this->declare_parameter<int>("costmap.height", 100);
  this->declare_parameter<double>("costmap.origin.position.x", -5.0);
  this->declare_parameter<double>("costmap.origin.position.y", -5.0);
  this->declare_parameter<double>("costmap.origin.orientation.w", 1.0);
  this->declare_parameter<double>("costmap.inflation_radius", 1.0);

  // Retrieve parameters and store them in member variables
  laserscan_topic_ = this->get_parameter("laserscan_topic").as_string();
  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  costmap_topic_ = this->get_parameter("costmap_topic").as_string();
  resolution_ = this->get_parameter("costmap.resolution").as_double();
  width_ = this->get_parameter("costmap.width").as_int();
  height_ = this->get_parameter("costmap.height").as_int();
  origin_.position.x = this->get_parameter("costmap.origin.position.x").as_double();
  origin_.position.y = this->get_parameter("costmap.origin.position.y").as_double();
  origin_.orientation.w = this->get_parameter("costmap.origin.orientation.w").as_double();
  inflation_radius_ = this->get_parameter("costmap.inflation_radius").as_double();
}

void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
  // Update the costmap according to the laser scan
  costmap_.updateCostmap(msg);
  // publish the costmap
  nav_msgs::msg::OccupancyGrid costmap_msg = *costmap_.getCostmapData();
  costmap_msg.header = msg->header;
  costmap_pub_->publish(costmap_msg);
}

void CostmapNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
  // Update the costmap according to the point cloud
  costmap_.updateCostmapFromPointCloud(msg);
  // publish the costmap
  nav_msgs::msg::OccupancyGrid costmap_msg = *costmap_.getCostmapData();
  costmap_msg.header = msg->header;
  costmap_pub_->publish(costmap_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
