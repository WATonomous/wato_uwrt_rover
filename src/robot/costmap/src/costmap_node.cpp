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
  costmap_.setTerrainFilter(filter_);

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
  //additional parameters
  this->declare_parameter<std::string>("chassis_frame", "robot/chassis");
  this->declare_parameter<std::string>("world_frame", "sim_world");
  this->declare_parameter<double>("tf_timeout", 0.05);

  this->declare_parameter<double>("terrain.min_range", 0.2);
  this->declare_parameter<double>("terrain.max_range", 5.0);
  this->declare_parameter<double>("terrain.band_low", 0.10);
  this->declare_parameter<double>("terrain.band_high", 0.60);
  this->declare_parameter<double>("terrain.ground_floor", -0.50);
  this->declare_parameter<double>("terrain.band_low_slack", 0.0);
  this->declare_parameter<int>("terrain.min_points_obstacle", 3);
  this->declare_parameter<int>("terrain.min_points_free", 6);
  this->declare_parameter<int>("terrain.point_stride", 1);

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
  //additional parameters stored
  chassis_frame_ = this->get_parameter("chassis_frame").as_string();
  world_frame_   = this->get_parameter("world_frame").as_string();
  tf_timeout_    = this->get_parameter("tf_timeout").as_double();

  filter_.min_range           = this->get_parameter("terrain.min_range").as_double();
  filter_.max_range           = this->get_parameter("terrain.max_range").as_double();
  filter_.band_low            = this->get_parameter("terrain.band_low").as_double();
  filter_.band_high           = this->get_parameter("terrain.band_high").as_double();
  filter_.ground_floor        = this->get_parameter("terrain.ground_floor").as_double();
  filter_.band_low_slack      = this->get_parameter("terrain.band_low_slack").as_double();
  filter_.min_points_obstacle = this->get_parameter("terrain.min_points_obstacle").as_int();
  filter_.min_points_free     = this->get_parameter("terrain.min_points_free").as_int();
  filter_.point_stride        = this->get_parameter("terrain.point_stride").as_int();

}

void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
  // Update the costmap according to the laser scan
  costmap_.updateCostmap(msg);
  // publish the costmap
  nav_msgs::msg::OccupancyGrid costmap_msg = *costmap_.getCostmapData();
  costmap_msg.header = msg->header;
  costmap_pub_->publish(costmap_msg);
}

bool CostmapNode::lookupTransforms(
  const rclcpp::Time & stamp,
  const std::string & cloud_frame,
  geometry_msgs::msg::TransformStamped & sensor_to_chassis,
  geometry_msgs::msg::TransformStamped & chassis_to_world)
{
  const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(tf_timeout_);
  try{
    //look up the transforms at specific timestamp
      sensor_to_chassis = tf_buffer_->lookupTransform(chassis_frame_, cloud_frame, stamp, timeout);
      chassis_to_world = tf_buffer_->lookupTransform(world_frame_, chassis_frame_, stamp, timeout);
  } catch(const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "TF lookup failed, dropping cloud: %s", ex.what());
    return false;
  }
  return true;
}

void CostmapNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped sensor_to_chassis;
  geometry_msgs::msg::TransformStamped chassis_to_world;
  if(!lookupTransforms(msg->header.stamp, msg->header.frame_id,sensor_to_chassis,chassis_to_world)){
    return;
  }
  // Update the costmap according to the point cloud
  costmap_.updateCostmapFromPointCloud(msg,sensor_to_chassis,chassis_to_world);
  // publish the costmap
  nav_msgs::msg::OccupancyGrid costmap_msg = *costmap_.getCostmapData();
  //keep the same timestamp because we need it to align with memorymap
  costmap_msg.header.stamp = msg->header.stamp;
  //change frame id because not in camera frame anymore
  costmap_msg.header.frame_id = chassis_frame_;
  costmap_pub_->publish(costmap_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
