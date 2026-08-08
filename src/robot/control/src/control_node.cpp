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

#include "control/control_node.hpp"

#include <string>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

ControlNode::ControlNode()
: Node("control")
, control_(robot::ControlCore(this->get_logger()))
{
  processParameters();

  // Initialize subscribers and publisher
  path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
    path_topic_, 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(control_period_ms_), std::bind(&ControlNode::timerCallback, this));

  control_.initControlCore(kp_, ki_, kd_, max_steering_angle_, linear_velocity_);
}

void ControlNode::processParameters()
{
  // Declare all ROS2 Parameters
  this->declare_parameter<std::string>("path_topic", "/path");
  this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<int>("control_period_ms", 100);
  this->declare_parameter<double>("kp", 1.0);
  this->declare_parameter<double>("ki", 0.6);
  this->declare_parameter<double>("kd", 0.4);
  this->declare_parameter<double>("max_steering_angle", 1.5);
  this->declare_parameter<double>("linear_velocity", 1.5);

  // Retrieve parameters and store them in member variables
  path_topic_ = this->get_parameter("path_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
  control_period_ms_ = this->get_parameter("control_period_ms").as_int();
  kp_ = this->get_parameter("kp").as_double();
  ki_ = this->get_parameter("ki").as_double();
  kd_ = this->get_parameter("kd").as_double();
  max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
  linear_velocity_ = this->get_parameter("linear_velocity").as_double();
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  control_.updatePath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // Get robot's orientation (yaw) from quaternion using utility function
  robot_theta_ = quaternionToYaw(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
}

void ControlNode::followPath()
{
  if (control_.isPathEmpty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Path is empty. Waiting for new path.");
    // Publish stop command to halt the robot
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_publisher_->publish(stop_cmd);
    return;
  }

  // Calculate control commands
  double dt = control_period_ms_ / 1000.0;
  geometry_msgs::msg::Twist cmd_vel = control_.calculateControlCommand(robot_x_, robot_y_, robot_theta_, dt);
  cmd_vel_publisher_->publish(cmd_vel);
}

void ControlNode::timerCallback()
{
  followPath();
}

double ControlNode::quaternionToYaw(double x, double y, double z, double w)
{
  // Using tf2 to convert to RPY
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
