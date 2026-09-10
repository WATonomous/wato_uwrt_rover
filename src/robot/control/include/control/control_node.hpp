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

#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <string>

#include "control/control_core.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_state_msgs/msg/rover_state.hpp"

class ControlNode : public rclcpp::Node
{
public:
  ControlNode();

  // Read and load in ROS2 parameters
  void processParameters();

  // Utility: Convert quaternion to yaw
  double quaternionToYaw(double x, double y, double z, double w);

  // Callback for path
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  // Callback for odometry
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Main loop to continuously follow the path
  void followPath();

  // Timer callback
  void timerCallback();

private:
  // Latches the rover's WAIT / CONTROL state published by state_manager.
  void stateCallback(const rover_state_msgs::msg::RoverState::SharedPtr msg);

  robot::ControlCore control_;

  // Subscriber and Publisher
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<rover_state_msgs::msg::RoverState>::SharedPtr state_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  // Fail-safe default: blocked until state_manager says otherwise.
  bool autonomy_enabled_ = false;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Path and robot state
  double robot_x_;
  double robot_y_;
  double robot_theta_;

  // ROS2 params
  std::string path_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;

  int control_period_ms_;
  double kp_;
  double ki_;
  double kd_;

  double max_steering_angle_;
  double linear_velocity_;
};

#endif
