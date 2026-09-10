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

#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <mutex>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner/planner_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_state_msgs/msg/rover_state.hpp"

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

  void processParameters();

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal_msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void timerCallback();

  void publishPath();
  void resetGoal();

private:
  // Latches the rover's WAIT / CONTROL state published by state_manager.
  void stateCallback(const rover_state_msgs::msg::RoverState::SharedPtr msg);

  robot::PlannerCore planner_;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rover_state_msgs::msg::RoverState>::SharedPtr state_sub_;

  // Fail-safe default: blocked until state_manager says otherwise.
  bool autonomy_enabled_ = false;

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Costmap & Mutex
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::mutex map_mutex_;

  // Current goal
  geometry_msgs::msg::PointStamped current_goal_;
  bool active_goal_;
  rclcpp::Time plan_start_time_;

  // Robot odometry (x,y). For simplicity, ignoring orientation usage here.
  bool have_odom_;
  double odom_x_;
  double odom_y_;

  // Parameters
  std::string map_topic_;
  std::string goal_topic_;
  std::string odom_topic_;
  std::string path_topic_;

  double smoothing_factor_;
  int iterations_;

  double goal_tolerance_;
  double plan_timeout_;
};

#endif
