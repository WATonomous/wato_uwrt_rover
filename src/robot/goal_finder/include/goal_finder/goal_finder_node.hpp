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

#ifndef GOAL_FINDER_NODE_HPP
#define GOAL_FINDER_NODE_HPP

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_state_msgs/msg/rover_state.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "goal_finder/goal_finder_core.hpp"

class GoalFinderNode : public rclcpp::Node
{
public:
  GoalFinderNode();

private:
  enum class State
  {
    WAITING_FOR_OBJECT,
    SEARCHING_FOR_OBJECT,
    MOVING_TO_OBJECT
  };
  // Substate of CONTROL. Reset to WAITING_FOR_OBJECT whenever autonomy is
  // halted, so re-arming does not resume a search or an approach begun before
  // the halt.
  State state_ = State::WAITING_FOR_OBJECT;

  // Fail-safe default: blocked until state_manager says otherwise.
  bool autonomy_enabled_ = false;

  bool objectDetected_ = false;
  double max_angle_ = 45.0;  // assuming 90 degree FOV
  double search_angle_ = 0.0;
  int image_width_ = 640;
  int desired_class_ = 0;  // class of the object we want to detect
  double goal_tolerance = 0.3;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr object_sub_;
  rclcpp::Subscription<rover_state_msgs::msg::RoverState>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rotate_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  //NEED A SUBSCRIBE THAT TELLS US THE DESIRED OBJECT

  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::Odometry robot_pose_;
  vision_msgs::msg::Detection2DArray object_message_;
  geometry_msgs::msg::PointStamped goal_point_;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void objectCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void stateCallback(const rover_state_msgs::msg::RoverState::SharedPtr msg);
  void timerCallback();
  double cameraToAngle(const vision_msgs::msg::BoundingBox2D & bbox);
  bool findGoalPoint(double angle);  // returns true if an occupied cell was found and fills goal_point_
};

#endif  // GOAL_FINDER_NODE_HPP
