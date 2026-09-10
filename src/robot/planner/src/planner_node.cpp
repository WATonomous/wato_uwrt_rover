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

#include "planner/planner_node.hpp"

#include <string>

PlannerNode::PlannerNode()
: Node("planner")
, planner_(robot::PlannerCore(this->get_logger()))
{
  // load ROS2 yaml parameters
  processParameters();

  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    goal_topic_, 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  // Subscribe to odometry from /odom/filtered
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Must match state_manager's QoS exactly. A volatile subscriber will not
  // connect to the transient-local publisher, silently and with no error.
  const auto state_qos = rclcpp::QoS(1).transient_local().reliable();
  state_sub_ = this->create_subscription<rover_state_msgs::msg::RoverState>(
    "/rover_state", state_qos, std::bind(&PlannerNode::stateCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

  // Timer to check goal/timeout status periodically (500 ms)
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

  planner_.initPlanner(smoothing_factor_, iterations_);
}

void PlannerNode::processParameters()
{
  // Declare and get parameters
  this->declare_parameter<std::string>("map_topic", "/map");
  this->declare_parameter<std::string>("goal_topic", "/goal_pose");
  this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  this->declare_parameter<std::string>("path_topic", "/path");
  this->declare_parameter<double>("smoothing_factor", 0.2);
  this->declare_parameter<int>("iterations", 20);
  this->declare_parameter<double>("goal_tolerance", 0.3);
  this->declare_parameter<double>("plan_timeout_seconds", 10.0);

  map_topic_ = this->get_parameter("map_topic").as_string();
  goal_topic_ = this->get_parameter("goal_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  path_topic_ = this->get_parameter("path_topic").as_string();
  smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();
  iterations_ = this->get_parameter("iterations").as_int();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  plan_timeout_ = this->get_parameter("plan_timeout_seconds").as_double();
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ = msg;
  }

  // If we have an active goal, re-run plan
  if (active_goal_) {
    double elapsed = (now() - plan_start_time_).seconds();
    if (elapsed <= plan_timeout_) {
      RCLCPP_INFO(this->get_logger(), "Map updated => Replanning for current goal (time elapsed: %.2f).", elapsed);
      publishPath();
    }
  }
}

void PlannerNode::stateCallback(const rover_state_msgs::msg::RoverState::SharedPtr msg)
{
  const bool enabled = (msg->state == rover_state_msgs::msg::RoverState::CONTROL);

  if (enabled == autonomy_enabled_) {
    return;  // heartbeat, not a transition
  }
  autonomy_enabled_ = enabled;

  RCLCPP_INFO(
    this->get_logger(), "Autonomy %s (%s)", enabled ? "enabled" : "halted", msg->reason.c_str());

  if (!enabled) {
    // Drop the active goal so re-arming does not resume planning toward a
    // target chosen before the halt. Cleared directly rather than through
    // resetGoal(), which would publish a path while in WAIT.
    active_goal_ = false;
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal_msg)
{
  // Do not accept goals in WAIT; accepting one would commit state that
  // outlives the halt.
  if (!autonomy_enabled_) {
    return;
  }

  if (active_goal_) {
    RCLCPP_WARN(this->get_logger(), "Ignoring new goal; a goal is already active.");
    return;
  }

  if (!map_) {
    RCLCPP_WARN(this->get_logger(), "No costmap available yet. Cannot set goal.");
    return;
  }

  current_goal_ = *goal_msg;
  active_goal_ = true;
  plan_start_time_ = now();

  RCLCPP_INFO(this->get_logger(), "Received new goal: (%.2f, %.2f)", goal_msg->point.x, goal_msg->point.y);

  publishPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  // Store the latest odometry in (odom_x_, odom_y_)
  // For simplicity, ignoring orientation or z
  odom_x_ = odom_msg->pose.pose.position.x;
  odom_y_ = odom_msg->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::timerCallback()
{
  if (!active_goal_) {
    return;
  }

  // Check if we've timed out
  double elapsed = (now() - plan_start_time_).seconds();
  // if (elapsed > plan_timeout_) {
  //   RCLCPP_WARN(this->get_logger(), "Plan timed out after %.2f seconds. Resetting goal.", elapsed);
  //   resetGoal();
  //   return;
  // }

  // Check if we reached the goal
  double distance = sqrt(pow(odom_x_ - current_goal_.point.x, 2) + pow(odom_y_ - current_goal_.point.y, 2));
  if (distance < goal_tolerance_) {
    RCLCPP_WARN(this->get_logger(), "Plan succeeded! Elapsed Time: %.2f", elapsed);
    resetGoal();
    return;
  }
}

void PlannerNode::publishPath()
{
  // Blocked in WAIT. Guarded here rather than in the timer because every path
  // publish routes through this function - mapCallback and goalCallback both
  // call it directly.
  if (!autonomy_enabled_) {
    return;
  }

  if (!have_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry received yet. Cannot plan.");
    resetGoal();
    return;
  }

  // Use the robot's odometry as the start pose
  double start_world_x = odom_x_;
  double start_world_y = odom_y_;

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!planner_.planPath(start_world_x, start_world_y, current_goal_.point.x, current_goal_.point.y, map_)) {
      RCLCPP_ERROR(this->get_logger(), "Plan Failed.");
      resetGoal();
      return;
    }
  }

  nav_msgs::msg::Path path_msg = *planner_.getPath();
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = map_->header.frame_id;

  path_pub_->publish(path_msg);
}

void PlannerNode::resetGoal()
{
  active_goal_ = false;
  RCLCPP_INFO(this->get_logger(), "Resetting active goal.");

  // Publish an empty path, which should tell the robot to stop or have no path
  nav_msgs::msg::Path empty_path;
  empty_path.header.stamp = this->now();

  // Use the costmap frame if available; otherwise a default like "sim_world"
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (map_) {
      empty_path.header.frame_id = map_->header.frame_id;
    } else {
      empty_path.header.frame_id = "sim_world";
    }
  }

  // Publish the empty path
  path_pub_->publish(empty_path);

  RCLCPP_INFO(this->get_logger(), "Published empty path to stop the robot.");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
