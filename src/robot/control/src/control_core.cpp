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

#include "control/control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger & logger)
: path_(nav_msgs::msg::Path())
, logger_(logger)
, prev_error_(0.0)
, integral_error_(0.0)
{}

void ControlCore::initControlCore(double kp, double ki, double kd, double max_steering_angle, double linear_velocity)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  max_steering_angle_ = max_steering_angle;
  linear_velocity_ = linear_velocity;
}

void ControlCore::updatePath(nav_msgs::msg::Path path)
{
  RCLCPP_INFO(logger_, "Path Updated");
  path_ = path;
  // Reset PID state when path changes
  prev_error_ = 0.0;
  integral_error_ = 0.0;
}

bool ControlCore::isPathEmpty()
{
  return path_.poses.empty();
}

geometry_msgs::msg::Twist ControlCore::calculateControlCommand(
  double robot_x, double robot_y, double robot_theta, double dt)
{
  geometry_msgs::msg::Twist twist;

  if (path_.poses.empty()) {
    return twist;
  }

  unsigned int closest_index = findClosestPoint(robot_x, robot_y);

  // Determine path segment for CTE calculation
  // We need two points to define a line.
  double p1_x, p1_y, p2_x, p2_y;

  if (closest_index < path_.poses.size() - 1) {
    p1_x = path_.poses[closest_index].pose.position.x;
    p1_y = path_.poses[closest_index].pose.position.y;
    p2_x = path_.poses[closest_index + 1].pose.position.x;
    p2_y = path_.poses[closest_index + 1].pose.position.y;
  } else if (path_.poses.size() > 1) {
    // If closest is the last point, use the previous segment
    p1_x = path_.poses[closest_index - 1].pose.position.x;
    p1_y = path_.poses[closest_index - 1].pose.position.y;
    p2_x = path_.poses[closest_index].pose.position.x;
    p2_y = path_.poses[closest_index].pose.position.y;
  } else {
    // Only one point in path, go to it
    p1_x = path_.poses[0].pose.position.x;
    p1_y = path_.poses[0].pose.position.y;
    // Treat as point at same location, just use distance logic or stop
    // For now, create a dummy small segment in direction of robot heading or just return 0
    // Simplest: treat p2 as p1 + some epsilon in x
    p2_x = p1_x + 0.001;
    p2_y = p1_y;
  }

  // Calculate Cross Track Error (CTE)
  // Vector V from p1 to p2
  double vx = p2_x - p1_x;
  double vy = p2_y - p1_y;

  // Normalize V
  double v_len = std::sqrt(vx * vx + vy * vy);
  if (v_len > 0) {
    vx /= v_len;
    vy /= v_len;
  }

  // Vector R from p1 to robot
  double rx = robot_x - p1_x;
  double ry = robot_y - p1_y;

  // Signed distance (cross product of normalized V and R) in 2D (z component)
  // if V is along X, R is along Y, cross is positive (Left).
  double cross_product = vx * ry - vy * rx;

  double error = cross_product;  // Positive if robot is to the left of the path

  // PID Calculation
  // Integral
  integral_error_ += error * dt;

  // Derivative
  double derivative = 0.0;
  if (dt > 0) {
    derivative = (error - prev_error_) / dt;
  }

  // PID Output
  // We want to steer OPPOSITE to the error.
  // If error is positive (left), we want negative angular velocity (turn right).
  double output = -(kp_ * error + ki_ * integral_error_ + kd_ * derivative);

  prev_error_ = error;

  // Clamp output to max steering/angular velocity
  if (output > max_steering_angle_) output = max_steering_angle_;
  if (output < -max_steering_angle_) output = -max_steering_angle_;

  // Set linear velocity
  // Logic: slow down if error is high or turning sharp?
  // For now, constant velocity unless very sharp turn required?
  // Let's stick to the previous logic: if turning hard, slow down?
  // Or just constant linear velocity as per prompt "make sure the robot follows the path"

  // Just use the configured linear velocity
  double speed_factor =
    1.0 - (std::abs(output) / max_steering_angle_ * 0.5);  // during turns slows down to avoid overshoot
  twist.linear.x = linear_velocity_ * speed_factor;
  twist.angular.z = output;

  return twist;
}

unsigned int ControlCore::findClosestPoint(double robot_x, double robot_y)
{
  double min_distance = std::numeric_limits<double>::max();
  unsigned int closest_index = 0;

  for (size_t i = 0; i < path_.poses.size(); ++i) {
    double dx = path_.poses[i].pose.position.x - robot_x;
    double dy = path_.poses[i].pose.position.y - robot_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    }
  }
  return closest_index;
}

}  // namespace robot
