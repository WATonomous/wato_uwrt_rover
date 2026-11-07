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

// Simulation-only scoring: compares the rover's own pose estimate with Gazebo ground truth.
//   position_error       map-frame estimate (odometry + orbital fixes) vs truth
//   odom_position_error  odometry alone, anchored to the true start pose
//   drift_percent        odometry error per distance driven

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_core/grid.hpp"
#include "rover_msgs/msg/eval_stats.hpp"

using rover_core::Pose2;

namespace
{

Pose2 toPose2(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  return {p.position.x, p.position.y, std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
}

}  // namespace

class EvalNode : public rclcpp::Node
{
public:
  EvalNode()
  : Node("eval")
  , tf_buffer_(get_clock())
  , tf_listener_(tf_buffer_)
  {
    pub_ = create_publisher<rover_msgs::msg::EvalStats>("/eval/stats", 10);
    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/sim/ground_truth", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onTruth(*m); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { odom_ = toPose2(m->pose.pose); });
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::seconds(1), [this]() { report(); });
  }

private:
  void onTruth(const nav_msgs::msg::Odometry & msg)
  {
    const Pose2 truth = toPose2(msg.pose.pose);
    if (truth_) distance_ += std::hypot(truth.x - truth_->x, truth.y - truth_->y);
    truth_ = truth;
  }

  void report()
  {
    if (!truth_ || !odom_ || !tf_buffer_.canTransform("map", "odom", tf2::TimePointZero)) return;
    // Anchor odometry to the true pose once, so odom_position_error measures pure drift.
    if (!true_map_from_odom_) true_map_from_odom_ = rover_core::compose(*truth_, rover_core::inverse(*odom_));

    const auto tf = tf_buffer_.lookupTransform("map", "odom", tf2::TimePointZero);
    const auto & q = tf.transform.rotation;
    const Pose2 map_from_odom{
      tf.transform.translation.x,
      tf.transform.translation.y,
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
    const Pose2 estimate = rover_core::compose(map_from_odom, *odom_);
    const Pose2 odom_only = rover_core::compose(*true_map_from_odom_, *odom_);

    rover_msgs::msg::EvalStats stats;
    stats.header.stamp = now();
    stats.distance_traveled = static_cast<float>(distance_);
    stats.position_error = static_cast<float>(std::hypot(estimate.x - truth_->x, estimate.y - truth_->y));
    stats.heading_error = static_cast<float>(std::abs(rover_core::wrapAngle(estimate.yaw - truth_->yaw)));
    stats.odom_position_error = static_cast<float>(std::hypot(odom_only.x - truth_->x, odom_only.y - truth_->y));
    stats.drift_percent = distance_ > 1.0 ? static_cast<float>(100.0 * stats.odom_position_error / distance_) : 0.0f;
    pub_->publish(stats);
    if (++reports_ % 30 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "driven %.1f m | estimate error %.2f m, %.1f deg | odometry-only error %.2f m (%.1f%% of distance)",
        distance_,
        stats.position_error,
        stats.heading_error * 180.0 / M_PI,
        stats.odom_position_error,
        stats.drift_percent);
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::optional<Pose2> truth_;
  std::optional<Pose2> odom_;
  std::optional<Pose2> true_map_from_odom_;
  double distance_{0.0};
  int reports_{0};
  rclcpp::Publisher<rover_msgs::msg::EvalStats>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EvalNode>());
  rclcpp::shutdown();
  return 0;
}
