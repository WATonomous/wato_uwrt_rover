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

// Simulated sun sensor. Perseverance finds the sun in a navcam image and compares it with the
// ephemeris to get absolute heading; here the sun direction in the rover frame is computed
// from the simulator's true attitude, with angular noise. Simulation only.

#include <chrono>
#include <memory>
#include <optional>
#include <random>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_core/params.hpp"

class SunSensorNode : public rclcpp::Node
{
public:
  SunSensorNode()
  : Node("sun_sensor"),
    sun_world_(rover_core::loadWorldParams(rover_core::worldConfigPath()).sunDirection()),
    noise_(0.0, declare_parameter("noise", 0.005)),
    rng_(declare_parameter("seed", 42))
  {
    pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/sun_sensor", 10);
    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/sim/ground_truth", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { truth_ = *m; });
    const double rate = declare_parameter("rate", 1.0);
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate), [this]() { publish(); });
  }

private:
  void publish()
  {
    if (!truth_ || sun_world_.z() <= 0.0) return;  // no sun below the horizon
    const auto & o = truth_->pose.pose.orientation;
    const Eigen::Quaterniond world_from_body(o.w, o.x, o.y, o.z);
    const Eigen::Vector3d tilt(noise_(rng_), noise_(rng_), noise_(rng_));
    const Eigen::Vector3d sun_body =
      Eigen::AngleAxisd(tilt.norm(), tilt.normalized()) * (world_from_body.conjugate() * sun_world_);
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = truth_->header.stamp;
    msg.header.frame_id = "base_link";
    msg.vector.x = sun_body.x();
    msg.vector.y = sun_body.y();
    msg.vector.z = sun_body.z();
    pub_->publish(msg);
  }

  const Eigen::Vector3d sun_world_;
  std::normal_distribution<double> noise_;
  std::mt19937 rng_;
  std::optional<nav_msgs::msg::Odometry> truth_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SunSensorNode>());
  rclcpp::shutdown();
  return 0;
}
