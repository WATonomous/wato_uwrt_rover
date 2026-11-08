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

// Onboard state estimation, Perseverance style (no GPS):
//   attitude  = gyro integration corrected by gravity (roll/pitch) and the sun (heading)
//   position  = wheel odometry, replaced by visual odometry wherever VO succeeded
//   slip      = 1 - VO distance / wheel distance
// Publishes odom -> base_link. Global corrections (map -> odom) come from global_localization.

#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rover_core/params.hpp"
#include "rover_core/state_estimation.hpp"
#include "rover_msgs/msg/slip_estimate.hpp"
#include "rover_msgs/msg/visual_odometry.hpp"

namespace
{

using rover_core::AttitudeFilter;
using rover_core::kWheelJoints;

constexpr double kWheelMotionEpsilon = 1e-4;  // rad between joint_states messages
constexpr double kStationaryTime = 0.3;  // s without wheel motion before VO is ignored

Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3 & v)
{
  return {v.x, v.y, v.z};
}

}  // namespace

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode()
  : Node("localization")
  , rover_(rover_core::loadRoverParams(rover_core::roverConfigPath()))
  , sun_world_(rover_core::loadWorldParams(rover_core::worldConfigPath()).sunDirection())
  , attitude_(AttitudeFilter::Gains{
      declare_parameter("gravity", 3.721),
      declare_parameter("kp_gravity", 1.0),
      declare_parameter("ki_gravity", 0.02),
      declare_parameter("k_sun", 0.5),
      declare_parameter("ki_sun", 0.2)})
  , slip_(
      declare_parameter("slip_window", 0.6),
      declare_parameter("slip_min_distance", 0.25),
      declare_parameter("slip_max_age", 6.0))
  , odom_frame_(declare_parameter<std::string>("odom_frame", "odom"))
  , base_frame_(declare_parameter<std::string>("base_frame", "base_link"))
  , tf_(this)
  {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    slip_pub_ = create_publisher<rover_msgs::msg::SlipEstimate>("/localization/slip", 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { onImu(*m); });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        onJoints(*m);
      });
    sun_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/sun_sensor", 10, [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr m) { onSun(*m); });
    vo_sub_ = create_subscription<rover_msgs::msg::VisualOdometry>(
      "/visual_odometry", 10, [this](rover_msgs::msg::VisualOdometry::ConstSharedPtr m) { onVisualOdometry(*m); });
  }

private:
  void onImu(const sensor_msgs::msg::Imu & msg)
  {
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    const Eigen::Vector3d gyro = toEigen(msg.angular_velocity);
    const Eigen::Vector3d accel = toEigen(msg.linear_acceleration);

    if (!attitude_.initialized()) {
      // Wait at rest for the first sun fix, averaging gravity meanwhile.
      accel_sum_ += accel;
      ++accel_count_;
      if (sun_body_ && accel_count_ >= 25) {
        attitude_.initialize(accel_sum_ / accel_count_, *sun_body_, sun_world_);
        last_imu_ = stamp;
        wheel_distance_ = 0.0;
        dead_reckoning_.addWheelMotion(stamp, 0.0, attitude_.orientation());
        RCLCPP_INFO(get_logger(), "Attitude initialized from gravity and the sun (TRIAD)");
      }
      return;
    }
    const double dt = stamp - last_imu_;
    last_imu_ = stamp;
    if (dt <= 0.0 || dt > 0.2) return;

    attitude_.propagate(gyro, accel, dt);
    dead_reckoning_.addWheelMotion(stamp, wheel_distance_, attitude_.orientation());
    const double speed = wheel_distance_ / dt;
    wheel_distance_ = 0.0;
    publish(msg.header.stamp, speed, gyro - attitude_.gyroBias());
  }

  void onJoints(const sensor_msgs::msg::JointState & msg)
  {
    if (wheel_index_[0] < 0) {
      for (size_t k = 0; k < kWheelJoints.size(); ++k) {
        for (size_t i = 0; i < msg.name.size(); ++i) {
          if (msg.name[i] == kWheelJoints[k]) wheel_index_[k] = static_cast<int>(i);
        }
        if (wheel_index_[k] < 0) throw std::runtime_error("joint_states is missing " + kWheelJoints[k]);
      }
    }
    std::array<double, 6> position;
    for (size_t k = 0; k < 6; ++k) position[k] = msg.position[wheel_index_[k]];
    if (last_wheel_) {
      double sum = 0.0;
      double largest = 0.0;
      for (size_t k = 0; k < 6; ++k) {
        sum += position[k] - (*last_wheel_)[k];
        largest = std::max(largest, std::abs(position[k] - (*last_wheel_)[k]));
      }
      if (largest > kWheelMotionEpsilon) last_wheel_motion_ = rclcpp::Time(msg.header.stamp).seconds();
      // Ignore motion before the attitude (and so the odom frame) exists.
      if (attitude_.initialized()) wheel_distance_ += sum / 6.0 * rover_.wheel_radius;
    }
    last_wheel_ = position;
  }

  void onSun(const geometry_msgs::msg::Vector3Stamped & msg)
  {
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    sun_body_ = toEigen(msg.vector).normalized();
    if (attitude_.initialized() && stamp > last_sun_) {
      attitude_.correctSun(*sun_body_, sun_world_, last_sun_ > 0.0 ? stamp - last_sun_ : 1.0);
    }
    last_sun_ = stamp;
  }

  void onVisualOdometry(const rover_msgs::msg::VisualOdometry & msg)
  {
    rover_msgs::msg::SlipEstimate out;
    out.header = msg.header;
    const double t1 = rclcpp::Time(msg.header.stamp).seconds();
    // Zero-velocity update: with every wheel still, the rover is not moving, so VO noise must
    // not be integrated into the position.
    const bool stationary = t1 - last_wheel_motion_ > kStationaryTime;
    if (msg.valid && attitude_.initialized() && !stationary) {
      const auto & p = msg.delta.position;
      const auto update =
        dead_reckoning_.applyVisualOdometry(rclcpp::Time(msg.previous_stamp).seconds(), t1, {p.x, p.y, p.z});
      if (update) slip_.add(t1, update->wheel_distance, update->vo_distance);
    }
    const std::optional<double> slip = slip_.slip(t1);
    out.valid = slip.has_value();
    out.slip = static_cast<float>(slip.value_or(0.0));
    out.wheel_distance = static_cast<float>(slip_.wheelDistance());
    out.vo_distance = static_cast<float>(slip_.voDistance());
    slip_pub_->publish(out);
  }

  void publish(const builtin_interfaces::msg::Time & stamp, double speed, const Eigen::Vector3d & angular)
  {
    const Eigen::Vector3d & p = dead_reckoning_.position();
    const Eigen::Quaterniond & q = attitude_.orientation();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = p.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.pose.covariance[0] = odom.pose.covariance[7] = dead_reckoning_.positionVariance();
    odom.pose.covariance[35] = 3e-4;
    odom.twist.twist.linear.x = speed;
    odom.twist.twist.angular.x = angular.x();
    odom.twist.twist.angular.y = angular.y();
    odom.twist.twist.angular.z = angular.z();
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = p.x();
    tf.transform.translation.y = p.y();
    tf.transform.translation.z = p.z();
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_.sendTransform(tf);
  }

  const rover_core::RoverParams rover_;
  const Eigen::Vector3d sun_world_;
  AttitudeFilter attitude_;
  rover_core::DeadReckoning dead_reckoning_;
  rover_core::SlipEstimator slip_;
  const std::string odom_frame_;
  const std::string base_frame_;
  tf2_ros::TransformBroadcaster tf_;

  Eigen::Vector3d accel_sum_{Eigen::Vector3d::Zero()};
  int accel_count_{0};
  std::optional<Eigen::Vector3d> sun_body_;
  double last_imu_{0.0};
  double last_sun_{0.0};
  double last_wheel_motion_{0.0};
  double wheel_distance_{0.0};
  std::array<int, 6> wheel_index_{-1, -1, -1, -1, -1, -1};
  std::optional<std::array<double, 6>> last_wheel_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<rover_msgs::msg::SlipEstimate>::SharedPtr slip_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sun_sub_;
  rclcpp::Subscription<rover_msgs::msg::VisualOdometry>::SharedPtr vo_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
