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

// Global localization without GPS. Odometry drifts, so every few metres the rover matches
// the terrain it has mapped against the orbital DEM (what JPL does on the ground with HiRISE
// images, and what Perseverance's onboard "Mars Global Localization" now does itself). An
// accepted match replaces map -> odom, which resets the accumulated drift.

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "rover_core/params.hpp"
#include "rover_core/terrain_match.hpp"
#include "rover_msgs/msg/height_map.hpp"
#include "rover_msgs/msg/localization_fix.hpp"

using rover_core::Grid;
using rover_core::Pose2;

class GlobalLocalizationNode : public rclcpp::Node
{
public:
  GlobalLocalizationNode()
  : Node("global_localization"),
    dem_(rover_core::loadOrbitalMap(rover_core::orbitalMapPath())),
    map_frame_(declare_parameter<std::string>("map_frame", "map")),
    odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
    trigger_distance_(declare_parameter("trigger_distance", 8.0)),
    retry_period_(declare_parameter("retry_period", 5.0)),
    tf_(this)
  {
    cfg_ = {
      declare_parameter("search_xy", 2.0),
      declare_parameter("search_yaw_deg", 3.0) * M_PI / 180.0,
      declare_parameter("coarse_xy_step", 0.25),
      declare_parameter("coarse_yaw_step_deg", 1.0) * M_PI / 180.0,
      declare_parameter("fine_xy_step", 0.05),
      declare_parameter("fine_yaw_step_deg", 0.25) * M_PI / 180.0,
      static_cast<int>(declare_parameter("min_points", 300)),
      declare_parameter("min_relief", 0.05),
      declare_parameter("min_score", 0.6),
      declare_parameter("min_peak_ratio", 2.0),
      declare_parameter("peak_separation", 1.0)};

    // Landing-site knowledge: the rover starts at the spawn point, with a deliberate error so
    // the first orbital match has something to correct. Odom axes are sun-aligned (east,
    // north), so map -> odom starts as a translation plus that small heading error.
    const auto rover = rover_core::loadRoverParams(rover_core::roverConfigPath());
    const auto world = rover_core::loadWorldParams(rover_core::worldConfigPath());
    const auto offset = declare_parameter<std::vector<double>>("initial_offset", {0.7, -0.5, 0.0});
    if (offset.size() != 3) throw std::invalid_argument("initial_offset must be [dx, dy, dyaw]");
    map_from_odom_ = {world.spawn.x + offset[0], world.spawn.y + offset[1], offset[2]};
    map_z_ = rover_core::bilinear(dem_, world.spawn.x, world.spawn.y) + rover.pivotHeight();

    fix_pub_ = create_publisher<rover_msgs::msg::LocalizationFix>("/global_localization/fix", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onOdom(*m); });
    map_sub_ = create_subscription<rover_msgs::msg::HeightMap>(
      "/enav/height_map", 1, [this](rover_msgs::msg::HeightMap::ConstSharedPtr m) { onHeightMap(*m); });
    srv_ = create_service<std_srvs::srv::Trigger>(
      "/global_localization/localize",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        force_ = true;
        res->success = true;
        res->message = "matching on the next height map";
      });
    const double rate = declare_parameter("tf_rate", 20.0);
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(1.0 / rate), [this]() { publishTf(); });
  }

private:
  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    if (last_odom_) distance_since_fix_ += std::hypot(p.x - last_odom_->x, p.y - last_odom_->y);
    last_odom_ = p;
  }

  void onHeightMap(const rover_msgs::msg::HeightMap & msg)
  {
    const double t = now().seconds();
    const bool due = force_ || !first_fix_ || distance_since_fix_ >= trigger_distance_;
    if (!due || t - last_attempt_ < retry_period_) return;
    last_attempt_ = t;
    force_ = false;

    Grid<float> elevation(msg.resolution, msg.origin_x, msg.origin_y, msg.width, msg.height, 0.0f);
    elevation.data = msg.elevation;
    const auto points = rover_core::terrainPoints(elevation, dem_.resolution);
    const auto start = std::chrono::steady_clock::now();
    const rover_core::MatchResult r = rover_core::matchTerrain(points, dem_, map_from_odom_, cfg_);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

    rover_msgs::msg::LocalizationFix fix;
    fix.header.stamp = msg.header.stamp;
    fix.header.frame_id = map_frame_;
    fix.accepted = r.accepted;
    fix.score = static_cast<float>(r.score);
    fix.runner_up = static_cast<float>(r.runner_up);
    fix.points = static_cast<uint32_t>(r.points);
    if (r.accepted) {
      fix.dx = static_cast<float>(r.map_from_odom.x - map_from_odom_.x);
      fix.dy = static_cast<float>(r.map_from_odom.y - map_from_odom_.y);
      fix.dyaw = static_cast<float>(rover_core::wrapAngle(r.map_from_odom.yaw - map_from_odom_.yaw));
      map_from_odom_ = r.map_from_odom;
      first_fix_ = true;
      distance_since_fix_ = 0.0;
    }
    fix_pub_->publish(fix);
    RCLCPP_INFO(
      get_logger(), "Orbital match %s: score %.3f runner-up %.3f, %d points, relief %.2f m, shift (%.2f, %.2f) m, %.2f deg, %.0f ms",
      r.accepted ? "ACCEPTED" : "rejected", r.score, r.runner_up, r.points, r.relief, fix.dx, fix.dy,
      fix.dyaw * 180.0 / M_PI, ms);
  }

  void publishTf()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = map_from_odom_.x;
    tf.transform.translation.y = map_from_odom_.y;
    tf.transform.translation.z = map_z_;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, map_from_odom_.yaw);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_.sendTransform(tf);
  }

  const Grid<float> dem_;
  const std::string map_frame_;
  const std::string odom_frame_;
  const double trigger_distance_;
  const double retry_period_;
  tf2_ros::TransformBroadcaster tf_;
  rover_core::MatchConfig cfg_;
  Pose2 map_from_odom_;
  double map_z_{0.0};
  bool first_fix_{false};
  bool force_{false};
  double distance_since_fix_{0.0};
  double last_attempt_{-1e9};
  std::optional<geometry_msgs::msg::Point> last_odom_;

  rclcpp::Publisher<rover_msgs::msg::LocalizationFix>::SharedPtr fix_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rover_msgs::msg::HeightMap>::SharedPtr map_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
