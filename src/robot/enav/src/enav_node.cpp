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

// ENav: Perseverance's autonomous navigator.
//   1. Navcam depth -> rover-centric 2.5D height map (odom frame).
//   2. Route costs in the map frame: orbital slope cost + step heights the rover has seen
//      + keep-out zones; Dijkstra from the goal gives cost-to-go for every cell.
//   3. A fixed tree of candidate paths (turn in place + two arcs) is ranked cheaply by
//      route cost + cost-to-go at the endpoint.
//   4. ACE checks candidates best-first; the first safe one is published. Mobility drives its
//      first arc while ENav is already planning the next cycle ("thinking while driving").

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rover_core/height_map.hpp"
#include "rover_core/params.hpp"
#include "rover_core/planner.hpp"
#include "rover_msgs/msg/enav_plan.hpp"
#include "rover_msgs/msg/height_map.hpp"
#include "rover_msgs/msg/mobility_status.hpp"

using rover_core::Grid;
using rover_core::Pose2;

namespace
{

constexpr int kMapPublishCycles = 4;  // maps for Foxglove every 4 planning cycles (0.5 Hz)
constexpr int kCloudStride = 3;  // height-map cells between Foxglove cloud points
constexpr size_t kMaxRejectedShown = 25;

struct Rgba
{
  float r;
  float g;
  float b;
  float a;
};

constexpr Rgba kWhite{1.0f, 1.0f, 1.0f, 1.0f};
constexpr Rgba kRejected{1.0f, 0.15f, 0.15f, 0.9f};
constexpr Rgba kChosen{0.2f, 1.0f, 0.3f, 0.9f};
constexpr Rgba kRoute{0.1f, 0.85f, 1.0f, 0.9f};
constexpr Rgba kWaypointDone{0.5f, 0.5f, 0.5f, 0.5f};
constexpr Rgba kWaypointCurrent{1.0f, 0.2f, 0.9f, 0.9f};
constexpr Rgba kWaypointUpcoming{1.0f, 0.6f, 0.1f, 0.9f};

Pose2 toPose2(const geometry_msgs::msg::TransformStamped & tf)
{
  return {tf.transform.translation.x, tf.transform.translation.y, rover_core::yawOf(tf.transform.rotation)};
}

visualization_msgs::msg::Marker marker(const std::string & frame, const std::string & ns, int type, const Rgba & c)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame;
  m.ns = ns;
  m.type = type;
  m.color.r = c.r;
  m.color.g = c.g;
  m.color.b = c.b;
  m.color.a = c.a;
  m.pose.orientation.w = 1.0;
  return m;
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

nav_msgs::msg::OccupancyGrid toOccupancy(const Grid<float> & g, double scale, const std::string & frame)
{
  nav_msgs::msg::OccupancyGrid out;
  out.header.frame_id = frame;
  out.info.resolution = static_cast<float>(g.resolution);
  out.info.width = g.width;
  out.info.height = g.height;
  out.info.origin.position.x = g.origin_x;
  out.info.origin.position.y = g.origin_y;
  out.info.origin.orientation.w = 1.0;
  out.data.resize(g.data.size());
  for (size_t k = 0; k < g.data.size(); ++k) {
    const float v = g.data[k];
    out.data[k] = std::isnan(v) ? -1 : std::isinf(v) ? 100 : static_cast<int8_t>(std::min(99.0, v / scale * 99.0));
  }
  return out;
}

}  // namespace

class EnavNode : public rclcpp::Node
{
public:
  EnavNode()
  : Node("enav")
  , rover_(rover_core::loadRoverParams(rover_core::roverConfigPath()))
  , base_from_optical_(rover_.navcamOptical())
  , height_map_(declare_parameter("map_resolution", 0.05), declare_parameter("map_cells", 240))
  , tf_buffer_(get_clock())
  , tf_listener_(tf_buffer_)
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    recenter_margin_ = declare_parameter("recenter_margin", 3.0);
    depth_stride_ = declare_parameter("depth_stride", 1);
    max_range_ = declare_parameter("max_range", 7.0);
    footprint_radius_ = declare_parameter("footprint_radius", 1.2);
    goal_tolerance_ = declare_parameter("goal_tolerance", 1.0);
    max_ace_paths_ = declare_parameter("max_ace_paths", 400);
    hazard_radius_ = declare_parameter("hazard_radius", 1.5);
    cost_ = {
      declare_parameter("slope_free_deg", 8.0) * M_PI / 180.0,
      declare_parameter("slope_max_deg", 22.0) * M_PI / 180.0,
      declare_parameter("steep_cost", 3.0),
      declare_parameter("step_free", 0.12),
      declare_parameter("step_lethal", 0.50),
      declare_parameter("cost_weight", 4.0)};
    weights_ = {
      declare_parameter("turn_weight", 0.8),
      declare_parameter("unknown_weight", 0.2),
      declare_parameter("heading_change_weight", 0.3)};
    const rover_core::PathTreeConfig tree{
      degToRad(declare_parameter<std::vector<double>>(
        "tree_turns_deg", {0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 120.0, -120.0, 150.0, -150.0, 180.0})),
      declare_parameter<std::vector<double>>("tree_curvatures", {0.0, 0.2, -0.2, 0.5, -0.5, 1.0, -1.0, 1.6, -1.6}),
      declare_parameter("tree_segment_length", 1.2),
      declare_parameter("tree_step", 0.1),
      declare_parameter("tree_turn_step_deg", 15.0) * M_PI / 180.0};
    tree_ = rover_core::buildPathTree(tree);

    dem_ = rover_core::loadOrbitalMap(rover_core::orbitalMapPath());
    orbital_cost_ = rover_core::orbitalCost(dem_, cost_);
    observed_cost_ =
      Grid<float>(dem_.resolution, dem_.origin_x, dem_.origin_y, dem_.width, dem_.height, rover_core::kUnknown);
    keep_out_ = Grid<float>(dem_.resolution, dem_.origin_x, dem_.origin_y, dem_.width, dem_.height, 0.0f);
    const auto keep_out = declare_parameter<std::vector<double>>("keep_out", std::vector<double>{});
    if (keep_out.size() % 3 != 0) throw std::invalid_argument("keep_out must be [x, y, radius, ...]");
    for (size_t k = 0; k < keep_out.size(); k += 3) {
      rover_core::stampKeepOut(keep_out_, keep_out[k], keep_out[k + 1], keep_out[k + 2]);
    }
    const auto waypoints = declare_parameter<std::vector<double>>("waypoints", std::vector<double>{});
    if (waypoints.size() % 2 != 0) throw std::invalid_argument("waypoints must be [x, y, ...]");
    for (size_t k = 0; k < waypoints.size(); k += 2) waypoints_.push_back({waypoints[k], waypoints[k + 1]});
    RCLCPP_INFO(get_logger(), "Path tree: %zu candidates, %zu waypoints", tree_.size(), waypoints_.size());

    plan_pub_ = create_publisher<rover_msgs::msg::EnavPlan>("/enav/plan", 10);
    height_pub_ = create_publisher<rover_msgs::msg::HeightMap>("/enav/height_map", 1);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/enav/height_cloud", 1);
    cost_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/enav/cost", 1);
    ctg_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/enav/cost_to_go", 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/enav/candidates", 1);
    mission_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/enav/mission", 1);
    goal_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/enav/goal", 1);

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/navcam/camera_info", 10, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
        intrinsics_ = rover_core::CameraIntrinsics{m->k[0], m->k[4], m->k[2], m->k[5]};
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/navcam/depth_image", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
        onDepth(m);
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, [this](geometry_msgs::msg::PointStamped::ConstSharedPtr m) {
        waypoints_ = {{m->point.x, m->point.y}};
        next_ = 0;
        has_previous_ = false;
        RCLCPP_INFO(get_logger(), "New goal (%.1f, %.1f)", m->point.x, m->point.y);
      });
    hazard_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/mobility/hazard", 10, [this](geometry_msgs::msg::PointStamped::ConstSharedPtr m) { onHazard(*m); });
    mobility_sub_ = create_subscription<rover_msgs::msg::MobilityStatus>(
      "/mobility/status", 10, [this](rover_msgs::msg::MobilityStatus::ConstSharedPtr m) { onMobility(*m); });

    const double rate = declare_parameter("plan_rate", 2.0);
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(1.0 / rate), [this]() { cycle(); });
  }

private:
  struct Goal
  {
    double x;
    double y;
  };

  static std::vector<double> degToRad(std::vector<double> v)
  {
    for (double & d : v) d *= M_PI / 180.0;
    return v;
  }

  void onDepth(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!intrinsics_) return;
    const rclcpp::Time stamp(msg->header.stamp);
    if (!tf_buffer_.canTransform(odom_frame_, base_frame_, stamp)) return;
    const Eigen::Isometry3d odom_from_base =
      tf2::transformToEigen(tf_buffer_.lookupTransform(odom_frame_, base_frame_, stamp));

    const cv::Mat depth = cv_bridge::toCvShare(msg, "32FC1")->image;
    std::vector<Eigen::Vector3f> points = rover_core::depthToPoints(
      depth.ptr<float>(),
      depth.cols,
      depth.rows,
      *intrinsics_,
      base_from_optical_,
      depth_stride_,
      rover_.navcam.near,
      max_range_);
    // Drop returns from the rover itself, then move the rest into the odom frame.
    const float half_length = static_cast<float>(rover_.body_length / 2.0 + rover_.wheel_radius + 0.1);
    const float half_width = static_cast<float>(rover_.track_width / 2.0 + rover_.wheel_width);
    const Eigen::Isometry3f tf = odom_from_base.cast<float>();
    std::vector<Eigen::Vector3f> odom_points;
    odom_points.reserve(points.size());
    for (const auto & p : points) {
      if (std::abs(p.x()) < half_length && std::abs(p.y()) < half_width) continue;
      odom_points.push_back(tf * p);
    }
    height_map_.recenter(odom_from_base.translation().x(), odom_from_base.translation().y(), recenter_margin_);
    height_map_.integrate(odom_points);
  }

  // A panorama maps the terrain all around from scratch. After a "no safe path" fault the
  // old map may hold false steps where terrain seen before an odometry error meets terrain
  // seen after it, and those can block every path. The ground under the turning rover is the
  // plane it stands on.
  void onMobility(const rover_msgs::msg::MobilityStatus & msg)
  {
    const bool panorama = msg.mode == rover_msgs::msg::MobilityStatus::MODE_PANORAMA;
    if (panorama && !panorama_) {
      height_map_.clear();
      RCLCPP_INFO(get_logger(), "Panorama: cleared the height map to map the terrain afresh");
    }
    panorama_ = panorama;
  }

  void onHazard(const geometry_msgs::msg::PointStamped & msg)
  {
    if (!map_from_odom_) return;
    double mx;
    double my;
    rover_core::transformPoint(*map_from_odom_, msg.point.x, msg.point.y, mx, my);
    rover_core::stampKeepOut(keep_out_, mx, my, hazard_radius_);
    RCLCPP_WARN(
      get_logger(), "Hazard reported by mobility: keep-out %.1f m around (%.1f, %.1f)", hazard_radius_, mx, my);
  }

  void cycle()
  {
    ++cycles_;
    if (
      !tf_buffer_.canTransform(odom_frame_, base_frame_, tf2::TimePointZero) ||
      !tf_buffer_.canTransform(map_frame_, odom_frame_, tf2::TimePointZero))
    {
      return;
    }
    const auto odom_tf = tf_buffer_.lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    const Pose2 rover = toPose2(odom_tf);
    map_from_odom_ = toPose2(tf_buffer_.lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero));
    // Only before the camera has seen anything (startup) and during a panorama, which starts
    // from a cleared map: otherwise unseen ground near the rover (e.g. behind a rock) must
    // stay unknown rather than be assumed flat.
    if (!footprint_stamped_ || panorama_) {
      height_map_.stampFootprint(tf2::transformToEigen(odom_tf), rover_.pivotHeight(), footprint_radius_);
      footprint_stamped_ = true;
    }
    rover_core::stampObservedCost(observed_cost_, height_map_.elevation(), *map_from_odom_, cost_);
    if (cycles_ % kMapPublishCycles == 0) publishMaps();

    rover_msgs::msg::EnavPlan msg;
    msg.header.stamp = odom_tf.header.stamp;
    msg.header.frame_id = odom_frame_;
    msg.path.header = msg.header;
    const Pose2 rover_map = rover_core::compose(*map_from_odom_, rover);
    if (next_ >= waypoints_.size()) {
      msg.status = rover_msgs::msg::EnavPlan::STATUS_IDLE;
      plan_pub_->publish(msg);
      publishMission(rover_map, {});
      return;
    }
    const Goal goal = waypoints_[next_];
    publishGoal(goal);
    if (std::hypot(goal.x - rover_map.x, goal.y - rover_map.y) < goal_tolerance_) {
      RCLCPP_INFO(get_logger(), "Reached waypoint %zu/%zu (%.1f, %.1f)", next_ + 1, waypoints_.size(), goal.x, goal.y);
      ++next_;
      has_previous_ = false;
      msg.status = rover_msgs::msg::EnavPlan::STATUS_GOAL_REACHED;
      plan_pub_->publish(msg);
      return;
    }

    const auto start = std::chrono::steady_clock::now();
    const Grid<float> cost = rover_core::combineCost(orbital_cost_, observed_cost_, keep_out_);
    const Grid<float> ctg = rover_core::costToGo(cost, goal.x, goal.y, goal_tolerance_, cost_.cost_weight);
    const rover_core::PlanInputs in{
      rover, *map_from_odom_, &height_map_.elevation(), &cost, &ctg, has_previous_, previous_commit_yaw_};
    const rover_core::PlanResult result =
      rover_core::plan(rover_, tree_, in, weights_, cost_.cost_weight, max_ace_paths_);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

    msg.candidates = result.candidates;
    msg.ace_evaluations = result.ace_evaluations;
    msg.planning_time_ms = static_cast<float>(ms);
    if (result.found) {
      msg.status = rover_msgs::msg::EnavPlan::STATUS_DRIVING;
      msg.commit_length = static_cast<float>(result.commit_length);
      for (const auto & p : result.poses) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg.header;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.position.z = odom_tf.transform.translation.z;
        pose.pose.orientation = yawQuaternion(p.yaw);
        msg.path.poses.push_back(pose);
        if (p.s <= result.commit_length) previous_commit_yaw_ = p.yaw;
      }
      has_previous_ = true;
    } else {
      msg.status = rover_msgs::msg::EnavPlan::STATUS_NO_SAFE_PATH;
      std::string reasons;
      for (size_t k = 0; k < result.rejections.size(); ++k) {
        if (result.rejections[k] == 0) continue;
        reasons += std::string(" ") + rover_core::toString(static_cast<rover_core::AceResult>(k)) + "=" +
                   std::to_string(result.rejections[k]);
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "No safe path: %d candidates, ACE rejected %zu:%s",
        result.candidates,
        result.ace_rejected.size(),
        reasons.c_str());
    }
    plan_pub_->publish(msg);
    publishCandidates(result, rover, odom_tf.transform.translation.z - rover_.pivotHeight() + 0.1);
    publishMission(rover_map, rover_core::routeToGoal(ctg, rover_map.x, rover_map.y));
    if (cycles_ % kMapPublishCycles == 0) publishCosts(cost, ctg, msg.header.stamp);
  }

  void publishCosts(const Grid<float> & cost, const Grid<float> & ctg, const builtin_interfaces::msg::Time & stamp)
  {
    double max_ctg = 1.0;
    for (float v : ctg.data) {
      if (std::isfinite(v)) max_ctg = std::max(max_ctg, static_cast<double>(v));
    }
    auto grid = toOccupancy(cost, 1.0, map_frame_);
    grid.header.stamp = stamp;
    cost_pub_->publish(grid);
    grid = toOccupancy(ctg, max_ctg, map_frame_);
    grid.header.stamp = stamp;
    ctg_pub_->publish(grid);
  }

  void publishMaps()
  {
    const Grid<float> & e = height_map_.elevation();
    rover_msgs::msg::HeightMap hm;
    hm.header.stamp = now();
    hm.header.frame_id = odom_frame_;
    hm.resolution = static_cast<float>(e.resolution);
    hm.width = e.width;
    hm.height = e.height;
    hm.origin_x = e.origin_x;
    hm.origin_y = e.origin_y;
    hm.elevation = e.data;
    height_pub_->publish(hm);

    // Every kCloudStride-th cell (15 cm) as a point cloud for Foxglove: plenty to see the
    // terrain, ~5x less to send and draw than every cell.
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = hm.header;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    size_t n = 0;
    for (int j = 0; j < e.height; j += kCloudStride) {
      for (int i = 0; i < e.width; i += kCloudStride) n += std::isnan(e.at(i, j)) ? 0 : 1;
    }
    mod.resize(n);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (int j = 0; j < e.height; j += kCloudStride) {
      for (int i = 0; i < e.width; i += kCloudStride) {
        if (std::isnan(e.at(i, j))) continue;
        *x = static_cast<float>(e.cellX(i));
        *y = static_cast<float>(e.cellY(j));
        *z = e.at(i, j);
        ++x;
        ++y;
        ++z;
      }
    }
    cloud_pub_->publish(cloud);
  }

  void publishGoal(const Goal & goal)
  {
    geometry_msgs::msg::PointStamped g;
    g.header.stamp = now();
    g.header.frame_id = map_frame_;
    g.point.x = goal.x;
    g.point.y = goal.y;
    goal_pub_->publish(g);
  }

  void publishCandidates(const rover_core::PlanResult & result, const Pose2 & rover, double z)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    auto add = [&](int index, const Rgba & color, double width) {
      auto m = marker(odom_frame_, "candidates", visualization_msgs::msg::Marker::LINE_STRIP, color);
      m.header.stamp = now();
      m.id = static_cast<int>(markers.markers.size());
      m.scale.x = width;
      for (const auto & p : rover_core::toOdom(tree_[index], rover)) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = z;
        m.points.push_back(pt);
      }
      markers.markers.push_back(m);
    };
    // Red: paths ACE checked and rejected this cycle. Green: the path being driven.
    for (size_t k = 0; k < std::min(kMaxRejectedShown, result.ace_rejected.size()); ++k) {
      add(result.ace_rejected[k], kRejected, 0.025);
    }
    if (result.found) add(result.index, kChosen, 0.08);
    marker_pub_->publish(markers);
  }

  // Where the rover is going: waypoint pillars (done grey, current magenta, upcoming orange),
  // the global route down the cost-to-go grid, and a status label above the rover.
  void publishMission(const Pose2 & rover_map, const std::vector<Pose2> & route)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    auto add = [&](visualization_msgs::msg::Marker m) {
      m.header.stamp = now();
      m.id = static_cast<int>(markers.markers.size());
      markers.markers.push_back(m);
    };
    using visualization_msgs::msg::Marker;

    for (size_t k = 0; k < waypoints_.size(); ++k) {
      const bool current = k == next_;
      const Rgba & color = k < next_ ? kWaypointDone : current ? kWaypointCurrent : kWaypointUpcoming;
      const double height = current ? 2.0 : 1.2;
      auto pillar = marker(map_frame_, "mission", Marker::CYLINDER, color);
      pillar.pose.position.x = waypoints_[k].x;
      pillar.pose.position.y = waypoints_[k].y;
      pillar.pose.position.z = rover_core::bilinear(dem_, waypoints_[k].x, waypoints_[k].y) + height / 2.0;
      pillar.scale.x = pillar.scale.y = current ? 0.4 : 0.25;
      pillar.scale.z = height;
      add(pillar);

      auto label = marker(map_frame_, "mission", Marker::TEXT_VIEW_FACING, kWhite);
      label.pose.position = pillar.pose.position;
      label.pose.position.z += height / 2.0 + 0.4;
      label.scale.z = current ? 0.6 : 0.4;
      label.text = (current ? "GOAL " : "") + std::to_string(k + 1);
      add(label);
    }

    if (!route.empty()) {
      auto line = marker(map_frame_, "mission", Marker::LINE_STRIP, kRoute);
      line.scale.x = 0.08;
      for (const auto & p : route) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = rover_core::bilinear(dem_, p.x, p.y) + 0.15;
        line.points.push_back(pt);
      }
      add(line);
    }

    // The rover itself is drawn from its URDF (robot_state_publisher); this is its status label.
    auto status = marker(base_frame_, "mission", Marker::TEXT_VIEW_FACING, kWhite);
    status.frame_locked = true;
    status.pose.position.z = 1.4;
    status.scale.z = 0.35;
    if (waypoints_.empty()) {
      status.text = "no goal: use Publish point";
    } else if (next_ >= waypoints_.size()) {
      status.text = "all waypoints reached";
    } else {
      const Goal & g = waypoints_[next_];
      char buf[64];
      std::snprintf(
        buf,
        sizeof(buf),
        "goal %zu/%zu: %.1f m",
        next_ + 1,
        waypoints_.size(),
        std::hypot(g.x - rover_map.x, g.y - rover_map.y));
      status.text = buf;
    }
    add(status);
    mission_pub_->publish(markers);
  }

  const rover_core::RoverParams rover_;
  const Eigen::Isometry3d base_from_optical_;
  rover_core::HeightMap height_map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string odom_frame_;
  std::string map_frame_;
  std::string base_frame_;
  double recenter_margin_;
  int depth_stride_;
  double max_range_;
  double footprint_radius_;
  double goal_tolerance_;
  int max_ace_paths_;
  double hazard_radius_;
  rover_core::CostConfig cost_;
  rover_core::RankWeights weights_;
  std::vector<rover_core::CandidatePath> tree_;

  Grid<float> dem_;
  Grid<float> orbital_cost_;
  Grid<float> observed_cost_;
  Grid<float> keep_out_;
  std::vector<Goal> waypoints_;  // map frame
  size_t next_{0};  // index of the current goal in waypoints_
  std::optional<rover_core::CameraIntrinsics> intrinsics_;
  std::optional<Pose2> map_from_odom_;
  bool has_previous_{false};
  bool footprint_stamped_{false};
  bool panorama_{false};
  double previous_commit_yaw_{0.0};
  int cycles_{0};

  rclcpp::Publisher<rover_msgs::msg::EnavPlan>::SharedPtr plan_pub_;
  rclcpp::Publisher<rover_msgs::msg::HeightMap>::SharedPtr height_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr ctg_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mission_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr hazard_sub_;
  rclcpp::Subscription<rover_msgs::msg::MobilityStatus>::SharedPtr mobility_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnavNode>());
  rclcpp::shutdown();
  return 0;
}
