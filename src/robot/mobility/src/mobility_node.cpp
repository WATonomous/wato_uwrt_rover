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

// Mobility: executes ENav's committed arc and owns fault protection.
//   PANORAMA   rotate once in place at startup (on the flat landing pad) so ENav has terrain
//              all around; never used for recovery, since the turn is not checked by ACE
//   IDLE       wait for a plan
//   DRIVING    turn in place to the path heading, then pure-pursuit along the committed arc
//   FAULT      stopped; tilt / suspension / slip / stall / no safe path. Cleared by
//              /mobility/clear_fault (the "ground in the loop"), or by auto-recovery
//   RECOVERING retrace its own trail backwards (ground it just drove over), and report the
//              hazard to ENav as a keep-out zone

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rover_core/fault_monitor.hpp"
#include "rover_core/grid.hpp"
#include "rover_core/params.hpp"
#include "rover_msgs/msg/enav_plan.hpp"
#include "rover_msgs/msg/mobility_status.hpp"
#include "rover_msgs/msg/slip_estimate.hpp"

using rover_core::Fault;
using rover_msgs::msg::EnavPlan;
using rover_msgs::msg::MobilityStatus;

namespace
{

const std::array<std::string, 6> kWheelJoints = {
  "wheel_front_left_joint",
  "wheel_middle_left_joint",
  "wheel_rear_left_joint",
  "wheel_front_right_joint",
  "wheel_middle_right_joint",
  "wheel_rear_right_joint"};
constexpr double kRecoveryReset = 30.0;  // s of fault-free driving that restores the recovery budget
constexpr double kTrailSpacing = 0.05;  // m between recorded trail points
constexpr size_t kTrailLength = 200;  // points kept (10 m)
constexpr double kMinBackout = 0.2;  // m of trail needed to back out at all
constexpr double kBackoutDone = 0.1;  // m from the end of the retraced trail
constexpr size_t kBackoutLookahead = 8;  // trail points (0.4 m) ahead of the closest one
constexpr double kBackoutProgress = 0.05;  // m closer to the end that counts as progress
constexpr double kBackoutNoProgress = 10.0;  // s without progress before giving up

enum class Backout
{
  kGoing,
  kDone,
  kStuck,
};

const std::array<std::string, 4> kSuspensionJoints = {
  "rocker_left_joint", "rocker_right_joint", "bogie_left_joint", "bogie_right_joint"};

enum class Mode : uint8_t
{
  kIdle = MobilityStatus::MODE_IDLE,
  kPanorama = MobilityStatus::MODE_PANORAMA,
  kDriving = MobilityStatus::MODE_DRIVING,
  kFault = MobilityStatus::MODE_FAULT,
  kRecovering = MobilityStatus::MODE_RECOVERING,
};

struct Pose
{
  double x;
  double y;
  double yaw;
  double roll;
  double pitch;
};

Pose toPose(const nav_msgs::msg::Odometry & odom)
{
  const auto & q = odom.pose.pose.orientation;
  return {
    odom.pose.pose.position.x,
    odom.pose.pose.position.y,
    std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
    std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y)),
    std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0))};
}

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class MobilityNode : public rclcpp::Node
{
public:
  MobilityNode()
  : Node("mobility")
  , rover_(rover_core::loadRoverParams(rover_core::roverConfigPath()))
  , monitor_(rover_.faults)
  , lookahead_(declare_parameter("lookahead", 0.6))
  , turn_start_(declare_parameter("turn_start", 0.6))
  , turn_done_(declare_parameter("turn_done", 0.05))
  , plan_timeout_(declare_parameter("plan_timeout", 2.0))
  , no_path_limit_(declare_parameter("no_safe_path_cycles", 10))
  , auto_recover_(declare_parameter("auto_recover", false))
  , max_recoveries_(declare_parameter("max_recoveries", 3))
  , backout_distance_(declare_parameter("backout_distance", 1.0))
  , backout_speed_(declare_parameter("backout_speed", 0.15))
  , hazard_ahead_(declare_parameter("hazard_ahead", 0.6))
  , mode_(declare_parameter("startup_panorama", true) ? Mode::kPanorama : Mode::kIdle)
  {
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_pub_ = create_publisher<MobilityStatus>("/mobility/status", 10);
    hazard_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/mobility/hazard", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { odom_ = *m; });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        joints_ = *m;
      });
    slip_sub_ = create_subscription<rover_msgs::msg::SlipEstimate>(
      "/localization/slip", 10, [this](rover_msgs::msg::SlipEstimate::ConstSharedPtr m) { slip_ = *m; });
    plan_sub_ = create_subscription<EnavPlan>("/enav/plan", 10, [this](EnavPlan::ConstSharedPtr m) { onPlan(*m); });
    clear_srv_ = create_service<std_srvs::srv::Trigger>(
      "/mobility/clear_fault",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        res->success = mode_ == Mode::kFault;
        res->message = res->success ? "fault cleared" : "no active fault";
        if (res->success) setMode(Mode::kIdle, Fault::kNone, "cleared by operator");
      });
    panorama_srv_ = create_service<std_srvs::srv::Trigger>(
      "/mobility/panorama",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        res->success = mode_ == Mode::kIdle;
        res->message = res->success ? "panorama started" : "rover is busy";
        if (res->success) setMode(Mode::kPanorama, Fault::kNone, "");
      });
    const double rate = declare_parameter("control_rate", 20.0);
    dt_ = 1.0 / rate;
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(dt_), [this]() { tick(); });
  }

private:
  void onPlan(const EnavPlan & msg)
  {
    plan_ = msg;
    no_path_count_ = msg.status == EnavPlan::STATUS_NO_SAFE_PATH ? no_path_count_ + 1 : 0;
    path_s_.clear();
    double s = 0.0;
    for (size_t k = 0; k < msg.path.poses.size(); ++k) {
      if (k > 0) {
        s += std::hypot(
          msg.path.poses[k].pose.position.x - msg.path.poses[k - 1].pose.position.x,
          msg.path.poses[k].pose.position.y - msg.path.poses[k - 1].pose.position.y);
      }
      path_s_.push_back(s);
    }
  }

  void tick()
  {
    if (!odom_ || !joints_) return;
    const Pose pose = toPose(*odom_);
    double v = 0.0;
    double w = 0.0;

    if (mode_ == Mode::kPanorama || mode_ == Mode::kDriving || mode_ == Mode::kRecovering) {
      // Backing out is how a slip or suspension fault gets fixed, so those are not checked
      // while recovering; tilt and stall always are.
      const bool recovering = mode_ == Mode::kRecovering;
      std::string detail;
      const Fault fault = monitor_.update(faultInputs(pose), {mode_ == Mode::kDriving, !recovering}, detail);
      if (fault != Fault::kNone) enterFault(fault, detail, pose);
    }

    switch (mode_) {
      case Mode::kPanorama:
        if (panorama_last_yaw_) panorama_turned_ += std::abs(rover_core::wrapAngle(pose.yaw - *panorama_last_yaw_));
        panorama_last_yaw_ = pose.yaw;
        w = rover_.drive.turn_rate;
        if (panorama_turned_ >= 2.0 * M_PI) {
          w = 0.0;
          setMode(Mode::kIdle, Fault::kNone, "panorama complete");
        }
        break;
      case Mode::kIdle:
        if (freshPlan() && plan_->status == EnavPlan::STATUS_DRIVING) setMode(Mode::kDriving, Fault::kNone, "");
        if (freshPlan() && no_path_count_ >= no_path_limit_) {
          enterFault(Fault::kNoSafePath, "ENav found no safe path", pose);
        }
        break;
      case Mode::kDriving:
        // A good stretch of fault-free driving earns the recovery budget back.
        if (now() - fault_time_ > rclcpp::Duration::from_seconds(kRecoveryReset)) recoveries_ = 0;
        drive(pose, v, w);
        break;
      case Mode::kFault:
        if (auto_recover_ && recoveries_ < max_recoveries_ && now() - fault_time_ > rclcpp::Duration::from_seconds(1.0))
        {
          // Back out along the rover's own tracks: ground it has just driven over, so no
          // blind turns. Without a usable trail it waits for the ground instead.
          if (startBackout()) {
            ++recoveries_;
            fault_before_recovery_ = fault_;
            if (fault_ != Fault::kNoSafePath) reportHazard();
            setMode(Mode::kRecovering, Fault::kNone, "auto-recovery: backing out along the trail");
          }
        }
        break;
      case Mode::kRecovering:
        switch (backOut(pose, v, w)) {
          case Backout::kGoing:
            break;
          case Backout::kDone:
            setMode(Mode::kIdle, Fault::kNone, "backed out");
            break;
          case Backout::kStuck:
            // Do not keep thrashing: stop and wait for the ground.
            v = 0.0;
            w = 0.0;
            recoveries_ = max_recoveries_;
            enterFault(fault_before_recovery_, "auto-recovery could not back out", pose);
            break;
        }
        break;
    }
    if (mode_ == Mode::kDriving) recordTrail(pose);
    if (mode_ == Mode::kFault) {
      v = 0.0;
      w = 0.0;
    }
    command(v, w);
    publishStatus();
  }

  // Positions driven forward, newest last, spaced kTrailSpacing apart.
  void recordTrail(const Pose & pose)
  {
    if (!trail_.empty() && std::hypot(pose.x - trail_.back()[0], pose.y - trail_.back()[1]) < kTrailSpacing) return;
    trail_.push_back({pose.x, pose.y});
    if (trail_.size() > kTrailLength) trail_.pop_front();
  }

  // The stretch of trail to retrace (newest first), up to backout_distance long.
  bool startBackout()
  {
    backout_path_.clear();
    backout_start_ = now();
    backout_progress_time_ = now();
    backout_best_ = std::numeric_limits<double>::max();
    double length = 0.0;
    for (auto it = trail_.rbegin(); it != trail_.rend() && length < backout_distance_; ++it) {
      if (!backout_path_.empty()) {
        length += std::hypot((*it)[0] - backout_path_.back()[0], (*it)[1] - backout_path_.back()[1]);
      }
      backout_path_.push_back(*it);
    }
    if (length < kMinBackout) return false;
    backout_length_ = length;
    // The retraced part is behind the rover afterwards; drop it so the next backout goes further back.
    trail_.resize(trail_.size() - backout_path_.size() + 1);
    return true;
  }

  // Reverse pure pursuit along backout_path_.
  Backout backOut(const Pose & pose, double & v, double & w)
  {
    const auto & end = backout_path_.back();
    const double remaining = std::hypot(end[0] - pose.x, end[1] - pose.y);
    if (remaining < kBackoutDone) return Backout::kDone;
    // Stuck: no progress towards the end for a while, or far longer than the drive should take.
    if (remaining < backout_best_ - kBackoutProgress) {
      backout_best_ = remaining;
      backout_progress_time_ = now();
    }
    const double expected = backout_length_ / backout_speed_;
    if (
      now() - backout_progress_time_ > rclcpp::Duration::from_seconds(kBackoutNoProgress) ||
      now() - backout_start_ > rclcpp::Duration::from_seconds(3.0 * expected + kBackoutNoProgress))
    {
      return Backout::kStuck;
    }
    size_t closest = 0;
    for (size_t k = 0; k < backout_path_.size(); ++k) {
      if (
        std::hypot(backout_path_[k][0] - pose.x, backout_path_[k][1] - pose.y) <
        std::hypot(backout_path_[closest][0] - pose.x, backout_path_[closest][1] - pose.y))
      {
        closest = k;
      }
    }
    if (closest + 1 == backout_path_.size()) return Backout::kDone;
    const size_t target = std::min(backout_path_.size() - 1, closest + kBackoutLookahead);
    const double dx = backout_path_[target][0] - pose.x;
    const double dy = backout_path_[target][1] - pose.y;
    const double tx = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
    const double ty = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
    // Arc through the rover tangent to its heading and through the target, driven backwards.
    v = -backout_speed_;
    w = std::clamp(v * 2.0 * ty / std::max(tx * tx + ty * ty, 0.04), -rover_.drive.turn_rate, rover_.drive.turn_rate);
    return Backout::kGoing;
  }

  bool freshPlan() const
  {
    return plan_ && now() - rclcpp::Time(plan_->header.stamp) < rclcpp::Duration::from_seconds(plan_timeout_);
  }

  void drive(const Pose & pose, double & v, double & w)
  {
    if (!freshPlan() || plan_->status == EnavPlan::STATUS_IDLE || plan_->status == EnavPlan::STATUS_GOAL_REACHED) {
      setMode(Mode::kIdle, Fault::kNone, freshPlan() ? "goal reached or no goal" : "plan timed out");
      return;
    }
    if (plan_->status == EnavPlan::STATUS_NO_SAFE_PATH) {
      if (no_path_count_ >= no_path_limit_) enterFault(Fault::kNoSafePath, "ENav found no safe path", pose);
      return;  // stop and wait: a few failed cycles in a row become a fault
    }
    const auto & poses = plan_->path.poses;

    // Turn in place to the path heading first (skid steer).
    const double heading_error = rover_core::wrapAngle(yawOf(poses.front().pose.orientation) - pose.yaw);
    if (std::abs(heading_error) > turn_start_) turning_ = true;
    if (turning_) {
      if (std::abs(heading_error) < turn_done_) {
        turning_ = false;
      } else {
        w = std::copysign(std::min(rover_.drive.turn_rate, 2.0 * std::abs(heading_error) + 0.05), heading_error);
        return;
      }
    }

    // Pure pursuit along the committed part of the path.
    size_t closest = 0;
    double best = std::numeric_limits<double>::max();
    for (size_t k = 0; k < poses.size(); ++k) {
      const double d = std::hypot(poses[k].pose.position.x - pose.x, poses[k].pose.position.y - pose.y);
      if (d < best) {
        best = d;
        closest = k;
      }
    }
    if (path_s_[closest] >= plan_->commit_length - 0.05) return;  // end of the commitment: wait for a new plan
    size_t target = closest;
    while (target + 1 < poses.size() && path_s_[target] < path_s_[closest] + lookahead_) ++target;
    const double dx = poses[target].pose.position.x - pose.x;
    const double dy = poses[target].pose.position.y - pose.y;
    const double alpha = rover_core::wrapAngle(std::atan2(dy, dx) - pose.yaw);
    v = rover_.drive.speed;
    w = std::clamp(
      2.0 * v * std::sin(alpha) / std::max(std::hypot(dx, dy), 0.2), -rover_.drive.turn_rate, rover_.drive.turn_rate);
  }

  rover_core::FaultInputs faultInputs(const Pose & pose) const
  {
    rover_core::FaultInputs in{};
    in.roll = pose.roll;
    in.pitch = pose.pitch;
    in.dt = dt_;
    const auto & j = *joints_;
    auto find = [&](const std::string & name) {
      for (size_t i = 0; i < j.name.size(); ++i) {
        if (j.name[i] == name) return i;
      }
      throw std::runtime_error("joint_states is missing " + name);
    };
    in.rocker = {j.position[find(kSuspensionJoints[0])], j.position[find(kSuspensionJoints[1])]};
    in.bogie = {j.position[find(kSuspensionJoints[2])], j.position[find(kSuspensionJoints[3])]};
    for (size_t k = 0; k < 6; ++k) {
      in.wheel_measured[k] = j.velocity[find(kWheelJoints[k])];
      in.wheel_command[k] = wheel_command_[k];
    }
    if (slip_ && slip_->valid && now() - rclcpp::Time(slip_->header.stamp) < rclcpp::Duration::from_seconds(2.0)) {
      in.slip = slip_->slip;
    }
    return in;
  }

  void command(double v, double w)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
    const double half = rover_.track_width / 2.0;
    for (size_t k = 0; k < 6; ++k) {
      wheel_command_[k] = (v + (k < 3 ? -w : w) * half) / rover_.wheel_radius;
    }
  }

  void enterFault(Fault fault, const std::string & detail, const Pose & pose)
  {
    fault_time_ = now();
    fault_pose_ = {pose.x, pose.y, pose.yaw};
    setMode(Mode::kFault, fault, detail);
    RCLCPP_ERROR(get_logger(), "FAULT %s: %s", rover_core::toString(fault), detail.c_str());
  }

  // The hazard is just ahead of where the fault happened; ENav turns it into a keep-out zone.
  void reportHazard()
  {
    geometry_msgs::msg::PointStamped hazard;
    hazard.header.stamp = now();
    hazard.header.frame_id = odom_->header.frame_id;
    hazard.point.x = fault_pose_[0] + hazard_ahead_ * std::cos(fault_pose_[2]);
    hazard.point.y = fault_pose_[1] + hazard_ahead_ * std::sin(fault_pose_[2]);
    hazard_pub_->publish(hazard);
  }

  void setMode(Mode mode, Fault fault, const std::string & detail)
  {
    if (mode != mode_ && !detail.empty())
      RCLCPP_INFO(get_logger(), "mobility -> %d: %s", static_cast<int>(mode), detail.c_str());
    if (mode == Mode::kPanorama) {
      panorama_turned_ = 0.0;
      panorama_last_yaw_.reset();
    }
    if (mode != Mode::kFault) monitor_.reset();
    if (mode == Mode::kIdle) no_path_count_ = 0;
    turning_ = false;
    mode_ = mode;
    fault_ = fault;
    detail_ = detail;
  }

  void publishStatus()
  {
    MobilityStatus status;
    status.header.stamp = now();
    status.mode = static_cast<uint8_t>(mode_);
    status.fault = static_cast<uint8_t>(fault_);
    status.detail = detail_;
    status_pub_->publish(status);
  }

  const rover_core::RoverParams rover_;
  rover_core::FaultMonitor monitor_;
  const double lookahead_;
  const double turn_start_;
  const double turn_done_;
  const double plan_timeout_;
  const int no_path_limit_;
  const bool auto_recover_;
  const int max_recoveries_;
  const double backout_distance_;
  const double backout_speed_;
  const double hazard_ahead_;
  double dt_{0.05};

  Mode mode_;
  Fault fault_{Fault::kNone};
  std::string detail_;
  rclcpp::Time fault_time_{0, 0, RCL_ROS_TIME};
  std::array<double, 3> fault_pose_{};
  std::deque<std::array<double, 2>> trail_;
  std::vector<std::array<double, 2>> backout_path_;
  double backout_length_{0.0};
  double backout_best_{0.0};
  rclcpp::Time backout_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time backout_progress_time_{0, 0, RCL_ROS_TIME};
  Fault fault_before_recovery_{Fault::kNone};
  int recoveries_{0};
  int no_path_count_{0};
  bool turning_{false};
  double panorama_turned_{0.0};
  std::optional<double> panorama_last_yaw_;
  std::array<double, 6> wheel_command_{};
  std::vector<double> path_s_;

  std::optional<nav_msgs::msg::Odometry> odom_;
  std::optional<sensor_msgs::msg::JointState> joints_;
  std::optional<rover_msgs::msg::SlipEstimate> slip_;
  std::optional<EnavPlan> plan_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<MobilityStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr hazard_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<rover_msgs::msg::SlipEstimate>::SharedPtr slip_sub_;
  rclcpp::Subscription<EnavPlan>::SharedPtr plan_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr panorama_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobilityNode>());
  rclcpp::shutdown();
  return 0;
}
