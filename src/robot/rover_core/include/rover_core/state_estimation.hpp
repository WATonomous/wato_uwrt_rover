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

#pragma once

#include <deque>
#include <Eigen/Geometry>  // NOLINT(build/include_order): clang-format sorts it with the C++ headers
#include <optional>

namespace rover_core
{

// Attitude from gyro integration, corrected toward two reference vectors (Mahony et al. 2008):
// gravity (roll/pitch, from the accelerometer) and the sun (heading, from the sun sensor).
// The integral term estimates gyro bias, which is what makes heading drift on Mars without a
// sun fix. Orientation maps base_link vectors into the odom frame (x east, y north, z up).
class AttitudeFilter
{
public:
  struct Gains
  {
    double gravity;  // m/s^2, local gravity magnitude
    double kp_gravity;  // 1/s
    double ki_gravity;  // 1/s^2
    double k_sun;  // fraction of the sun error removed per measurement
    double ki_sun;  // fraction of the implied gyro bias absorbed per measurement
  };

  explicit AttitudeFilter(const Gains & gains)
  : gains_(gains)
  {}

  bool initialized() const
  {
    return initialized_;
  }

  // TRIAD from one accelerometer sample (at rest) and one sun vector.
  void initialize(const Eigen::Vector3d & accel, const Eigen::Vector3d & sun_body, const Eigen::Vector3d & sun_world);

  void propagate(const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel, double dt);

  // sun_period: seconds since the previous sun correction (converts error to a rate).
  void correctSun(const Eigen::Vector3d & sun_body, const Eigen::Vector3d & sun_world, double sun_period);

  const Eigen::Quaterniond & orientation() const
  {
    return q_;
  }

  const Eigen::Vector3d & gyroBias() const
  {
    return bias_;
  }

private:
  Gains gains_;
  bool initialized_{false};
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d bias_{Eigen::Vector3d::Zero()};
};

// Position by dead reckoning. Wheel odometry is integrated as it arrives; when a visual
// odometry measurement covers an interval, it replaces the wheel motion over that interval
// (what Perseverance does: VO, when it succeeds, overrides wheel odometry, which slips).
class DeadReckoning
{
public:
  // Forward distance driven since the previous call, with the attitude at `stamp`.
  void addWheelMotion(double stamp, double distance, const Eigen::Quaterniond & attitude);

  struct VoUpdate
  {
    double wheel_distance;  // |forward wheel distance| over the interval
    double vo_distance;  // |VO translation| over the interval
  };

  // Replace the wheel motion over [t0, t1] with a VO translation expressed in base_link at
  // t0. Returns nullopt when the history does not cover the interval.
  std::optional<VoUpdate> applyVisualOdometry(double t0, double t1, const Eigen::Vector3d & translation);

  const Eigen::Vector3d & position() const
  {
    return position_;
  }

  double positionVariance() const
  {
    return variance_;
  }

private:
  struct Entry
  {
    double stamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond attitude;
    double distance;  // cumulative forward wheel distance
  };

  // Index of the entry closest to `stamp`, or -1 if none is within tolerance.
  int find(double stamp) const;

  std::deque<Entry> history_;
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  double distance_{0.0};
  double variance_{0.0};
};

// Slip over a sliding window of VO intervals: 1 - vo_distance / wheel_distance.
class SlipEstimator
{
public:
  SlipEstimator(double window_distance, double min_distance, double max_age)
  : window_distance_(window_distance)
  , min_distance_(min_distance)
  , max_age_(max_age)
  {}

  void add(double stamp, double wheel_distance, double vo_distance);

  // nullopt until at least min_distance of wheel motion is inside the window.
  std::optional<double> slip(double now) const;
  double wheelDistance() const;
  double voDistance() const;

private:
  struct Sample
  {
    double stamp;
    double wheel;
    double vo;
  };

  std::deque<Sample> samples_;
  double window_distance_;
  double min_distance_;
  double max_age_;
};

}  // namespace rover_core
