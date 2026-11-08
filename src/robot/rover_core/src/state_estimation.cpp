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

#include "rover_core/state_estimation.hpp"

#include <cmath>
#include <optional>

namespace rover_core
{

namespace
{

constexpr double kHistorySeconds = 10.0;
constexpr double kStampTolerance = 0.03;
constexpr double kDistanceVariance = 0.03;  // position std grows by this fraction of distance
// m of forward wheel travel a VO interval needs to count towards slip. A turn in place has
// none (its wheels cancel out), so VO noise over it would read as slip; a slipping rover still
// spins its wheels forward (~0.04 m per interval at drive speed).
constexpr double kMinSlipSample = 0.01;

Eigen::Quaterniond rotationVectorToQuaternion(const Eigen::Vector3d & v)
{
  const double angle = v.norm();
  if (angle < 1e-12) return Eigen::Quaterniond::Identity();
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, v / angle));
}

}  // namespace

void AttitudeFilter::initialize(
  const Eigen::Vector3d & accel, const Eigen::Vector3d & sun_body, const Eigen::Vector3d & sun_world)
{
  // TRIAD: build orthonormal triads from (up, sun) in both frames; R maps body -> world.
  const Eigen::Vector3d b1 = accel.normalized();
  const Eigen::Vector3d b2 = b1.cross(sun_body).normalized();
  const Eigen::Vector3d w1 = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d w2 = w1.cross(sun_world).normalized();
  Eigen::Matrix3d body;
  Eigen::Matrix3d world;
  body << b1, b2, b1.cross(b2);
  world << w1, w2, w1.cross(w2);
  q_ = Eigen::Quaterniond(world * body.transpose()).normalized();
  bias_.setZero();
  initialized_ = true;
}

void AttitudeFilter::propagate(const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel, double dt)
{
  Eigen::Vector3d error = Eigen::Vector3d::Zero();
  // The accelerometer measures "up" only when the rover is not accelerating much.
  if (std::abs(accel.norm() - gains_.gravity) < 0.1 * gains_.gravity) {
    const Eigen::Vector3d up_estimated = q_.conjugate() * Eigen::Vector3d::UnitZ();
    error = accel.normalized().cross(up_estimated);
  }
  bias_ -= gains_.ki_gravity * error * dt;
  const Eigen::Vector3d omega = gyro - bias_ + gains_.kp_gravity * error;
  q_ = (q_ * rotationVectorToQuaternion(omega * dt)).normalized();
}

void AttitudeFilter::correctSun(const Eigen::Vector3d & sun_body, const Eigen::Vector3d & sun_world, double sun_period)
{
  const Eigen::Vector3d sun_estimated = q_.conjugate() * sun_world;
  const Eigen::Vector3d error = sun_body.normalized().cross(sun_estimated);
  q_ = (q_ * rotationVectorToQuaternion(gains_.k_sun * error)).normalized();
  bias_ -= gains_.ki_sun * error / sun_period;
}

void DeadReckoning::addWheelMotion(double stamp, double distance, const Eigen::Quaterniond & attitude)
{
  position_ += attitude * Eigen::Vector3d(distance, 0.0, 0.0);
  distance_ += distance;
  variance_ += std::pow(kDistanceVariance * distance, 2);
  history_.push_back({stamp, position_, attitude, distance_});
  while (!history_.empty() && history_.front().stamp < stamp - kHistorySeconds) history_.pop_front();
}

int DeadReckoning::find(double stamp) const
{
  int best = -1;
  double best_dt = kStampTolerance;
  for (int i = static_cast<int>(history_.size()) - 1; i >= 0; --i) {
    const double dt = std::abs(history_[i].stamp - stamp);
    if (dt <= best_dt) {
      best = i;
      best_dt = dt;
    }
    if (history_[i].stamp < stamp - kStampTolerance) break;
  }
  return best;
}

std::optional<DeadReckoning::VoUpdate> DeadReckoning::applyVisualOdometry(
  double t0, double t1, const Eigen::Vector3d & translation)
{
  const int i0 = find(t0);
  const int i1 = find(t1);
  if (i0 < 0 || i1 < 0 || i1 < i0) return std::nullopt;
  const Entry & e0 = history_[i0];
  const Entry & e1 = history_[i1];

  // The rover moves in its own deck plane, so VO's body-frame z is error, and a biased one: the
  // false climb left steps in ENav's height map where older and newer observations met.
  const Eigen::Vector3d planar(translation.x(), translation.y(), 0.0);
  // Earlier corrections shifted e0 and e1 together, so their difference is wheel motion only.
  const Eigen::Vector3d correction = e0.attitude * planar - (e1.position - e0.position);
  // Signed forward progress: a rover sliding back while its wheels drive forward must not
  // look like a rover that is moving fine.
  const VoUpdate update{e1.distance - e0.distance, translation.x()};
  for (size_t i = i1; i < history_.size(); ++i) history_[i].position += correction;
  position_ += correction;
  return update;
}

void SlipEstimator::add(double stamp, double wheel_distance, double vo_distance)
{
  if (std::abs(wheel_distance) < kMinSlipSample) return;
  samples_.push_back({stamp, wheel_distance, vo_distance});
  // Keep the newest samples that make up the window distance.
  double total = 0.0;
  size_t keep = 0;
  for (auto it = samples_.rbegin(); it != samples_.rend() && total < window_distance_; ++it, ++keep) {
    total += std::abs(it->wheel);
  }
  while (samples_.size() > keep) samples_.pop_front();
}

std::optional<double> SlipEstimator::slip(double now) const
{
  double wheel = 0.0;
  double vo = 0.0;
  for (const auto & s : samples_) {
    if (s.stamp < now - max_age_) continue;
    wheel += s.wheel;
    vo += s.vo;
  }
  if (std::abs(wheel) < min_distance_) return std::nullopt;
  return 1.0 - vo / wheel;
}

double SlipEstimator::wheelDistance() const
{
  double total = 0.0;
  for (const auto & s : samples_) total += s.wheel;
  return total;
}

double SlipEstimator::voDistance() const
{
  double total = 0.0;
  for (const auto & s : samples_) total += s.vo;
  return total;
}

}  // namespace rover_core
