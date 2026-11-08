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

#include <gtest/gtest.h>

#include <cmath>

#include "rover_core/state_estimation.hpp"

namespace rover_core
{
namespace
{

constexpr double kG = 3.721;
const AttitudeFilter::Gains kGains{kG, 1.0, 0.02, 0.5, 0.2};

Eigen::Quaterniond rpy(double roll, double pitch, double yaw)
{
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

double angleBetween(const Eigen::Quaterniond & a, const Eigen::Quaterniond & b)
{
  return a.angularDistance(b);
}

const Eigen::Vector3d kSun = Eigen::Vector3d(-0.5, 0.5, 0.64).normalized();

TEST(AttitudeFilter, TriadRecoversAttitude)
{
  const Eigen::Quaterniond truth = rpy(0.05, -0.08, 0.7);
  AttitudeFilter f(kGains);
  f.initialize(truth.conjugate() * Eigen::Vector3d(0, 0, kG), truth.conjugate() * kSun, kSun);
  EXPECT_LT(angleBetween(f.orientation(), truth), 1e-9);
}

TEST(AttitudeFilter, SunAndGravityCancelGyroBias)
{
  const Eigen::Quaterniond truth = rpy(0.05, -0.08, 0.7);
  const Eigen::Vector3d bias(0.0006, -0.0004, 0.0008);
  AttitudeFilter f(kGains);
  f.initialize(truth.conjugate() * Eigen::Vector3d(0, 0, kG), truth.conjugate() * kSun, kSun);
  const double dt = 0.02;
  for (int k = 0; k < 50 * 600; ++k) {
    f.propagate(bias, truth.conjugate() * Eigen::Vector3d(0, 0, kG), dt);
    if (k % 50 == 49) f.correctSun(truth.conjugate() * kSun, kSun, 1.0);
  }
  EXPECT_LT(angleBetween(f.orientation(), truth), 0.2 * M_PI / 180.0);
  EXPECT_LT((f.gyroBias() - bias).norm(), 1e-4);

  // Without the sun, heading drifts with the gyro bias (gravity only fixes roll and pitch).
  AttitudeFilter no_sun(kGains);
  no_sun.initialize(truth.conjugate() * Eigen::Vector3d(0, 0, kG), truth.conjugate() * kSun, kSun);
  for (int k = 0; k < 50 * 600; ++k) no_sun.propagate(bias, truth.conjugate() * Eigen::Vector3d(0, 0, kG), dt);
  EXPECT_GT(angleBetween(no_sun.orientation(), truth), 10.0 * 0.0008 * 600 * 0.05);
}

TEST(AttitudeFilter, TracksRotation)
{
  AttitudeFilter f(kGains);
  const Eigen::Quaterniond start = rpy(0, 0, 0.2);
  f.initialize(start.conjugate() * Eigen::Vector3d(0, 0, kG), start.conjugate() * kSun, kSun);
  const double rate = 0.3;
  const double dt = 0.02;
  for (int k = 0; k < 500; ++k) {
    const Eigen::Quaterniond truth = rpy(0, 0, 0.2 + rate * (k + 1) * dt);
    f.propagate({0, 0, rate}, truth.conjugate() * Eigen::Vector3d(0, 0, kG), dt);
  }
  EXPECT_LT(angleBetween(f.orientation(), rpy(0, 0, 0.2 + rate * 10.0)), 1e-6);
}

TEST(DeadReckoning, VisualOdometryReplacesWheelMotionAndRevealsSlip)
{
  DeadReckoning dr;
  SlipEstimator slip(0.5, 0.2, 5.0);
  const Eigen::Quaterniond heading = rpy(0, 0, M_PI / 2);  // driving north
  const double dt = 0.02;
  double t = 0.0;
  dr.addWheelMotion(t, 0.0, heading);
  for (int frame = 0; frame < 25; ++frame) {
    const double t0 = t;
    for (int k = 0; k < 10; ++k) {
      t += dt;
      dr.addWheelMotion(t, 0.005, heading);  // wheels say 0.25 m/s
    }
    // The camera sees the rover move half as far: 50 % slip.
    const auto update = dr.applyVisualOdometry(t0, t, Eigen::Vector3d(0.025, 0.0, 0.0));
    ASSERT_TRUE(update.has_value());
    EXPECT_NEAR(update->wheel_distance, 0.05, 1e-9);
    slip.add(t, update->wheel_distance, update->vo_distance);
  }
  EXPECT_NEAR(dr.position().x(), 0.0, 1e-9);
  EXPECT_NEAR(dr.position().y(), 25 * 0.025, 1e-9);
  ASSERT_TRUE(slip.slip(t).has_value());
  EXPECT_NEAR(*slip.slip(t), 0.5, 1e-9);

  // Motion after the last VO frame is still wheel odometry.
  dr.addWheelMotion(t + dt, 0.005, heading);
  EXPECT_NEAR(dr.position().y(), 25 * 0.025 + 0.005, 1e-9);

  // An interval the history does not cover is rejected.
  EXPECT_FALSE(dr.applyVisualOdometry(t - 100.0, t, Eigen::Vector3d::Zero()).has_value());
}

TEST(DeadReckoning, VisualOdometryCannotLiftTheRoverOffItsDeckPlane)
{
  DeadReckoning dr;
  const Eigen::Quaterniond uphill = rpy(0, -0.2, 0);  // driving east, nose up ~11 deg
  double t = 0.0;
  dr.addWheelMotion(t, 0.0, uphill);
  for (int frame = 0; frame < 100; ++frame) {
    const double t0 = t;
    t += 0.2;
    dr.addWheelMotion(t, 0.05, uphill);
    // VO with a 1 cm/frame upward bias in the body frame (20 % of the forward motion).
    ASSERT_TRUE(dr.applyVisualOdometry(t0, t, Eigen::Vector3d(0.05, 0.0, 0.01)).has_value());
  }
  // 5 m up an 11 deg slope: the climb comes from the attitude, none from the VO bias.
  EXPECT_NEAR(dr.position().x(), 5.0 * std::cos(0.2), 1e-9);
  EXPECT_NEAR(dr.position().z(), 5.0 * std::sin(0.2), 1e-9);
}

TEST(SlipEstimator, NeedsDistanceAndForgetsOldSamples)
{
  SlipEstimator slip(0.5, 0.2, 5.0);
  slip.add(0.0, 0.1, 0.1);
  EXPECT_FALSE(slip.slip(0.0).has_value());
  slip.add(1.0, 0.2, 0.0);
  ASSERT_TRUE(slip.slip(1.0).has_value());
  EXPECT_NEAR(*slip.slip(1.0), 1.0 - 0.1 / 0.3, 1e-9);
  EXPECT_FALSE(slip.slip(100.0).has_value());
}

TEST(SlipEstimator, TurningInPlaceIsNotSlip)
{
  SlipEstimator slip(0.6, 0.25, 6.0);
  double t = 0.0;
  // A 360 deg turn in place: no net forward wheel travel, but VO sees a little forward noise.
  for (int k = 0; k < 50; ++k) slip.add(t += 0.2, 0.001, 0.02);
  EXPECT_FALSE(slip.slip(t).has_value());
  // Then clean driving: the turn must not show up as slip.
  for (int k = 0; k < 8; ++k) slip.add(t += 0.2, 0.04, 0.04);
  ASSERT_TRUE(slip.slip(t).has_value());
  EXPECT_NEAR(*slip.slip(t), 0.0, 1e-9);
}

TEST(SlipEstimator, SlidingBackIsNotRollingFine)
{
  // Wheels drive 0.3 m forward while the rover slides 0.3 m back: as large as distances go,
  // but slip must be 2, not 0.
  SlipEstimator slip(0.5, 0.2, 5.0);
  slip.add(1.0, 0.3, -0.3);
  ASSERT_TRUE(slip.slip(1.0).has_value());
  EXPECT_NEAR(*slip.slip(1.0), 2.0, 1e-9);

  // Backing out cleanly: both negative, no slip.
  SlipEstimator backing(0.5, 0.2, 5.0);
  backing.add(1.0, -0.3, -0.3);
  EXPECT_NEAR(*backing.slip(1.0), 0.0, 1e-9);
}

}  // namespace
}  // namespace rover_core
