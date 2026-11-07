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

#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "rover_core/params.hpp"
#include "visual_odometry/motion.hpp"

namespace
{

// Ground points seen by the navcam before and after a known rover motion must give back that
// motion, expressed in base_link.
TEST(Motion, RecoversBodyMotionFromGroundPoints)
{
  const auto rover = rover_core::loadRoverParams(rover_core::roverConfigPath());
  const Eigen::Isometry3d base_from_optical = rover.navcamOptical();
  const cv::Matx33d k(160, 0, 160, 0, 160, 120, 0, 0, 1);

  Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();  // base_link(t1) in base_link(t0)
  motion.translation() = Eigen::Vector3d(0.06, 0.01, 0.0);
  motion.linear() = Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  std::mt19937 rng(5);
  std::uniform_real_distribution<double> fwd(1.5, 6.0);
  std::uniform_real_distribution<double> side(-2.0, 2.0);
  std::uniform_real_distribution<double> bump(-0.1, 0.1);
  std::vector<cv::Point3f> previous;
  std::vector<cv::Point2f> current;
  const double ground = -rover.pivotHeight();
  for (int i = 0; i < 150; ++i) {
    const Eigen::Vector3d world(fwd(rng), side(rng), ground + bump(rng));  // in base_link(t0)
    const Eigen::Vector3d in_prev = base_from_optical.inverse() * world;
    const Eigen::Vector3d in_cur = (motion * base_from_optical).inverse() * world;
    if (in_prev.z() <= 0 || in_cur.z() <= 0) continue;
    const double u = k(0, 0) * in_cur.x() / in_cur.z() + k(0, 2);
    const double v = k(1, 1) * in_cur.y() / in_cur.z() + k(1, 2);
    if (u < 0 || u >= 320 || v < 0 || v >= 240) continue;
    previous.emplace_back(in_prev.x(), in_prev.y(), in_prev.z());
    current.emplace_back(u, v);
  }
  // A few gross outliers that RANSAC has to reject.
  for (int i = 0; i < 10; ++i) current[i] += cv::Point2f(25.0f, -18.0f);

  const auto estimate = visual_odometry::estimateMotion(previous, current, k, base_from_optical, 20);
  ASSERT_TRUE(estimate.has_value());
  EXPECT_LT((estimate->body_motion.translation() - motion.translation()).norm(), 2e-3);
  EXPECT_LT(Eigen::AngleAxisd(estimate->body_motion.linear().transpose() * motion.linear()).angle(), 1e-3);
  EXPECT_LE(estimate->inliers, static_cast<int>(current.size()) - 10);
}

}  // namespace
