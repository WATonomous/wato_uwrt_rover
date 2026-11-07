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

#include <Eigen/Geometry>
#include <optional>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace visual_odometry
{

struct MotionEstimate
{
  Eigen::Isometry3d body_motion;  // base_link at the current frame, in base_link at the previous frame
  int inliers;
};

// Rover motion between two frames from 3D points seen in the previous frame (previous optical
// frame, from depth) and where they appear in the current image. PnP gives the transform of
// previous-camera points into the current camera; inverting it and changing frames from the
// camera to base_link gives the body motion.
inline std::optional<MotionEstimate> estimateMotion(
  const std::vector<cv::Point3f> & previous_points,
  const std::vector<cv::Point2f> & current_pixels,
  const cv::Matx33d & k,
  const Eigen::Isometry3d & base_from_optical,
  int min_inliers)
{
  if (static_cast<int>(previous_points.size()) < min_inliers) return std::nullopt;
  cv::Mat rvec;
  cv::Mat tvec;
  std::vector<int> inliers;
  const bool ok = cv::solvePnPRansac(
    previous_points, current_pixels, k, cv::noArray(), rvec, tvec, false, 200, 2.0f, 0.999, inliers, cv::SOLVEPNP_EPNP);
  if (!ok || static_cast<int>(inliers.size()) < min_inliers) return std::nullopt;

  std::vector<cv::Point3f> in_points;
  std::vector<cv::Point2f> in_pixels;
  for (int i : inliers) {
    in_points.push_back(previous_points[i]);
    in_pixels.push_back(current_pixels[i]);
  }
  cv::solvePnP(in_points, in_pixels, k, cv::noArray(), rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);

  cv::Mat r;
  cv::Rodrigues(rvec, r);
  Eigen::Isometry3d current_from_previous = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) current_from_previous.linear()(i, j) = r.at<double>(i, j);
    current_from_previous.translation()(i) = tvec.at<double>(i);
  }
  const Eigen::Isometry3d camera_motion = current_from_previous.inverse();
  return MotionEstimate{
    base_from_optical * camera_motion * base_from_optical.inverse(), static_cast<int>(inliers.size())};
}

}  // namespace visual_odometry
