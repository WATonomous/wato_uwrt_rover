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
#include <vector>

#include "rover_core/grid.hpp"

namespace rover_core
{

struct CameraIntrinsics
{
  double fx;
  double fy;
  double cx;
  double cy;
};

// Back-project every `stride`-th depth pixel into the target frame.
// depth: row-major metres along the optical axis; target_from_optical: optical frame pose.
std::vector<Eigen::Vector3f> depthToPoints(
  const float * depth,
  int width,
  int height,
  const CameraIntrinsics & k,
  const Eigen::Isometry3d & target_from_optical,
  int stride,
  double min_depth,
  double max_depth);

// Rover-centric 2.5D height map in the odom frame. Each cell stores the top surface height
// from the most recent frame that observed it ("overwrite on observe"), so slow odometry
// drift never smears old and new observations of the same rock together.
class HeightMap
{
public:
  HeightMap(double resolution, int size_cells);

  // Re-center the window on (x, y) when the rover moved more than `margin` from its center.
  void recenter(double x, double y, double margin);

  // Points in the odom frame from one depth frame.
  void integrate(const std::vector<Eigen::Vector3f> & points);

  // Mark never-observed cells within `radius` of the rover as the ground plane under it.
  // The rover is standing there, so that ground is traversable; this covers the blind
  // spot below the mast camera.
  void stampFootprint(const Eigen::Isometry3d & odom_from_base, double pivot_height, double radius);

  const Grid<float> & elevation() const
  {
    return elevation_;
  }

private:
  Grid<float> elevation_;
  std::vector<float> frame_max_;
};

// Max of known cells in a (2r + 1)^2 window around each cell; NaN where the window holds no
// known cell, or (require_all_known) any unknown cell. ACE uses these to bound wheel and
// belly contact heights in O(1) per lookup.
Grid<float> maxFilter(const Grid<float> & g, int r, bool require_all_known);

}  // namespace rover_core
