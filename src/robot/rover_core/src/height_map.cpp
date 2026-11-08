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

#include "rover_core/height_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace rover_core
{

std::vector<Eigen::Vector3f> depthToPoints(
  const float * depth,
  int width,
  int height,
  const CameraIntrinsics & k,
  const Eigen::Isometry3d & target_from_optical,
  int stride,
  double min_depth,
  double max_depth)
{
  std::vector<Eigen::Vector3f> points;
  points.reserve(static_cast<size_t>(width / stride + 1) * (height / stride + 1));
  const Eigen::Isometry3f tf = target_from_optical.cast<float>();
  for (int v = 0; v < height; v += stride) {
    for (int u = 0; u < width; u += stride) {
      const float d = depth[static_cast<size_t>(v) * width + u];
      if (!std::isfinite(d) || d < min_depth || d > max_depth) continue;
      const Eigen::Vector3f optical((u - k.cx) * d / k.fx, (v - k.cy) * d / k.fy, d);
      points.push_back(tf * optical);
    }
  }
  return points;
}

HeightMap::HeightMap(double resolution, int size_cells)
: elevation_(
    resolution, -size_cells * resolution / 2.0, -size_cells * resolution / 2.0, size_cells, size_cells, kUnknown)
, frame_max_(static_cast<size_t>(size_cells) * size_cells, -std::numeric_limits<float>::infinity())
{}

void HeightMap::recenter(double x, double y, double margin)
{
  const double half = elevation_.width * elevation_.resolution / 2.0;
  const double cx = elevation_.origin_x + half;
  const double cy = elevation_.origin_y + half;
  if (std::abs(x - cx) < margin && std::abs(y - cy) < margin) return;

  const double res = elevation_.resolution;
  const int shift_i = static_cast<int>(std::lround((x - cx) / res));
  const int shift_j = static_cast<int>(std::lround((y - cy) / res));
  Grid<float> moved(
    res,
    elevation_.origin_x + shift_i * res,
    elevation_.origin_y + shift_j * res,
    elevation_.width,
    elevation_.height,
    kUnknown);
  for (int j = 0; j < moved.height; ++j) {
    for (int i = 0; i < moved.width; ++i) {
      if (elevation_.contains(i + shift_i, j + shift_j)) moved.at(i, j) = elevation_.at(i + shift_i, j + shift_j);
    }
  }
  elevation_ = std::move(moved);
}

void HeightMap::integrate(const std::vector<Eigen::Vector3f> & points)
{
  std::vector<size_t> touched;
  for (const auto & p : points) {
    int i;
    int j;
    if (!elevation_.cellOf(p.x(), p.y(), i, j)) continue;
    const size_t idx = static_cast<size_t>(j) * elevation_.width + i;
    if (std::isinf(frame_max_[idx])) touched.push_back(idx);
    frame_max_[idx] = std::max(frame_max_[idx], p.z());
  }
  for (size_t idx : touched) {
    elevation_.data[idx] = frame_max_[idx];
    frame_max_[idx] = -std::numeric_limits<float>::infinity();
  }
}

void HeightMap::clear()
{
  std::fill(elevation_.data.begin(), elevation_.data.end(), kUnknown);
}

void HeightMap::stampFootprint(const Eigen::Isometry3d & odom_from_base, double pivot_height, double radius)
{
  const Eigen::Vector3d up = odom_from_base.linear().col(2);
  const Eigen::Vector3d ground = odom_from_base * Eigen::Vector3d(0.0, 0.0, -pivot_height);
  const double res = elevation_.resolution;
  const int r = static_cast<int>(std::ceil(radius / res));
  int ci;
  int cj;
  elevation_.cellOf(ground.x(), ground.y(), ci, cj);
  for (int j = cj - r; j <= cj + r; ++j) {
    for (int i = ci - r; i <= ci + r; ++i) {
      if (!elevation_.contains(i, j) || !std::isnan(elevation_.at(i, j))) continue;
      const double dx = elevation_.cellX(i) - ground.x();
      const double dy = elevation_.cellY(j) - ground.y();
      if (dx * dx + dy * dy > radius * radius) continue;
      // Plane through `ground` with normal `up`: up . (p - ground) = 0.
      elevation_.at(i, j) = static_cast<float>(ground.z() - (up.x() * dx + up.y() * dy) / up.z());
    }
  }
}

Grid<float> maxFilter(const Grid<float> & g, int r, bool require_all_known)
{
  const float neg_inf = -std::numeric_limits<float>::infinity();
  // Separable: a row pass, then a column pass over the row results. `holes` tracks whether
  // any unknown cell (or the map edge) fell inside the window.
  Grid<float> rows(g.resolution, g.origin_x, g.origin_y, g.width, g.height, neg_inf);
  Grid<uint8_t> row_holes(g.resolution, g.origin_x, g.origin_y, g.width, g.height, 0);
  for (int j = 0; j < g.height; ++j) {
    for (int i = 0; i < g.width; ++i) {
      float m = neg_inf;
      bool hole = i - r < 0 || i + r >= g.width;
      for (int k = std::max(0, i - r); k <= std::min(g.width - 1, i + r); ++k) {
        const float v = g.at(k, j);
        if (std::isnan(v)) {
          hole = true;
        } else {
          m = std::max(m, v);
        }
      }
      rows.at(i, j) = m;
      row_holes.at(i, j) = hole;
    }
  }
  Grid<float> out(g.resolution, g.origin_x, g.origin_y, g.width, g.height, kUnknown);
  for (int j = 0; j < g.height; ++j) {
    for (int i = 0; i < g.width; ++i) {
      float m = neg_inf;
      bool hole = j - r < 0 || j + r >= g.height;
      for (int k = std::max(0, j - r); k <= std::min(g.height - 1, j + r); ++k) {
        m = std::max(m, rows.at(i, k));
        hole = hole || row_holes.at(i, k);
      }
      if (!std::isinf(m) && !(require_all_known && hole)) out.at(i, j) = m;
    }
  }
  return out;
}

}  // namespace rover_core
