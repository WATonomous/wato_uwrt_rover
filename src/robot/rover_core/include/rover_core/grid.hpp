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

#include <cmath>
#include <limits>
#include <vector>

namespace rover_core
{

constexpr float kUnknown = std::numeric_limits<float>::quiet_NaN();
constexpr float kLethal = std::numeric_limits<float>::infinity();

// 2D rover pose in a planar frame (x, y, yaw).
struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// Heading of any quaternion type with x, y, z, w members (ROS messages, tf2).
template <typename Quaternion>
double yawOf(const Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Apply `frame` (pose of a child frame in a parent frame) to a point in the child frame.
inline void transformPoint(const Pose2 & frame, double x, double y, double & out_x, double & out_y)
{
  const double c = std::cos(frame.yaw);
  const double s = std::sin(frame.yaw);
  out_x = frame.x + c * x - s * y;
  out_y = frame.y + s * x + c * y;
}

inline Pose2 compose(const Pose2 & a, const Pose2 & b)
{
  Pose2 out;
  transformPoint(a, b.x, b.y, out.x, out.y);
  out.yaw = wrapAngle(a.yaw + b.yaw);
  return out;
}

inline Pose2 inverse(const Pose2 & p)
{
  const double c = std::cos(p.yaw);
  const double s = std::sin(p.yaw);
  return {-c * p.x - s * p.y, s * p.x - c * p.y, -p.yaw};
}

// Axis-aligned regular grid. Cell (i, j) spans [origin_x + i * res, origin_x + (i + 1) * res)
// in x (same for y); data is row-major with j as the row.
template <typename T>
struct Grid
{
  double resolution{1.0};
  double origin_x{0.0};
  double origin_y{0.0};
  int width{0};
  int height{0};
  std::vector<T> data;

  Grid() = default;

  Grid(double res, double ox, double oy, int w, int h, T fill)
  : resolution(res)
  , origin_x(ox)
  , origin_y(oy)
  , width(w)
  , height(h)
  , data(static_cast<size_t>(w) * h, fill)
  {}

  bool contains(int i, int j) const
  {
    return i >= 0 && j >= 0 && i < width && j < height;
  }

  T & at(int i, int j)
  {
    return data[static_cast<size_t>(j) * width + i];
  }

  const T & at(int i, int j) const
  {
    return data[static_cast<size_t>(j) * width + i];
  }

  // Cell containing (x, y); returns false if it is outside the grid.
  bool cellOf(double x, double y, int & i, int & j) const
  {
    i = static_cast<int>(std::floor((x - origin_x) / resolution));
    j = static_cast<int>(std::floor((y - origin_y) / resolution));
    return contains(i, j);
  }

  double cellX(int i) const
  {
    return origin_x + (i + 0.5) * resolution;
  }

  double cellY(int j) const
  {
    return origin_y + (j + 0.5) * resolution;
  }

  // Value at (x, y), or `outside` when (x, y) is off the grid.
  T valueAt(double x, double y, T outside) const
  {
    int i;
    int j;
    return cellOf(x, y, i, j) ? at(i, j) : outside;
  }
};

// The square of cells within `half` metres of (x, y), at the same world coordinates; cells that
// fall off `g` are `fill`.
template <typename T>
Grid<T> crop(const Grid<T> & g, double x, double y, double half, T fill)
{
  const int r = static_cast<int>(std::ceil(half / g.resolution));
  const int ci = static_cast<int>(std::floor((x - g.origin_x) / g.resolution));
  const int cj = static_cast<int>(std::floor((y - g.origin_y) / g.resolution));
  Grid<T> out(
    g.resolution,
    g.origin_x + (ci - r) * g.resolution,
    g.origin_y + (cj - r) * g.resolution,
    2 * r + 1,
    2 * r + 1,
    fill);
  for (int j = 0; j < out.height; ++j) {
    for (int i = 0; i < out.width; ++i) {
      if (g.contains(ci - r + i, cj - r + j)) out.at(i, j) = g.at(ci - r + i, cj - r + j);
    }
  }
  return out;
}

// Bilinear interpolation between cell centers. Returns NaN off-grid or next to unknown cells.
inline float bilinear(const Grid<float> & g, double x, double y)
{
  const double fx = (x - g.origin_x) / g.resolution - 0.5;
  const double fy = (y - g.origin_y) / g.resolution - 0.5;
  const int i = static_cast<int>(std::floor(fx));
  const int j = static_cast<int>(std::floor(fy));
  if (i < 0 || j < 0 || i + 1 >= g.width || j + 1 >= g.height) return kUnknown;
  const double tx = fx - i;
  const double ty = fy - j;
  const double v = (g.at(i, j) * (1 - tx) + g.at(i + 1, j) * tx) * (1 - ty) +
                   (g.at(i, j + 1) * (1 - tx) + g.at(i + 1, j + 1) * tx) * ty;
  return static_cast<float>(v);
}

}  // namespace rover_core
