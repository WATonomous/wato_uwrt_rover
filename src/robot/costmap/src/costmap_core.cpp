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
#include "costmap/costmap_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger & logger)
: costmap_data_(std::make_shared<nav_msgs::msg::OccupancyGrid>())
, logger_(logger)
{}

void CostmapCore::initCostmap(
  double resolution, int width, int height, geometry_msgs::msg::Pose origin, double inflation_radius)
{
  costmap_data_->info.resolution = resolution;
  costmap_data_->info.width = width;
  costmap_data_->info.height = height;
  costmap_data_->info.origin = origin;

  const size_t cells = static_cast<size_t>(width) * height;
  costmap_data_->data.assign(cells, -1);

  // Cached so the per-point loop touches no message fields and does no
  // uint32 -> int conversions
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  origin_x_ = origin.position.x;
  origin_y_ = origin.position.y;

  count_.resize(cells);
  sum_z_.resize(cells);
  sum_z2_.resize(cells);
  min_z_.resize(cells);
  max_z_.resize(cells);
  mean_z_.resize(cells);

  inflation_radius_ = inflation_radius;
  inflation_cells_ = static_cast<int>(std::ceil(inflation_radius / resolution));

  setTerrainFilter(filter_);

  RCLCPP_INFO(
    logger_, "Costmap initialized with resolution: %.2f, width: %d, height: %d", resolution, width, height);
}

void CostmapCore::setTerrainFilter(const TerrainFilter & filter)
{
  filter_ = filter;
  // Squared once here so the hot loop never calls sqrt()
  min_range_sq_ = filter_.min_range * filter_.min_range;
  max_range_sq_ = filter_.max_range * filter_.max_range;
}

bool CostmapCore::worldToMap(double wx, double wy, int & grid_x, int & grid_y) const
{
  grid_x = static_cast<int>(std::floor(wx-origin_x / resolution_));
  grid_y = static_cast<int>(std::floor(yw-origin_y / resolution));
  //in grid range check
  return grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_;
}

/* CostmapCore::mapToWorld(double wx, double wy, int & grid_x, int & grid_y) const
{
  wx = grid_x * resolution_ + origin_x;
  wy = grid_y * resolution + origin_y;
}
 */
void CostmapCore::resetAccumulators()
{
  std::fill(count_.begin(), count_.end(), 0);
  std::fill(sum_z_.begin(), sum_z_.end(), 0.0);
  std::fill(sum_z2_.begin(), sum_z2_.end(), 0.0);
  // Sentinels, so the first real sample wins both comparisons. Note lowest(),
  // not FLT_MIN -- FLT_MIN is the smallest *positive* float.
  std::fill(min_z_.begin(), min_z_.end(), std::numeric_limits<float>::max());
  std::fill(max_z_.begin(), max_z_.end(), std::numeric_limits<float>::lowest());
}

/*populate the accumulators
count - number of points in a grid index
sum_z - sum of heights,
sum_z - sum of heights squared
min_z_ - the smallest height 
min_z_ - largest height
*/
void CostmapCore::accumulate(
  const sensor_msgs::msg::PointCloud2::SharedPtr & cloud,
  const geometry_msgs::msg::TransformStamped & sensor_to_target)
{
  // obtain rotation matrix from sensor to chassis frame
  tf2::Transform sensor_to_target_tf;
  tf2::fromMsg(sensor_to_target.transform, sensor_to_target_tf);

  const int stride = std::max(1, filter_.point_stride);

  //iterators to go through each point in the point cloud
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

  int point_index = 0;

  //iterate through each point
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_index) {
    if (point_index % stride != 0) {
      continue;
    }

    //x,y,z values for point
    const float sx = *iter_x;
    const float sy = *iter_y;
    const float sz = *iter_z;

    // NaN for failed points, filter out
    if (!std::isfinite(sx) || !std::isfinite(sy) || !std::isfinite(sz)) {
      continue;
    }
    const double range_sq = static_cast<double>(sx) * sx +
                            static_cast<double>(sy) * sy +
                            static_cast<double>(sz) * sz;
    if (range_sq < min_range_sq_ || range_sq > max_range_sq_) {
      continue;
    }

    // Into the target frame. Only past this line do x/y lie in the ground plane
    // and z mean "height" -- in the camera's own frame they do not.
    //transform from sensor to chassis frame, (3x3) matmul (3x1)
    const tf2::Vector3 p = sensor_to_target_tf * tf2::Vector3(sx, sy, sz);

    // A SANITY gate, not a ground classifier: this only drops the ceiling and
    // filters out too high and too low points
    const double h = p.z();
    if (h < filter_.sanity_min_z || h > filter_.sanity_max_z) {
      continue;
    }

    // Collapse 3D -> 2D: drop z, let the point vote for the cell its vertical
    // shadow lands in, and fold its height into that cell's statistics.
    int grid_x, grid_y;
    if (!worldToMap(p.x(), p.y(), grid_x, grid_y)) {
      continue;
    }
    //find associated index in the occupancy grid(y*width + x)
    const size_t idx = static_cast<size_t>(grid_y) * width_ + grid_x;

    //updating accumulators
    if (count_[idx] < std::numeric_limits<uint16_t>::max()) {
      ++count_[idx];
    }
    sum_z_[idx] += h;
    sum_z2_[idx] += h * h;
    min_z_[idx] = std::min(min_z_[idx], static_cast<float>(h));
    max_z_[idx] = std::max(max_z_[idx], static_cast<float>(h));
  }
}


bool CostmapCore::fitLocalPlane(int cx, int cy, double & a, double & b, double & c) const
{
  // Least squares for z = a*dx + b*dy + c, with (dx, dy) in METRES relative to
  // the cell being classified. Centring matters: it makes `c` the fitted ground
  // height exactly AT this cell, so the residual test needs no plane evaluation.
  const int half = std::max(1, filter_.fit_window / 2);
  const uint16_t min_points = static_cast<uint16_t>(std::max(1, filter_.min_points_per_cell));

  // Symmetric normal-equation accumulators
  double s_xx = 0, s_xy = 0, s_yy = 0, s_x = 0, s_y = 0, s_1 = 0;
  double s_xz = 0, s_yz = 0, s_z = 0;
  int used = 0;

  for (int ny = std::max(0, cy - half); ny <= std::min(height_ - 1, cy + half); ++ny) {
    for (int nx = std::max(0, cx - half); nx <= std::min(width_ - 1, cx + half); ++nx) {
      // Exclude the centre. If an obstacle sits here, letting it into the fit
      // would drag the ground estimate up toward the very thing we are trying
      // to measure against.
      if (nx == cx && ny == cy) {
        continue;
      }
      const size_t n_idx = static_cast<size_t>(ny) * width_ + nx;
      if (count_[n_idx] < min_points) {
        continue;
      }

      const double dx = (nx - cx) * resolution_;
      const double dy = (ny - cy) * resolution_;
      const double z = mean_z_[n_idx];

      s_xx += dx * dx;  s_xy += dx * dy;  s_x += dx;
      s_yy += dy * dy;  s_y += dy;        s_1 += 1.0;
      s_xz += dx * z;   s_yz += dy * z;   s_z += z;
      ++used;
    }
  }

  if (used < filter_.min_fit_cells) {
    return false;  // not enough ground to say anything about
  }

  // Solve the symmetric 3x3 by Cramer's rule. A near-zero determinant means the
  // surviving neighbours are collinear (e.g. a single row of cells along a grid
  // edge) and no unique plane exists.
  const double det =
      s_xx * (s_yy * s_1 - s_y * s_y)
    - s_xy * (s_xy * s_1 - s_y * s_x)
    + s_x  * (s_xy * s_y - s_yy * s_x);

  if (std::fabs(det) < 1e-9) {
    return false;
  }

  a = (s_xz * (s_yy * s_1 - s_y * s_y)
     - s_xy * (s_yz * s_1 - s_y * s_z)
     + s_x  * (s_yz * s_y - s_yy * s_z)) / det;

  b = (s_xx * (s_yz * s_1 - s_y * s_z)
     - s_xz * (s_xy * s_1 - s_y * s_x)
     + s_x  * (s_xy * s_z - s_yz * s_x)) / det;

  c = (s_xx * (s_yy * s_z - s_yz * s_y)
     - s_xy * (s_xy * s_z - s_yz * s_x)
     + s_xz * (s_xy * s_y - s_yy * s_x)) / det;

  return true;
}

void CostmapCore::classify()
{
  const uint16_t min_points = static_cast<uint16_t>(std::max(1, filter_.min_points_per_cell));
  const size_t cells = count_.size();

  // --- Stage 2: sums -> means, once, because the plane fit reads each mean
  // up to fit_window^2 times ---
  for (size_t i = 0; i < cells; ++i) {
    mean_z_[i] = (count_[i] > 0) ? sum_z_[i] / count_[i] : 0.0;
  }

  // Extent that pure terrain tilt can explain across one cell, corner to corner
  const double cell_diagonal = resolution_ * std::sqrt(2.0);

  // --- Stage 4: classify ---
  for (size_t i = 0; i < cells; ++i) {
    // Never seen: leave unknown. map_memory skips negatives, so an unknown cell
    // is "no opinion this frame" rather than a claim of free space.
    if (count_[i] < min_points) {
      costmap_data_->data[i] = -1;
      continue;
    }

    const int cx = static_cast<int>(i) % width_;
    const int cy = static_cast<int>(i) / width_;
    const double extent = max_z_[i] - min_z_[i];

    double a, b, c;
    if (!fitLocalPlane(cx, cy, a, b, c)) {
      // No local ground reference. We can still trust the cell's own vertical
      // extent, but we must not call it free -- there is nothing to call it
      // free relative to.
      costmap_data_->data[i] = (extent > filter_.max_step) ? 100 : -1;
      continue;
    }

    // TEST 1 -- is the ground itself too steep to climb?
    const double slope = std::atan(std::hypot(a, b));
    if (slope > filter_.max_slope) {
      costmap_data_->data[i] = 100;
      continue;
    }

    // TEST 2 -- does something stick up out of the LOCAL ground surface?
    // Because the fit is centred, the ground height here is exactly `c`.
    // This is the test that survives slope: a curb on a ramp has a high
    // absolute height and a ramp-matching slope, but a 15 cm residual.
    const double residual = max_z_[i] - c;
    if (residual > filter_.max_step) {
      costmap_data_->data[i] = 100;
      continue;
    }

    // TEST 3 -- vertical structure inside this one cell.
    // Covers the case tests 1 and 2 miss: a wall filling the fit window, where
    // the means are uniformly high and flat, so the plane is level and the
    // residual is zero. Allowance is slope-compensated, otherwise at 0.4 m
    // cells a 20 deg ramp (0.146 m of rise) looks exactly like a 15 cm curb.
    const double allowed_extent = filter_.max_step + std::tan(slope) * cell_diagonal;
    if (extent > allowed_extent) {
      costmap_data_->data[i] = 100;
      continue;
    }

    // Traversable, but how comfortably? Height variance about the cell mean.
    const double mean = mean_z_[i];
    const double variance = std::max(0.0, sum_z2_[i] / count_[i] - mean * mean);
    const double roughness = std::sqrt(variance);

    const double span = std::max(1e-6, filter_.rough_lethal - filter_.rough_free);
    const double t = std::clamp((roughness - filter_.rough_free) / span, 0.0, 1.0);

    // Capped at 99: roughness must never reach 100, or inflateObstacles() would
    // seed inflation off gravel.
    costmap_data_->data[i] = static_cast<int8_t>(t * 99.0);
  }
}


void CostmapCore::updateCostmapFromPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud,
  const geometry_msgs::msg::TransformStamped & sensor_to_target)
{
  // A *local* costmap: what the camera can see right now. Persistence across
  // frames is map_memory's job.
  resetAccumulators();

  accumulate(cloud, sensor_to_target);
  classify();
  inflateObstacles();
}

void CostmapCore::inflateObstacles()
{
  if (inflation_cells_ <= 0 || inflation_radius_ <= 0.0) {
    return;
  }

  // Snapshot the lethal cells first, so inflation never seeds off a cell that
  // inflation itself just wrote.
  std::vector<int> lethal;
  for (size_t i = 0; i < costmap_data_->data.size(); ++i) {
    if (costmap_data_->data[i] == 100) {
      lethal.push_back(static_cast<int>(i));
    }
  }

  for (const int index : lethal) {
    inflateObstacle(index % width_, index / width_);
  }
}

void CostmapCore::inflateObstacle(int origin_x, int origin_y)
{
  // A bounded window, not a BFS over a freshly allocated width*height visited
  // grid: the footprint is (2r+1)^2 cells and never larger.
  const int x_min = std::max(0, origin_x - inflation_cells_);
  const int x_max = std::min(width_ - 1, origin_x + inflation_cells_);
  const int y_min = std::max(0, origin_y - inflation_cells_);
  const int y_max = std::min(height_ - 1, origin_y + inflation_cells_);

  for (int y = y_min; y <= y_max; ++y) {
    for (int x = x_min; x <= x_max; ++x) {
      const double distance = std::hypot(x - origin_x, y - origin_y) * resolution_;
      if (distance > inflation_radius_) {
        continue;  // circular footprint, not the square window
      }

      const int8_t cost = static_cast<int8_t>((1.0 - distance / inflation_radius_) * 100.0);
      int8_t & cell = costmap_data_->data[static_cast<size_t>(y) * width_ + x];
      if (cost > cell) {
        cell = cost;  // max-combine overlaps; also lifts unknown (-1) cells
      }
    }
  }
}

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan)
{
  // The 2D lidar path has no height information at all, so it stays a plain
  // hit-and-inflate. Everything a beam returns is by definition at beam height.
  std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0);

  double angle = laserscan->angle_min;
  for (size_t i = 0; i < laserscan->ranges.size(); ++i, angle += laserscan->angle_increment) {
    const double range = laserscan->ranges[i];
    if (range < laserscan->range_min || range > laserscan->range_max) {
      continue;
    }
    int grid_x, grid_y;
    if (worldToMap(range * std::cos(angle), range * std::sin(angle), grid_x, grid_y)) {
      costmap_data_->data[static_cast<size_t>(grid_y) * width_ + grid_x] = 100;
    }
  }

  inflateObstacles();
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmapData() const
{
  return costmap_data_;
}

}  // namespace robot
