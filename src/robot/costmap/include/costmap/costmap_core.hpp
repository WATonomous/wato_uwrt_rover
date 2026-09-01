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
#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace robot
{

// Tuning for the point cloud -> terrain -> occupancy pipeline.
// Distances in metres, angles in radians, heights in the costmap's target frame.
struct TerrainFilter
{
  //Stage 1: filter out point cloud before performing more expensive computation
  double min_range = 0.2; //if too close noisy
  double max_range = 8.0; //if too far also noisy/error
  double sanity_min_z = -2.0;// coarse gate ONLY: below this is impossible
  double sanity_max_z = 2.0;// above this is ceiling/canopy we drive under
  int min_points_per_cell = 4;//how many points lie in a cell for it to count as obstacle
  int point_stride = 1;//process every nth point, decrease compute time if needed

  // Stage 3: the local ground fit, seeing if point belongs to ground plane or obstacle
  int fit_window = 5;// k x k neighbourhood, odd; 5 spans 2.0 m at 0.4 m/cell
  int min_fit_cells = 6;// neighbours with data needed before the fit is trusted

  // --- Stage 4: classification ---
  double max_slope = 0.52;   // ~30 deg; steeper ground is not climbable
  double max_step = 0.12;    // vertical jump out of local ground that blocks us
  double rough_free = 0.02;  // height stddev at or below this is smooth
  double rough_lethal = 0.10;  // stddev at or above this is maximum soft cost
};

class CostmapCore
{
public:
  explicit CostmapCore(const rclcpp::Logger & logger);

  void initCostmap(double resolution, int width, int height,
                   geometry_msgs::msg::Pose origin, double inflation_radius);

  void setTerrainFilter(const TerrainFilter & filter);

  void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan);

  // `sensor_to_target` must map points from the cloud's frame into the costmap's
  // target frame. Heights are meaningless before it is applied.
  void updateCostmapFromPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud,
    const geometry_msgs::msg::TransformStamped & sensor_to_target);

  nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;

private:
  bool worldToMap(double wx, double wy, int & grid_x, int & grid_y) const;

  // Stage 1: fold every usable cloud point into the per-cell statistics
  void accumulate(const sensor_msgs::msg::PointCloud2::SharedPtr & cloud,
                  const geometry_msgs::msg::TransformStamped & sensor_to_target);

  // Stage 2 + 4: statistics -> costs
  void classify();

  // Stage 3: least-squares plane through neighbouring cell means, centre excluded.
  // Plane is z = a*dx + b*dy + c in metres relative to (cx, cy), so `c` is the
  // ground height AT the cell. Returns false if too few neighbours have data.
  bool fitLocalPlane(int cx, int cy, double & a, double & b, double & c) const;

  void inflateObstacles();
  void inflateObstacle(int origin_x, int origin_y);

  void resetAccumulators();

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
  rclcpp::Logger logger_;

  //Per-cell scratch. Same length and indexing as the occupancy grid.
  // Never published; dead the moment classify() has read it. ---
  std::vector<uint16_t> count_;   // points that landed in this cell
  std::vector<double> sum_z_;     // -> mean height
  std::vector<double> sum_z2_;    // with sum_z -> variance, i.e. roughness
  std::vector<float> min_z_;      // with max_z -> vertical extent
  std::vector<float> max_z_;
  std::vector<double> mean_z_;    // precomputed: read k*k times by the plane fit

  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;

  double inflation_radius_ = 0.0;
  int inflation_cells_ = 0;

  TerrainFilter filter_;
  double min_range_sq_ = 0.0;
  double max_range_sq_ = 0.0;
};

}  // namespace robot

#endif
