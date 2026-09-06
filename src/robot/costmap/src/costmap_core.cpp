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

  band_count_.resize(cells);
  ground_count_.resize(cells);

  inflation_radius_ = inflation_radius;
  inflation_cells_ = static_cast<int>(std::ceil(inflation_radius / resolution_));

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
  grid_x = static_cast<int>(std::floor((wx-origin_x_) / resolution_));
  grid_y = static_cast<int>(std::floor((wy-origin_y_) / resolution_));
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
  std::fill(band_count_.begin(), band_count_.end(), 0);
  std::fill(ground_count_.begin(), ground_count_.end(), 0);
}


//full sensor to chassis transform(not sure if theres a rotation even, TEST BY LOOKING AT MESSAGES)
//only rotation for chassis to world, and trip the yaw rotation 

//combines these two transforms together
tf2::Transform CostmapCore::makeSensorToLevel(
    const geometry_msgs::msg::TransformStamped & sensor_to_chassis,
    const geometry_msgs::msg::TransformStamped & chassis_to_world)
{
  //conver
  tf2::Transform sensor_to_chassis_tf;
  tf2::fromMsg(sensor_to_chassis.transform,sensor_to_chassis_tf);

  //pitch/roll only
  tf2::Quaternion q_cw;
  tf2::fromMsg(chassis_to_world.transform.rotation,q_cw);

  //extract roll/pitch/yaw
  double roll,pitch,yaw;
  tf2::Matrix3x3(q_cw).getRPY(roll,pitch,yaw);

  //find the quat where yaw is fixed at zero
  tf2::Quaternion q_level;
  q_level.setRPY(roll,pitch,0.0);

  //turn the above quat into a transform
  const tf2::Transform level(q_level,tf2::Vector3(0,0,0));//builds transform rot + translate from quat and vec

  return level * sensor_to_chassis_tf;
  //L(Sv) = (LS)v, associative, now just need to multiply this transform by the point vector

  //first transform 

}



void CostmapCore::accumulate(const sensor_msgs::msg::PointCloud2::SharedPtr & cloud,
                  const tf2::Transform & sensor_to_level)
{

  //stride of points, how many points are we checking
  const int stride = std::max(1,filter_.point_stride);
  //iterators to go through each point in cloud
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

  int point_index = 0;
  //iterate through each point
  for(;iter_x!=iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_index)
  {
    if(point_index%stride != 0)
    {
      continue;
    }

    //get values of xyz

    const float sx = *iter_x;
    const float sy = *iter_y;
    const float sz = *iter_z;
    //before doing transforms, filter for out of range and NaN

    //range calc
    float range_sq = sx*sx + sy*sy + sz*sz;

    if(!std::isfinite(sx) || !std::isfinite(sy) || !std::isfinite(sz) || range_sq < min_range_sq_ || range_sq > max_range_sq_)
      continue;

    //now apply the transformation to it
    tf2::Vector3 newPoint = sensor_to_level * tf2::Vector3(sx,sy,sz);

    //transforms are done, can calculate the index now

    //apply height filter
    float h = newPoint.z();

    //if bigger just throw out, if smaller needs to be added to ground
    if(h>filter_.band_high)
      continue;
  
    if (h < filter_.ground_floor) 
      continue;

    //from worldtomap index
    int grid_x,grid_y;
    if(!worldToMap(newPoint.x(),newPoint.y(), grid_x, grid_y))
      continue;

    size_t idx =  static_cast<size_t> (grid_y)*width_ + grid_x;

    if(h<filter_.band_low)
    {
      ground_count_[idx]++;
    }
    else //the point survived all the filters, add to band_count
    {
      band_count_[idx]++;
    }


  } 
}

//takes accumulators and builds a costmap using it
void CostmapCore::classify()
{
  const uint16_t min_obstacle =
    static_cast<uint16_t>(std::max(1, filter_.min_points_obstacle));
  const uint16_t min_free =
    static_cast<uint16_t>(std::max(1, filter_.min_points_free));

  for (size_t i = 0; i < costmap_data_->data.size(); ++i) {
    if (band_count_[i] >= min_obstacle) {
      costmap_data_->data[i] = 100;
    } else if (ground_count_[i] >= min_free) {
      costmap_data_->data[i] = 0;
    } else {
      costmap_data_->data[i] = -1;
    }
  }
}

void CostmapCore::updateCostmapFromPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud,
  const geometry_msgs::msg::TransformStamped & sensor_to_chassis,
  const geometry_msgs::msg::TransformStamped & chassis_to_world
)
{
  // A *local* costmap: what the camera can see right now. Persistence across
  // frames is map_memory's job.
  resetAccumulators();

  accumulate(cloud, makeSensorToLevel(sensor_to_chassis,chassis_to_world));
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
  std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0/0);

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
