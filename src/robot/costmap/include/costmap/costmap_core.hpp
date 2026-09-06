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

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"

namespace robot
{

// Tuning for the point cloud -> height band -> occupancy pipeline.
// Distances in metres, angles in radians.
//
// Heights are measured in the LEVELED frame: robot-centred, yaw-aligned with the
// chassis, but with roll and pitch removed so z is a true height above the ground
// plane. See makeSensorToLevel(). None of the height numbers below mean anything
// in the camera's own frame.
struct TerrainFilter
{
  // --- Pre-transform rejects. Frame-independent, so they run before the matmul ---
  double min_range = 0.2;  // closer than this is noise / self-returns
  double max_range = 5.0;  // Deliberately short. Height error from residual tilt
                           // grows as r*sin(theta), and depth noise as ~2% of r,
                           // so distant ground drifts into the band and reads as
                           // an obstacle. We drive forward; map_memory accumulates.

  // --- The height band. This is the whole classifier. ---
  //
  //   h > band_high      -> overhead (lintel, branch). We pass under it. Discard.
  //   band_low..band_high -> we would collide with it.  -> band_count_
  //   ground_floor..band_low -> floor, or debris we roll over. -> ground_count_
  //   h < ground_floor   -> depth noise, or the far side of a drop-off. Discard.
  //
  // band_low is a TRAVERSABILITY threshold, not a wheel dimension: it is the
  // tallest thing the rover can actually drive over, i.e.
  //   min(max climbable step, chassis ground clearance).
  // For a rigid diff-drive that is roughly 0.3-0.5x wheel radius, NOT the wheel
  // top -- a curb at wheel height is not climbable. Take the real numbers from
  // the URDF.
  double band_low = 0.10;
  double band_high = 0.60;   // rover height + margin
  double ground_floor = -0.50;  // below this we do not believe the point at all.
                                // Without this floor, a point 2 m down (a ledge,
                                // or a bad return) increments ground_count_ and
                                // the cell reads as confirmed free space.

  // Optional: widen the band's lower bound with range, to absorb residual tilt
  // that leveling does not remove (extrinsic calibration error, real terrain
  // slope). Effective bound becomes band_low + range * band_low_slack.
  // tan(3 deg) ~= 0.052. Leave at 0.0 to disable.
  double band_low_slack = 0.0;

  // --- Cell thresholds. Asymmetric on purpose. ---
  // Fewer points needed to believe an obstacle than to believe free space:
  // a false obstacle costs a detour, a false clear costs a collision.
  int min_points_obstacle = 3;
  int min_points_free = 6;

  int point_stride = 1;  // process every nth point; raise to cut per-frame cost
};

class CostmapCore
{
public:
  // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
  explicit CostmapCore(const rclcpp::Logger & logger);

  // Initializes the Costmap with the parameters that we get from the params.yaml
  void initCostmap(double resolution, int width, int height,
                   geometry_msgs::msg::Pose origin, double inflation_radius);

  void setTerrainFilter(const TerrainFilter & filter);

  // The main entry point.
  //
  //   sensor_to_chassis  cloud frame -> chassis. Static, from the URDF.
  //   chassis_to_world   chassis -> a GRAVITY-ALIGNED frame (sim_world in sim,
  //                      odom from the EKF on the real rover). Only its rotation
  //                      is used, and only the roll/pitch part of that -- see
  //                      makeSensorToLevel().
  //
  // Both must be looked up at cloud->header.stamp, not at TimePointZero. At 30 Hz
  // with the rover pitching, using the latest transform reintroduces exactly the
  // tilt error the leveling step exists to remove.
  //
  // The resulting grid is in the LEVELED frame, so the caller must publish it with
  // header.frame_id set to that frame -- NOT copied from the cloud's header, which
  // would place the grid at the camera's pose.
  void updateCostmapFromPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud,
    const geometry_msgs::msg::TransformStamped & sensor_to_chassis,
    const geometry_msgs::msg::TransformStamped & chassis_to_world);

  // Legacy 2D lidar path. No height information at all, so it stays a plain
  // hit-and-inflate: everything a beam returns is by definition at beam height.
  void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan);

  // Retrieves costmap data
  nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;

private:
  // Composes the one transform the per-point loop needs:
  //
  //     sensor -> leveled  =  R_level * (sensor -> chassis)
  //
  // where R_level is chassis_to_world's rotation with the YAW STRIPPED. Keeping
  // yaw would leave the grid world-aligned instead of robot-aligned, and
  // map_memory's own yaw rotation would then be applied twice. Translation of
  // chassis_to_world is dropped entirely -- that is the robot's world position,
  // and the local costmap stays centred on the robot.
  //
  // Because tf2 composes as Rz(yaw)*Ry(pitch)*Rx(roll), stripping yaw is exactly
  // setRPY(roll, pitch, 0) -- no explicit inverse needed.
  //
  // Composed ONCE per cloud so the hot loop stays a single transform * Vector3.
  static tf2::Transform makeSensorToLevel(
    const geometry_msgs::msg::TransformStamped & sensor_to_chassis,
    const geometry_msgs::msg::TransformStamped & chassis_to_world);

  // Pass 1, over POINTS. One loop, and the transform is the pivot inside it:
  //
  //   1. read x,y,z                          (sensor frame)
  //   2. NaN reject                          } frame-independent, so they run
  //   3. range reject (norm is rotation-      } before the matmul
  //      invariant, sensor is at its own origin)
  //   4. transform -> p                      <-- pivot
  //   5. h = p.z(), height bin                (leveled frame)
  //   6. idx from p.x(), p.y()                (leveled frame)
  //   7. ++band_count_[idx] or ++ground_count_[idx]
  //
  // Steps 5-7 CANNOT move above step 4: in the sensor frame z is depth along the
  // optical axis, not height, and x,y are not ground-plane coordinates. The index
  // is computed exactly once, from transformed coordinates -- there is no earlier
  // index. Step 5 goes before 6 only because two comparisons are cheaper than the
  // floor/divide in worldToMap.
  void accumulate(const sensor_msgs::msg::PointCloud2::SharedPtr & cloud,
                  const tf2::Transform & sensor_to_level);

  // Pass 2, over CELLS. Reads only the two count arrays; touches no points.
  // Separate from accumulate() because a cell's verdict depends on every point
  // that will land in it -- the first point may be floor and the tenth a wall,
  // so nothing can be decided until the point loop has finished.
  //
  //   band_count   >= min_points_obstacle -> 100  (lethal)
  //   ground_count >= min_points_free     ->   0  (confirmed free)
  //   otherwise                           ->  -1  (unknown)
  //
  // The -1 is load-bearing. A cell with no returns is a sensor shadow, a black
  // surface, or outside the FOV -- not free space. map_memory skips negatives, so
  // -1 means "no opinion this frame" rather than a claim about the world.
  void classify();

  void inflateObstacles();
  void inflateObstacle(int origin_x, int origin_y);

  void resetAccumulators();

  // Metric coords in the LEVELED frame -> grid indices. (Name is historical; it
  // has never taken world coordinates.) Returns false if outside the grid.
  bool worldToMap(double wx, double wy, int & grid_x, int & grid_y) const;

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
  rclcpp::Logger logger_;

  // Per-cell scratch. Same length and indexing as the occupancy grid (y*width + x).
  // Never published; dead the moment classify() has read them.
  std::vector<uint16_t> band_count_;    // points inside the collision band
  std::vector<uint16_t> ground_count_;  // points below it -> evidence of floor

  // Cached from initCostmap so the per-point loop touches no message fields and
  // does no uint32 -> int conversions.
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;

  double inflation_radius_ = 0.0;
  int inflation_cells_ = 0;

  TerrainFilter filter_;

  // Squared once in setTerrainFilter() so the hot loop never calls sqrt().
  double min_range_sq_ = 0.0;
  double max_range_sq_ = 0.0;
};

}  // namespace robot

#endif

/*
The plan:

Once per cloud:
  0. Look up sensor->chassis and chassis->world AT THE CLOUD'S STAMP.
     Strip yaw from chassis->world, keep roll/pitch, drop translation.
     Compose into one sensor->leveled transform.

Pass 1, per point (accumulate):
  1. NaN reject          -- frame-independent
  2. range reject        -- frame-independent
  3. TRANSFORM           -- pivot; only past here does z mean height
  4. height band test    -- obstacle / ground / discard
  5. grid index          -- from transformed x,y, computed once
  6. bin into band_count_ or ground_count_

Pass 2, per cell (classify):
  7. counts -> 100 / 0 / -1

Pass 3, per cell (inflateObstacles):
  8. circular inflation around lethal cells

Only NaN and range can precede the transform. The band test and the index cannot:
both need coordinates that only exist after it.
*/
