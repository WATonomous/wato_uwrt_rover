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

#include <vector>

#include "rover_core/grid.hpp"

namespace rover_core
{

// Global localization by matching the rover's local height map to the orbital DEM, the
// onboard version of what JPL's ground team does with HiRISE images after each drive.

struct TerrainPoint
{
  double x;  // odom frame
  double y;
  double h;
};

// Downsample the odom-frame height map to `resolution` cells (mean of known cells, needs at
// least half of each block observed).
std::vector<TerrainPoint> terrainPoints(const Grid<float> & elevation, double resolution);

struct MatchConfig
{
  double search_xy;  // m, +/- search range around the current estimate
  double search_yaw;  // rad
  double coarse_xy_step;
  double coarse_yaw_step;
  double fine_xy_step;
  double fine_yaw_step;
  int min_points;
  double min_relief;  // m, rms of local heights about their best-fit plane; flat ground cannot be matched
  double min_score;  // normalized cross-correlation of the best match
  // Distinctness: (1 - runner_up) / (1 - best) must reach this. Smooth terrain gives broad
  // correlation peaks, so a ratio of "misfits" separates true peaks better than a score margin.
  double min_peak_ratio;
  double peak_separation;  // m, the runner-up must be at least this far from the best
};

struct MatchResult
{
  bool accepted{false};
  Pose2 map_from_odom;  // corrected transform (valid when accepted)
  double score{0.0};
  double runner_up{0.0};
  int points{0};
  double relief{0.0};
};

// Search corrections (dx, dy, dyaw) to `guess` that maximise the normalized cross-correlation
// between local heights and the DEM, both with their best-fit plane removed. The yaw
// correction rotates about the points' centroid.
MatchResult matchTerrain(
  const std::vector<TerrainPoint> & points, const Grid<float> & dem, const Pose2 & guess, const MatchConfig & cfg);

}  // namespace rover_core
