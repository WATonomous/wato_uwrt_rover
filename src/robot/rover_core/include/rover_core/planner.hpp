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

#include "rover_core/ace.hpp"
#include "rover_core/grid.hpp"
#include "rover_core/params.hpp"

namespace rover_core
{

// ------------------------------------------------------------------ path tree

struct PathTreeConfig
{
  std::vector<double> turns;  // rad, turn-in-place before the arcs
  std::vector<double> curvatures;  // 1/m, per arc segment
  double segment_length;  // m
  double step;  // m between sampled poses along an arc
  double turn_step;  // rad between sampled headings during a turn in place
};

struct PathPose
{
  double x;
  double y;
  double yaw;
  double s;  // arc length from the end of the turn in place (0 during the turn)
};

// One candidate: turn in place by `turn`, then two arcs. Poses are in the rover frame.
struct CandidatePath
{
  double turn;
  double k1;
  double k2;
  std::vector<PathPose> poses;
  int turn_poses;  // poses[0 .. turn_poses) are the in-place turn
  int commit_end;  // poses[0 .. commit_end) are the turn plus the first arc (what gets driven)
  double length;
};

// The full tree is fixed geometry relative to the rover, so it is built once at startup.
std::vector<CandidatePath> buildPathTree(const PathTreeConfig & cfg);

// ------------------------------------------------------------ route costs

// Per-cell traversal cost in the map frame: 0 is easy, larger is harder, kLethal is forbidden.
struct CostConfig
{
  double slope_free;  // rad, slopes below this cost nothing
  double slope_lethal;  // rad, slopes above this are forbidden
  double step_free;  // m, local height steps below this cost nothing
  double step_lethal;  // m, local height steps above this are forbidden
  double cost_weight;  // traversal cost per metre = 1 + cost_weight * cell cost
};

// Slope-based cost from the orbital DEM. Edge cells are lethal so routes stay on the map.
Grid<float> orbitalCost(const Grid<float> & dem, const CostConfig & cfg);

// Step-height cost from the rover's own height map (odom frame), written into `observed`
// (map frame, NaN = not observed). Cells observed again overwrite earlier values.
void stampObservedCost(
  Grid<float> & observed, const Grid<float> & elevation, const Pose2 & map_from_odom, const CostConfig & cfg);

// Mark a disk as forbidden (keep-out zones from ground planners, hazards found by faults).
void stampKeepOut(Grid<float> & keep_out, double x, double y, double radius);

// Combined cost: keep-out wins, then the worse of orbital and observed cost.
Grid<float> combineCost(const Grid<float> & orbital, const Grid<float> & observed, const Grid<float> & keep_out);

// Dijkstra from the goal over an 8-connected grid: cost-to-go in metres-equivalent for every
// cell, kLethal where the goal is unreachable. One search answers every candidate endpoint.
Grid<float> costToGo(const Grid<float> & cost, double goal_x, double goal_y, double cost_weight);

// ------------------------------------------------------------------ planner

struct RankWeights
{
  double turn;  // m-equivalent per rad of turn in place
  double unknown;  // m-equivalent per unknown pose beyond the committed part
  double heading_change;  // m-equivalent per rad of difference from the previous plan
};

struct PlanInputs
{
  Pose2 rover;  // odom frame
  Pose2 map_from_odom;
  const Grid<float> * elevation;  // odom frame
  const Grid<float> * cost;  // map frame, combined cost
  const Grid<float> * cost_to_go;  // map frame
  bool has_previous;
  double previous_commit_yaw;  // odom-frame heading at the end of the previous commit
};

struct RankedPath
{
  int index;
  double cost;
};

struct PlanResult
{
  bool found{false};
  int index{-1};
  int candidates{0};  // paths that survived the cheap filters
  int ace_evaluations{0};
  std::vector<PathPose> poses;  // chosen path in the odom frame
  double commit_length{0.0};
  std::vector<RankedPath> ranked;  // all survivors, best first
  std::vector<int> ace_rejected;  // indices ACE rejected, in evaluation order
};

// Rank every candidate with cheap heuristics (route cost + cost-to-go at the endpoint), then
// run ACE on them best-first and return the first that passes.
PlanResult plan(
  const RoverParams & p, const std::vector<CandidatePath> & tree, const PlanInputs & in, const RankWeights & w,
  double cost_weight, int max_ace_paths);

// A candidate's poses transformed into the odom frame.
std::vector<PathPose> toOdom(const CandidatePath & path, const Pose2 & rover);

}  // namespace rover_core
