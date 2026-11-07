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

#include "rover_core/planner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <utility>
#include <vector>

namespace rover_core
{

namespace
{

constexpr double kLethalCheckStart = 0.5;  // m along a path before lethal route cells reject it

PathPose arcPose(const PathPose & start, double k, double ds)
{
  if (std::abs(k) < 1e-9) {
    return {start.x + ds * std::cos(start.yaw), start.y + ds * std::sin(start.yaw), start.yaw, start.s + ds};
  }
  const double yaw = start.yaw + k * ds;
  return {
    start.x + (std::sin(yaw) - std::sin(start.yaw)) / k,
    start.y - (std::cos(yaw) - std::cos(start.yaw)) / k,
    yaw,
    start.s + ds};
}

void appendArc(std::vector<PathPose> & poses, double k, double length, double step)
{
  const PathPose start = poses.back();
  const int n = static_cast<int>(std::lround(length / step));
  for (int i = 1; i <= n; ++i) poses.push_back(arcPose(start, k, length * i / n));
}

}  // namespace

std::vector<CandidatePath> buildPathTree(const PathTreeConfig & cfg)
{
  std::vector<CandidatePath> tree;
  for (double turn : cfg.turns) {
    std::vector<PathPose> turn_poses;
    const int n = static_cast<int>(std::ceil(std::abs(turn) / cfg.turn_step));
    for (int k = 0; k < n; ++k) turn_poses.push_back({0.0, 0.0, turn * k / n, 0.0});
    for (double k1 : cfg.curvatures) {
      for (double k2 : cfg.curvatures) {
        CandidatePath path{turn, k1, k2, turn_poses, static_cast<int>(turn_poses.size()), 0, 2.0 * cfg.segment_length};
        path.poses.push_back({0.0, 0.0, turn, 0.0});
        appendArc(path.poses, k1, cfg.segment_length, cfg.step);
        path.commit_end = static_cast<int>(path.poses.size());
        appendArc(path.poses, k2, cfg.segment_length, cfg.step);
        tree.push_back(std::move(path));
      }
    }
  }
  return tree;
}

Grid<float> orbitalCost(const Grid<float> & dem, const CostConfig & cfg)
{
  Grid<float> cost(dem.resolution, dem.origin_x, dem.origin_y, dem.width, dem.height, kLethal);
  for (int j = 1; j + 1 < dem.height; ++j) {
    for (int i = 1; i + 1 < dem.width; ++i) {
      const double gx = (dem.at(i + 1, j) - dem.at(i - 1, j)) / (2.0 * dem.resolution);
      const double gy = (dem.at(i, j + 1) - dem.at(i, j - 1)) / (2.0 * dem.resolution);
      const double slope = std::atan(std::hypot(gx, gy));
      if (!std::isfinite(slope) || slope > cfg.slope_lethal) continue;
      cost.at(i, j) = static_cast<float>(std::max(0.0, (slope - cfg.slope_free) / (cfg.slope_lethal - cfg.slope_free)));
    }
  }
  return cost;
}

void stampObservedCost(
  Grid<float> & observed, const Grid<float> & elevation, const Pose2 & map_from_odom, const CostConfig & cfg)
{
  const int block = std::max(1, static_cast<int>(std::lround(observed.resolution / elevation.resolution)));
  const int pad = block / 2;
  for (int bj = 0; bj + block <= elevation.height; bj += block) {
    for (int bi = 0; bi + block <= elevation.width; bi += block) {
      int known = 0;
      for (int j = bj; j < bj + block; ++j) {
        for (int i = bi; i < bi + block; ++i) known += std::isnan(elevation.at(i, j)) ? 0 : 1;
      }
      if (2 * known < block * block) continue;

      float lo = kLethal;
      float hi = -kLethal;
      for (int j = std::max(0, bj - pad); j < std::min(elevation.height, bj + block + pad); ++j) {
        for (int i = std::max(0, bi - pad); i < std::min(elevation.width, bi + block + pad); ++i) {
          const float v = elevation.at(i, j);
          if (std::isnan(v)) continue;
          lo = std::min(lo, v);
          hi = std::max(hi, v);
        }
      }
      const double step = hi - lo;
      const float value =
        step > cfg.step_lethal
          ? kLethal
          : static_cast<float>(std::max(0.0, (step - cfg.step_free) / (cfg.step_lethal - cfg.step_free)));

      double mx;
      double my;
      transformPoint(
        map_from_odom,
        elevation.origin_x + (bi + block / 2.0) * elevation.resolution,
        elevation.origin_y + (bj + block / 2.0) * elevation.resolution,
        mx,
        my);
      int i;
      int j;
      if (observed.cellOf(mx, my, i, j)) observed.at(i, j) = value;
    }
  }
}

void stampKeepOut(Grid<float> & keep_out, double x, double y, double radius)
{
  for (int j = 0; j < keep_out.height; ++j) {
    for (int i = 0; i < keep_out.width; ++i) {
      if (std::hypot(keep_out.cellX(i) - x, keep_out.cellY(j) - y) <= radius) keep_out.at(i, j) = kLethal;
    }
  }
}

Grid<float> combineCost(const Grid<float> & orbital, const Grid<float> & observed, const Grid<float> & keep_out)
{
  Grid<float> cost = orbital;
  for (size_t k = 0; k < cost.data.size(); ++k) {
    if (std::isinf(keep_out.data[k])) {
      cost.data[k] = kLethal;
    } else if (!std::isnan(observed.data[k])) {
      cost.data[k] = std::max(cost.data[k], observed.data[k]);
    }
  }
  return cost;
}

Grid<float> costToGo(const Grid<float> & cost, double goal_x, double goal_y, double goal_radius, double cost_weight)
{
  Grid<float> ctg(cost.resolution, cost.origin_x, cost.origin_y, cost.width, cost.height, kLethal);
  using Item = std::pair<float, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;

  // Seed every passable cell within the goal radius (or the goal cell itself), so a rock
  // sitting exactly on the goal does not make the whole goal unreachable.
  const int r = static_cast<int>(std::ceil(goal_radius / cost.resolution));
  int gi;
  int gj;
  cost.cellOf(goal_x, goal_y, gi, gj);
  for (int j = gj - r; j <= gj + r; ++j) {
    for (int i = gi - r; i <= gi + r; ++i) {
      if (!cost.contains(i, j) || std::isinf(cost.at(i, j))) continue;
      const double d = std::hypot(cost.cellX(i) - goal_x, cost.cellY(j) - goal_y);
      if (d > std::max(goal_radius, cost.resolution)) continue;
      ctg.at(i, j) = static_cast<float>(d);
      open.push({ctg.at(i, j), j * cost.width + i});
    }
  }
  const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  while (!open.empty()) {
    const auto [d, idx] = open.top();
    open.pop();
    if (d > ctg.data[idx]) continue;
    const int i = idx % cost.width;
    const int j = idx / cost.width;
    for (int n = 0; n < 8; ++n) {
      const int ni = i + di[n];
      const int nj = j + dj[n];
      if (!cost.contains(ni, nj) || std::isinf(cost.at(ni, nj))) continue;
      const double dist = cost.resolution * (n < 4 ? 1.0 : M_SQRT2);
      const double step = dist * (1.0 + cost_weight * (cost.at(i, j) + cost.at(ni, nj)) / 2.0);
      const float nd = static_cast<float>(d + step);
      if (nd < ctg.at(ni, nj)) {
        ctg.at(ni, nj) = nd;
        open.push({nd, nj * cost.width + ni});
      }
    }
  }
  return ctg;
}

std::vector<PathPose> toOdom(const CandidatePath & path, const Pose2 & rover)
{
  std::vector<PathPose> out;
  out.reserve(path.poses.size());
  for (const auto & p : path.poses) {
    PathPose q;
    transformPoint(rover, p.x, p.y, q.x, q.y);
    q.yaw = wrapAngle(rover.yaw + p.yaw);
    q.s = p.s;
    out.push_back(q);
  }
  return out;
}

PlanResult plan(
  const RoverParams & p,
  const std::vector<CandidatePath> & tree,
  const PlanInputs & in,
  const RankWeights & w,
  double cost_weight,
  int max_ace_paths)
{
  PlanResult result;
  for (size_t idx = 0; idx < tree.size(); ++idx) {
    const CandidatePath & path = tree[idx];
    const std::vector<PathPose> poses = toOdom(path, in.rover);
    double total = w.turn * std::abs(path.turn);
    bool lethal = false;
    for (size_t k = path.turn_poses; k < poses.size(); ++k) {
      double mx;
      double my;
      transformPoint(in.map_from_odom, poses[k].x, poses[k].y, mx, my);
      const float c = in.cost->valueAt(mx, my, kLethal);
      // The rover may already stand in a coarse lethal cell (e.g. a blurred orbital rock);
      // ACE, not the route cost, decides whether it can drive out of it.
      if (std::isinf(c) && poses[k].s >= kLethalCheckStart) {
        lethal = true;
        break;
      }
      if (std::isinf(c)) continue;
      if (k > static_cast<size_t>(path.turn_poses)) {
        total += (poses[k].s - poses[k - 1].s) * (1.0 + cost_weight * c);
      }
      if (static_cast<int>(k) >= path.commit_end && std::isnan(in.elevation->valueAt(poses[k].x, poses[k].y, kUnknown)))
      {
        total += w.unknown;
      }
    }
    if (lethal) continue;

    double ex;
    double ey;
    transformPoint(in.map_from_odom, poses.back().x, poses.back().y, ex, ey);
    const float ctg = in.cost_to_go->valueAt(ex, ey, kLethal);
    if (std::isinf(ctg)) continue;
    total += ctg;
    if (in.has_previous) {
      total += w.heading_change * std::abs(wrapAngle(poses[path.commit_end - 1].yaw - in.previous_commit_yaw));
    }
    result.ranked.push_back({static_cast<int>(idx), total});
  }
  std::sort(result.ranked.begin(), result.ranked.end(), [](const RankedPath & a, const RankedPath & b) {
    return a.cost < b.cost;
  });
  result.candidates = static_cast<int>(result.ranked.size());

  const AceMaps maps = buildAceMaps(p, *in.elevation);
  for (const RankedPath & ranked : result.ranked) {
    if (result.ace_evaluations >= max_ace_paths) break;
    ++result.ace_evaluations;
    const CandidatePath & path = tree[ranked.index];
    const std::vector<PathPose> poses = toOdom(path, in.rover);
    AceResult verdict = AceResult::kSafe;
    // Pose 0 is where the rover already stands: its real suspension state is measured (and
    // guarded by fault protection), so only the poses it would move into are checked.
    for (size_t k = 1; k < poses.size(); ++k) {
      const AceResult r = evaluatePose(p, maps, {poses[k].x, poses[k].y, poses[k].yaw});
      if (r == AceResult::kSafe) continue;
      // Terrain beyond the committed part may still be unseen; it is checked on a later cycle.
      if (r != AceResult::kUnknown || static_cast<int>(k) < path.commit_end) verdict = r;
      break;
    }
    if (verdict != AceResult::kSafe) {
      result.ace_rejected.push_back(ranked.index);
      ++result.rejections[static_cast<size_t>(verdict)];
      continue;
    }
    result.found = true;
    result.index = ranked.index;
    result.poses.assign(poses.begin() + path.turn_poses, poses.end());
    result.commit_length = poses[path.commit_end - 1].s;
    return result;
  }
  return result;
}

}  // namespace rover_core
