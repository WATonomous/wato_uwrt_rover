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

#include "rover_core/terrain_match.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace rover_core
{

namespace
{

struct Candidate
{
  double dx;
  double dy;
  double dyaw;
  double score;
};

// Normalized cross-correlation between the local heights and the DEM under one candidate.
double score(
  const std::vector<TerrainPoint> & pts, const Grid<float> & dem, const Pose2 & guess, double cx, double cy,
  const Candidate & c)
{
  const double cos_c = std::cos(c.dyaw);
  const double sin_c = std::sin(c.dyaw);
  double sum_h = 0.0;
  double sum_d = 0.0;
  double sum_hh = 0.0;
  double sum_dd = 0.0;
  double sum_hd = 0.0;
  int n = 0;
  for (const auto & p : pts) {
    double gx;
    double gy;
    transformPoint(guess, p.x, p.y, gx, gy);
    const double mx = cx + cos_c * (gx - cx) - sin_c * (gy - cy) + c.dx;
    const double my = cy + sin_c * (gx - cx) + cos_c * (gy - cy) + c.dy;
    const float d = bilinear(dem, mx, my);
    if (std::isnan(d)) continue;
    sum_h += p.h;
    sum_d += d;
    sum_hh += p.h * p.h;
    sum_dd += d * d;
    sum_hd += p.h * d;
    ++n;
  }
  if (n < 2 || n * 10 < static_cast<int>(pts.size()) * 9) return -1.0;
  const double var_h = sum_hh - sum_h * sum_h / n;
  const double var_d = sum_dd - sum_d * sum_d / n;
  if (var_h <= 1e-12 || var_d <= 1e-12) return -1.0;
  return (sum_hd - sum_h * sum_d / n) / std::sqrt(var_h * var_d);
}

std::vector<Candidate> search(
  const std::vector<TerrainPoint> & pts, const Grid<float> & dem, const Pose2 & guess, double cx, double cy,
  const Candidate & center, double xy_range, double xy_step, double yaw_range, double yaw_step)
{
  std::vector<Candidate> out;
  const int nxy = static_cast<int>(std::lround(xy_range / xy_step));
  const int nyaw = static_cast<int>(std::lround(yaw_range / yaw_step));
  for (int a = -nyaw; a <= nyaw; ++a) {
    for (int b = -nxy; b <= nxy; ++b) {
      for (int k = -nxy; k <= nxy; ++k) {
        Candidate c{center.dx + k * xy_step, center.dy + b * xy_step, center.dyaw + a * yaw_step, 0.0};
        c.score = score(pts, dem, guess, cx, cy, c);
        out.push_back(c);
      }
    }
  }
  return out;
}

const Candidate & best(const std::vector<Candidate> & cands)
{
  return *std::max_element(
    cands.begin(), cands.end(), [](const Candidate & a, const Candidate & b) { return a.score < b.score; });
}

}  // namespace

std::vector<TerrainPoint> terrainPoints(const Grid<float> & elevation, double resolution)
{
  std::vector<TerrainPoint> pts;
  const int block = std::max(1, static_cast<int>(std::lround(resolution / elevation.resolution)));
  for (int bj = 0; bj + block <= elevation.height; bj += block) {
    for (int bi = 0; bi + block <= elevation.width; bi += block) {
      double sum = 0.0;
      int known = 0;
      for (int j = bj; j < bj + block; ++j) {
        for (int i = bi; i < bi + block; ++i) {
          const float v = elevation.at(i, j);
          if (std::isnan(v)) continue;
          sum += v;
          ++known;
        }
      }
      if (2 * known < block * block) continue;
      pts.push_back(
        {elevation.origin_x + (bi + block / 2.0) * elevation.resolution,
         elevation.origin_y + (bj + block / 2.0) * elevation.resolution, sum / known});
    }
  }
  return pts;
}

MatchResult matchTerrain(
  const std::vector<TerrainPoint> & points, const Grid<float> & dem, const Pose2 & guess, const MatchConfig & cfg)
{
  MatchResult result;
  result.points = static_cast<int>(points.size());
  if (result.points < cfg.min_points) return result;

  double mean = 0.0;
  for (const auto & p : points) mean += p.h;
  mean /= points.size();
  double var = 0.0;
  for (const auto & p : points) var += (p.h - mean) * (p.h - mean);
  result.relief = std::sqrt(var / points.size());
  if (result.relief < cfg.min_relief) return result;

  double cx = 0.0;
  double cy = 0.0;
  for (const auto & p : points) {
    double gx;
    double gy;
    transformPoint(guess, p.x, p.y, gx, gy);
    cx += gx;
    cy += gy;
  }
  cx /= points.size();
  cy /= points.size();

  const std::vector<Candidate> coarse = search(
    points, dem, guess, cx, cy, {0, 0, 0, 0}, cfg.search_xy, cfg.coarse_xy_step, cfg.search_yaw, cfg.coarse_yaw_step);
  const Candidate peak = best(coarse);
  result.runner_up = -1.0;
  for (const auto & c : coarse) {
    if (std::hypot(c.dx - peak.dx, c.dy - peak.dy) >= cfg.peak_separation) {
      result.runner_up = std::max(result.runner_up, c.score);
    }
  }
  const Candidate refined = best(search(
    points, dem, guess, cx, cy, peak, cfg.coarse_xy_step, cfg.fine_xy_step, cfg.coarse_yaw_step, cfg.fine_yaw_step));
  result.score = refined.score;
  result.accepted =
    result.score >= cfg.min_score && (1.0 - result.runner_up) >= cfg.min_peak_ratio * (1.0 - result.score);

  // p_map = R(dyaw) (guess(p) - c) + c + d  =>  compose into a single map_from_odom.
  const double cos_c = std::cos(refined.dyaw);
  const double sin_c = std::sin(refined.dyaw);
  const double tx = guess.x - cx;
  const double ty = guess.y - cy;
  result.map_from_odom = {
    cx + cos_c * tx - sin_c * ty + refined.dx, cy + sin_c * tx + cos_c * ty + refined.dy,
    wrapAngle(guess.yaw + refined.dyaw)};
  return result;
}

}  // namespace rover_core
