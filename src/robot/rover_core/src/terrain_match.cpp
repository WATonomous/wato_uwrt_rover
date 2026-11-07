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
#include <array>
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

// Local heights with their best-fit plane removed. The regional slope correlates with itself
// at any offset, which flattens the score peak; what is left (rocks, crater rims, swales)
// is what pins the position down. `basis` is an orthonormal basis of {1, x, y} over the
// points, so projecting it out of the DEM samples removes their plane too.
struct Detrended
{
  std::vector<double> residual;  // heights minus plane, per point
  std::array<std::vector<double>, 3> basis;
  double norm;  // |residual|
};

Detrended detrend(const std::vector<TerrainPoint> & pts)
{
  const size_t n = pts.size();
  Detrended out;
  out.basis = {std::vector<double>(n, 1.0), std::vector<double>(n), std::vector<double>(n)};
  for (size_t i = 0; i < n; ++i) {
    out.basis[1][i] = pts[i].x;
    out.basis[2][i] = pts[i].y;
  }
  for (size_t k = 0; k < 3; ++k) {  // Gram-Schmidt
    for (size_t m = 0; m < k; ++m) {
      double dot = 0.0;
      for (size_t i = 0; i < n; ++i) dot += out.basis[k][i] * out.basis[m][i];
      for (size_t i = 0; i < n; ++i) out.basis[k][i] -= dot * out.basis[m][i];
    }
    double norm = 0.0;
    for (double v : out.basis[k]) norm += v * v;
    norm = std::sqrt(norm);
    for (double & v : out.basis[k]) v /= norm;
  }
  out.residual.resize(n);
  for (size_t i = 0; i < n; ++i) out.residual[i] = pts[i].h;
  for (const auto & q : out.basis) {
    double dot = 0.0;
    for (size_t i = 0; i < n; ++i) dot += q[i] * pts[i].h;
    for (size_t i = 0; i < n; ++i) out.residual[i] -= dot * q[i];
  }
  out.norm = 0.0;
  for (double v : out.residual) out.norm += v * v;
  out.norm = std::sqrt(out.norm);
  return out;
}

// Normalized cross-correlation between detrended local heights and detrended DEM samples
// under one candidate. Candidates that put any point off the DEM score -1.
double score(
  const std::vector<TerrainPoint> & pts,
  const Detrended & local,
  const Grid<float> & dem,
  const Pose2 & guess,
  double cx,
  double cy,
  const Candidate & c)
{
  const double cos_c = std::cos(c.dyaw);
  const double sin_c = std::sin(c.dyaw);
  double dd = 0.0;
  double rd = 0.0;
  std::array<double, 3> qd{0.0, 0.0, 0.0};
  for (size_t i = 0; i < pts.size(); ++i) {
    double gx;
    double gy;
    transformPoint(guess, pts[i].x, pts[i].y, gx, gy);
    const double mx = cx + cos_c * (gx - cx) - sin_c * (gy - cy) + c.dx;
    const double my = cy + sin_c * (gx - cx) + cos_c * (gy - cy) + c.dy;
    const double d = bilinear(dem, mx, my);
    if (std::isnan(d)) return -1.0;
    dd += d * d;
    rd += local.residual[i] * d;
    for (size_t k = 0; k < 3; ++k) qd[k] += local.basis[k][i] * d;
  }
  // |d - plane(d)|^2 = |d|^2 - sum_k (q_k . d)^2; the residual is already orthogonal to q_k.
  const double dem_norm2 = dd - qd[0] * qd[0] - qd[1] * qd[1] - qd[2] * qd[2];
  if (dem_norm2 <= 1e-12 || local.norm <= 1e-12) return -1.0;
  return rd / (local.norm * std::sqrt(dem_norm2));
}

std::vector<Candidate> search(
  const std::vector<TerrainPoint> & pts,
  const Detrended & local,
  const Grid<float> & dem,
  const Pose2 & guess,
  double cx,
  double cy,
  const Candidate & center,
  double xy_range,
  double xy_step,
  double yaw_range,
  double yaw_step)
{
  std::vector<Candidate> out;
  const int nxy = static_cast<int>(std::lround(xy_range / xy_step));
  const int nyaw = static_cast<int>(std::lround(yaw_range / yaw_step));
  for (int a = -nyaw; a <= nyaw; ++a) {
    for (int b = -nxy; b <= nxy; ++b) {
      for (int k = -nxy; k <= nxy; ++k) {
        Candidate c{center.dx + k * xy_step, center.dy + b * xy_step, center.dyaw + a * yaw_step, 0.0};
        c.score = score(pts, local, dem, guess, cx, cy, c);
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
         elevation.origin_y + (bj + block / 2.0) * elevation.resolution,
         sum / known});
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

  const Detrended local = detrend(points);
  result.relief = local.norm / std::sqrt(static_cast<double>(points.size()));
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
    points,
    local,
    dem,
    guess,
    cx,
    cy,
    {0, 0, 0, 0},
    cfg.search_xy,
    cfg.coarse_xy_step,
    cfg.search_yaw,
    cfg.coarse_yaw_step);
  const Candidate peak = best(coarse);
  result.runner_up = -1.0;
  for (const auto & c : coarse) {
    if (std::hypot(c.dx - peak.dx, c.dy - peak.dy) >= cfg.peak_separation) {
      result.runner_up = std::max(result.runner_up, c.score);
    }
  }
  const Candidate refined = best(search(
    points,
    local,
    dem,
    guess,
    cx,
    cy,
    peak,
    cfg.coarse_xy_step,
    cfg.fine_xy_step,
    cfg.coarse_yaw_step,
    cfg.fine_yaw_step));
  result.score = refined.score;
  result.accepted =
    result.score >= cfg.min_score && (1.0 - result.runner_up) >= cfg.min_peak_ratio * (1.0 - result.score);

  // p_map = R(dyaw) (guess(p) - c) + c + d  =>  compose into a single map_from_odom.
  const double cos_c = std::cos(refined.dyaw);
  const double sin_c = std::sin(refined.dyaw);
  const double tx = guess.x - cx;
  const double ty = guess.y - cy;
  result.map_from_odom = {
    cx + cos_c * tx - sin_c * ty + refined.dx,
    cy + sin_c * tx + cos_c * ty + refined.dy,
    wrapAngle(guess.yaw + refined.dyaw)};
  return result;
}

}  // namespace rover_core
