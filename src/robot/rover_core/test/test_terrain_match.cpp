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

#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "rover_core/terrain_match.hpp"

namespace rover_core
{
namespace
{

const MatchConfig kConfig{2.0, 3.0 * M_PI / 180.0, 0.25, M_PI / 180.0, 0.05, 0.25 * M_PI / 180.0, 200, 0.05, 0.5, 2.0, 1.0};

// Smooth terrain: a sum of random Gaussian bumps, 0.25 m DEM over a 40 m square.
Grid<float> bumpyDem()
{
  Grid<float> dem(0.25, -20.0, -20.0, 160, 160, 0.0f);
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> pos(-20.0, 20.0);
  std::uniform_real_distribution<double> size(0.5, 2.5);
  std::uniform_real_distribution<double> amp(-0.4, 0.4);
  for (int b = 0; b < 120; ++b) {
    const double bx = pos(rng);
    const double by = pos(rng);
    const double s = size(rng);
    const double a = amp(rng);
    for (int j = 0; j < dem.height; ++j) {
      for (int i = 0; i < dem.width; ++i) {
        const double d2 = std::pow(dem.cellX(i) - bx, 2) + std::pow(dem.cellY(j) - by, 2);
        dem.at(i, j) += static_cast<float>(a * std::exp(-d2 / (2 * s * s)));
      }
    }
  }
  return dem;
}

// Local heights the rover would measure in its odom frame, given the true map_from_odom.
std::vector<TerrainPoint> observe(const Grid<float> & dem, const Pose2 & truth, double z_offset)
{
  std::vector<TerrainPoint> pts;
  for (double y = -6.0; y <= 6.0; y += 0.25) {
    for (double x = -6.0; x <= 6.0; x += 0.25) {
      double mx;
      double my;
      transformPoint(truth, x, y, mx, my);
      pts.push_back({x, y, bilinear(dem, mx, my) + z_offset});
    }
  }
  return pts;
}

TEST(TerrainMatch, RecoversAnOffsetOdomFrame)
{
  const Grid<float> dem = bumpyDem();
  const Pose2 truth{3.0, -2.0, 0.6};
  const Pose2 guess{3.0 + 0.8, -2.0 - 0.6, 0.6 + 0.02};
  const MatchResult r = matchTerrain(observe(dem, truth, 1.7), dem, guess, kConfig);
  ASSERT_TRUE(r.accepted) << "score " << r.score << " runner-up " << r.runner_up << " relief " << r.relief
                          << " at " << r.map_from_odom.x << ", " << r.map_from_odom.y;
  EXPECT_GT(r.score, 0.95);
  EXPECT_NEAR(r.map_from_odom.x, truth.x, 0.1);
  EXPECT_NEAR(r.map_from_odom.y, truth.y, 0.1);
  EXPECT_NEAR(r.map_from_odom.yaw, truth.yaw, 0.01);
}

TEST(TerrainMatch, RejectsFlatTerrain)
{
  const Grid<float> flat(0.25, -20.0, -20.0, 160, 160, 0.0f);
  const MatchResult r = matchTerrain(observe(flat, {0, 0, 0}, 0.0), flat, {0.5, 0.0, 0.0}, kConfig);
  EXPECT_FALSE(r.accepted);
}

TEST(TerrainMatch, TerrainPointsAverageObservedBlocks)
{
  Grid<float> elevation(0.05, 0.0, 0.0, 10, 10, kUnknown);
  for (int j = 0; j < 5; ++j) {
    for (int i = 0; i < 5; ++i) elevation.at(i, j) = static_cast<float>(i);
  }
  const auto pts = terrainPoints(elevation, 0.25);
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_NEAR(pts[0].x, 0.125, 1e-9);
  EXPECT_NEAR(pts[0].h, 2.0, 1e-6);
}

}  // namespace
}  // namespace rover_core
