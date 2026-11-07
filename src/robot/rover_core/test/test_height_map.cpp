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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "rover_core/height_map.hpp"
#include "rover_core/params.hpp"

namespace rover_core
{
namespace
{

TEST(HeightMap, LatestObservationWins)
{
  HeightMap map(0.1, 100);
  map.integrate({{0.02f, 0.02f, 0.3f}, {0.07f, 0.03f, 0.5f}});
  EXPECT_FLOAT_EQ(map.elevation().valueAt(0.05, 0.05, kUnknown), 0.5f);  // max within one frame
  map.integrate({{0.05f, 0.05f, 0.1f}});
  EXPECT_FLOAT_EQ(map.elevation().valueAt(0.05, 0.05, kUnknown), 0.1f);  // newer frame overwrites
  EXPECT_TRUE(std::isnan(map.elevation().valueAt(1.05, 0.05, 0.0f)));
}

TEST(HeightMap, CropKeepsWorldCoordinates)
{
  Grid<float> g(0.1, -5.0, -5.0, 100, 100, 0.0f);
  g.at(60, 55) = 1.0f;  // cell centre (1.05, 0.55)
  const Grid<float> c = crop(g, 1.0, 0.5, 0.5, kUnknown);
  EXPECT_EQ(c.width, 11);
  EXPECT_FLOAT_EQ(c.valueAt(1.05, 0.55, kUnknown), 1.0f);
  EXPECT_FLOAT_EQ(c.valueAt(0.75, 0.25, kUnknown), 0.0f);
  EXPECT_FLOAT_EQ(c.valueAt(2.0, 0.5, -1.0f), -1.0f);  // outside the crop
  const Grid<float> edge = crop(g, 4.95, 4.95, 0.5, kUnknown);  // runs off the source grid
  EXPECT_TRUE(std::isnan(edge.valueAt(5.2, 5.2, 0.0f)));
  EXPECT_FLOAT_EQ(edge.valueAt(4.85, 4.85, kUnknown), 0.0f);
}

TEST(HeightMap, RecenterKeepsWorldCoordinates)
{
  HeightMap map(0.1, 100);
  map.integrate({{2.05f, 1.05f, 0.7f}});
  map.recenter(3.0, 0.0, 2.0);
  EXPECT_FLOAT_EQ(map.elevation().valueAt(2.05, 1.05, kUnknown), 0.7f);
  EXPECT_NEAR(map.elevation().origin_x, -2.0, 1e-9);
  map.recenter(3.5, 0.0, 2.0);  // within the margin: no move
  EXPECT_NEAR(map.elevation().origin_x, -2.0, 1e-9);
}

TEST(HeightMap, FootprintStampsOnlyUnknownCells)
{
  HeightMap map(0.1, 100);
  map.integrate({{0.25f, 0.05f, 0.9f}});
  Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
  base.translation() = Eigen::Vector3d(0.0, 0.0, 0.43);
  map.stampFootprint(base, 0.43, 0.8);
  EXPECT_NEAR(map.elevation().valueAt(-0.35, 0.05, kUnknown), 0.0, 1e-6);
  EXPECT_FLOAT_EQ(map.elevation().valueAt(0.25, 0.05, kUnknown), 0.9f);
  EXPECT_TRUE(std::isnan(map.elevation().valueAt(1.5, 0.0, 0.0f)));
}

TEST(HeightMap, MaxFilterIgnoresUnknown)
{
  Grid<float> g(1.0, 0.0, 0.0, 5, 5, kUnknown);
  g.at(2, 2) = 1.0f;
  g.at(4, 4) = 3.0f;
  const Grid<float> m = maxFilter(g, 1, false);
  EXPECT_FLOAT_EQ(m.at(1, 1), 1.0f);
  EXPECT_FLOAT_EQ(m.at(3, 3), 3.0f);
  EXPECT_TRUE(std::isnan(m.at(0, 4)));

  // Strict: any unknown cell (or the map edge) in the window makes the result unknown.
  Grid<float> full(1.0, 0.0, 0.0, 5, 5, 0.5f);
  full.at(3, 3) = kUnknown;
  const Grid<float> strict = maxFilter(full, 1, true);
  EXPECT_FLOAT_EQ(strict.at(1, 1), 0.5f);
  EXPECT_TRUE(std::isnan(strict.at(2, 2)));
  EXPECT_TRUE(std::isnan(strict.at(0, 2)));
}

TEST(HeightMap, DepthOfFlatGroundProjectsToTheGround)
{
  const RoverParams p = loadRoverParams(roverConfigPath());
  const int w = 64;
  const int h = 48;
  const CameraIntrinsics k{32.0, 32.0, 32.0, 24.0};
  Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
  base.translation().z() = p.pivotHeight();
  const Eigen::Isometry3d cam = base * p.navcamOptical();
  // Depth of the plane z = 0 along each pixel ray (optical z component of the hit point).
  std::vector<float> depth(w * h, std::numeric_limits<float>::infinity());
  for (int v = 0; v < h; ++v) {
    for (int u = 0; u < w; ++u) {
      const Eigen::Vector3d ray_opt((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0);
      const Eigen::Vector3d ray = cam.linear() * ray_opt;
      if (ray.z() >= -1e-6) continue;
      depth[v * w + u] = static_cast<float>(-cam.translation().z() / ray.z());
    }
  }
  const auto pts = depthToPoints(depth.data(), w, h, k, cam, 1, 0.1, 20.0);
  ASSERT_GT(pts.size(), 500u);
  for (const auto & pt : pts) {
    EXPECT_NEAR(pt.z(), 0.0, 1e-4);
    EXPECT_GT(pt.x(), p.navcam.x);  // the camera looks forward
  }
}

}  // namespace
}  // namespace rover_core
