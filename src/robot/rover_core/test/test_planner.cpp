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
#include <vector>

#include <gtest/gtest.h>

#include "rover_core/planner.hpp"

namespace rover_core
{
namespace
{

PathTreeConfig treeConfig()
{
  return {
    {0.0, 0.5, -0.5, 1.0, -1.0, 1.6, -1.6, 2.4, -2.4, M_PI}, {0.0, 0.3, -0.3, 0.8, -0.8}, 1.2, 0.1, 15.0 * M_PI / 180.0};
}

CostConfig costConfig()
{
  return {8.0 * M_PI / 180.0, 22.0 * M_PI / 180.0, 0.12, 0.30, 4.0};
}

TEST(PathTree, GeometryAndCommitSegment)
{
  const auto tree = buildPathTree(treeConfig());
  ASSERT_EQ(tree.size(), 10u * 5u * 5u);
  const CandidatePath & straight = tree[0];
  EXPECT_EQ(straight.turn_poses, 0);
  EXPECT_NEAR(straight.poses.back().x, 2.4, 1e-9);
  EXPECT_NEAR(straight.poses.back().y, 0.0, 1e-9);
  EXPECT_NEAR(straight.poses[straight.commit_end - 1].s, 1.2, 1e-9);

  // A left arc of curvature k over length L ends at heading k * L, on the left.
  const CandidatePath & left = tree[1 * 5 + 1];  // turn 0, k1 = 0.3, k2 = 0.3
  EXPECT_NEAR(left.poses.back().yaw, 0.3 * 2.4, 1e-9);
  EXPECT_GT(left.poses.back().y, 0.0);

  const CandidatePath & turned = tree[1 * 25];  // turn 0.5, straight arcs
  EXPECT_GT(turned.turn_poses, 0);
  EXPECT_NEAR(turned.poses[turned.turn_poses].yaw, 0.5, 1e-9);
  EXPECT_NEAR(turned.poses.back().x, 2.4 * std::cos(0.5), 1e-9);
}

TEST(CostToGo, RoutesAroundWalls)
{
  Grid<float> cost(0.25, 0.0, 0.0, 40, 40, 0.0f);
  for (int j = 0; j < 35; ++j) cost.at(20, j) = kLethal;  // wall with a gap at the top
  const Grid<float> ctg = costToGo(cost, 9.0, 1.0, 0.0, 4.0);  // goal right of the wall
  const float from_left = ctg.valueAt(1.0, 1.0, kLethal);
  EXPECT_TRUE(std::isfinite(from_left));
  EXPECT_GT(from_left, 14.0);  // straight line is 8 m; the route detours around the wall's end
  EXPECT_LT(ctg.valueAt(9.0, 1.0, kLethal), 0.2f);  // seeded with the distance to the goal

  // Fully enclosed goal: everything outside is unreachable.
  Grid<float> boxed(0.25, 0.0, 0.0, 40, 40, 0.0f);
  for (int k = 10; k <= 20; ++k) {
    boxed.at(k, 10) = boxed.at(k, 20) = boxed.at(10, k) = boxed.at(20, k) = kLethal;
  }
  const Grid<float> unreachable = costToGo(boxed, 3.75, 3.75, 0.0, 4.0);
  EXPECT_TRUE(std::isinf(unreachable.valueAt(1.0, 1.0, 0.0f)));

  // A lethal cell exactly on the goal: the cells around it within the radius are sources.
  Grid<float> rock_on_goal(0.25, 0.0, 0.0, 40, 40, 0.0f);
  int i;
  int j;
  rock_on_goal.cellOf(5.0, 5.0, i, j);
  rock_on_goal.at(i, j) = kLethal;
  EXPECT_TRUE(std::isinf(costToGo(rock_on_goal, 5.0, 5.0, 0.0, 4.0).valueAt(1.0, 1.0, 0.0f)));
  EXPECT_TRUE(std::isfinite(costToGo(rock_on_goal, 5.0, 5.0, 1.0, 4.0).valueAt(1.0, 1.0, kLethal)));
}

TEST(OrbitalCost, SlopeThresholds)
{
  Grid<float> dem(0.25, 0.0, 0.0, 20, 20, 0.0f);
  for (int j = 0; j < 20; ++j) {
    for (int i = 0; i < 20; ++i) dem.at(i, j) = static_cast<float>(i * 0.25 * std::tan(30.0 * M_PI / 180.0));
  }
  const Grid<float> steep = orbitalCost(dem, costConfig());
  EXPECT_TRUE(std::isinf(steep.at(5, 5)));
  EXPECT_TRUE(std::isinf(steep.at(0, 5)));  // edges always lethal

  const Grid<float> flat = orbitalCost(Grid<float>(0.25, 0.0, 0.0, 20, 20, 1.0f), costConfig());
  EXPECT_FLOAT_EQ(flat.at(5, 5), 0.0f);
}

struct World
{
  Grid<float> elevation{0.05, -8.0, -8.0, 320, 320, 0.0f};
  Grid<float> cost{0.25, -20.0, -20.0, 160, 160, 0.0f};
  Grid<float> ctg;
};

PlanResult planTo(const World & w, double gx, double gy, const std::vector<CandidatePath> & tree)
{
  World copy = w;
  copy.ctg = costToGo(copy.cost, gx, gy, 0.5, 4.0);
  const PlanInputs in{{0, 0, 0}, {0, 0, 0}, &copy.elevation, &copy.cost, &copy.ctg, false, 0.0};
  return plan(loadRoverParams(roverConfigPath()), tree, in, {0.3, 0.2, 0.3}, 4.0, 200);
}

TEST(Planner, DrivesStraightAtAGoalAhead)
{
  const auto tree = buildPathTree(treeConfig());
  const PlanResult r = planTo(World{}, 10.0, 0.0, tree);
  ASSERT_TRUE(r.found);
  EXPECT_EQ(tree[r.index].turn, 0.0);
  EXPECT_NEAR(r.poses.back().y, 0.0, 0.3);
  EXPECT_NEAR(r.commit_length, 1.2, 1e-9);
  EXPECT_EQ(r.ace_evaluations, 1);
}

TEST(Planner, TurnsInPlaceTowardsAGoalBehind)
{
  const auto tree = buildPathTree(treeConfig());
  const PlanResult r = planTo(World{}, -10.0, 0.5, tree);
  ASSERT_TRUE(r.found);
  EXPECT_GT(std::abs(tree[r.index].turn), 2.0);
}

TEST(Planner, AceRejectsARockAheadAndPicksAnotherPath)
{
  World w;
  // 0.5 m boulder 1.5 m ahead: too tall to straddle, too tall to climb.
  int ci;
  int cj;
  w.elevation.cellOf(1.5, 0.0, ci, cj);
  for (int dj = -5; dj <= 5; ++dj) {
    for (int di = -5; di <= 5; ++di) w.elevation.at(ci + di, cj + dj) = 0.5f;
  }
  const auto tree = buildPathTree(treeConfig());
  const PlanResult r = planTo(w, 10.0, 0.0, tree);
  ASSERT_TRUE(r.found);
  EXPECT_GT(r.ace_evaluations, 1);
  EXPECT_FALSE(r.ace_rejected.empty());
  const AceMaps maps = buildAceMaps(loadRoverParams(roverConfigPath()), w.elevation);
  for (const auto & pose : r.poses) {
    if (pose.s > r.commit_length) break;
    EXPECT_EQ(evaluatePose(loadRoverParams(roverConfigPath()), maps, {pose.x, pose.y, pose.yaw}), AceResult::kSafe);
  }
}

TEST(Planner, NoPathWhenTheCommittedTerrainIsUnseen)
{
  World w;
  w.elevation = Grid<float>(0.05, -8.0, -8.0, 320, 320, kUnknown);
  const PlanResult r = planTo(w, 10.0, 0.0, buildPathTree(treeConfig()));
  EXPECT_FALSE(r.found);
}

}  // namespace
}  // namespace rover_core
