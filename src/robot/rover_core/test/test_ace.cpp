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

#include <array>
#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "rover_core/ace.hpp"

namespace rover_core
{
namespace
{

RoverParams params()
{
  return loadRoverParams(roverConfigPath());
}

std::array<Interval, 6> exact(const std::array<double, 6> & z)
{
  std::array<Interval, 6> out;
  for (size_t k = 0; k < 6; ++k) out[k] = {z[k], z[k]};
  return out;
}

// Ground heights under the wheels for a plane z = x * tan(pitch) + y * tan(roll).
std::array<double, 6> plane(const RoverParams & p, double pitch, double roll)
{
  std::array<double, 6> z;
  const auto xs = p.wheelX();
  for (int side = 0; side < 2; ++side) {
    const double y = (side == 0 ? 1.0 : -1.0) * p.track_width / 2.0;
    for (int k = 0; k < 3; ++k) z[side * 3 + k] = xs[k] * std::tan(pitch) + y * std::tan(roll);
  }
  return z;
}

TEST(Ace, FlatGroundIsLevelWithNominalClearance)
{
  const RoverParams p = params();
  const AceBounds b = aceBounds(p, exact({0, 0, 0, 0, 0, 0}), 0.0);
  EXPECT_NEAR(b.pitch.lo, 0.0, 1e-9);
  EXPECT_NEAR(b.pitch.hi, 0.0, 1e-9);
  EXPECT_NEAR(b.roll.hi, 0.0, 1e-9);
  EXPECT_NEAR(b.rocker.hi, 0.0, 1e-9);
  EXPECT_NEAR(b.bogie_left.hi, 0.0, 1e-9);
  EXPECT_NEAR(b.clearance, p.pivotHeight() + p.body_center_z - p.body_height / 2.0, 1e-9);
  EXPECT_EQ(classify(p, b), AceResult::kSafe);
}

TEST(Ace, SlopeShowsUpAsPitchAndRoll)
{
  const RoverParams p = params();
  const double slope = 10.0 * M_PI / 180.0;
  const AceBounds pitched = aceBounds(p, exact(plane(p, slope, 0.0)), -1.0);
  EXPECT_NEAR(pitched.pitch.lo, slope, 0.01);
  // Interval bounds are slightly conservative even for exact heights (bogie cos term).
  EXPECT_NEAR(pitched.roll.hi, 0.0, 0.01);
  EXPECT_NEAR(pitched.bogie_left.hi, 0.0, 0.01);
  const AceBounds rolled = aceBounds(p, exact(plane(p, 0.0, slope)), -1.0);
  EXPECT_NEAR(rolled.roll.lo, slope, 0.01);
  EXPECT_NEAR(rolled.pitch.hi, 0.0, 0.01);
  EXPECT_EQ(classify(p, rolled), AceResult::kSafe);

  const double steep = 25.0 * M_PI / 180.0;
  EXPECT_EQ(classify(p, aceBounds(p, exact(plane(p, steep, 0.0)), -1.0)), AceResult::kTilt);
}

TEST(Ace, StepUnderOneWheelArticulatesTheBogie)
{
  const RoverParams p = params();
  const AceBounds b = aceBounds(p, exact({0, 0.15, 0, 0, 0, 0}), 0.0);
  EXPECT_GT(b.bogie_left.lo, 0.1);
  EXPECT_LT(std::abs(b.bogie_right.hi), std::abs(b.bogie_left.lo));
  EXPECT_GT(b.roll.lo, 0.0);  // left side is higher
  EXPECT_EQ(classify(p, b), AceResult::kSafe);

  // A wheel-height step under one middle wheel is more than the bogie can absorb.
  EXPECT_EQ(classify(p, aceBounds(p, exact({0, 0.45, 0, 0, 0, 0}), 0.0)), AceResult::kBogie);
}

TEST(Ace, RockUnderTheBellyFailsClearance)
{
  const RoverParams p = params();
  EXPECT_EQ(classify(p, aceBounds(p, exact({0, 0, 0, 0, 0, 0}), 0.15)), AceResult::kSafe);
  EXPECT_EQ(classify(p, aceBounds(p, exact({0, 0, 0, 0, 0, 0}), 0.28)), AceResult::kClearance);
}

// The interval bounds must contain the exact answer for every height inside the intervals.
TEST(Ace, BoundsContainEveryExactConfiguration)
{
  const RoverParams p = params();
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> center(-0.15, 0.15);
  std::uniform_real_distribution<double> width(0.0, 0.08);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  for (int trial = 0; trial < 200; ++trial) {
    std::array<Interval, 6> ground;
    for (auto & g : ground) {
      g.lo = center(rng);
      g.hi = g.lo + width(rng);
    }
    const AceBounds b = aceBounds(p, ground, 0.0);
    for (int sample = 0; sample < 20; ++sample) {
      std::array<double, 6> z;
      for (size_t k = 0; k < 6; ++k) z[k] = ground[k].lo + unit(rng) * (ground[k].hi - ground[k].lo);
      const AceBounds e = aceBounds(p, exact(z), 0.0);
      const double eps = 1e-9;
      EXPECT_GE(e.pitch.lo, b.pitch.lo - eps);
      EXPECT_LE(e.pitch.hi, b.pitch.hi + eps);
      EXPECT_GE(e.roll.lo, b.roll.lo - eps);
      EXPECT_LE(e.roll.hi, b.roll.hi + eps);
      EXPECT_GE(e.rocker.lo, b.rocker.lo - eps);
      EXPECT_LE(e.rocker.hi, b.rocker.hi + eps);
      EXPECT_GE(e.bogie_left.lo, b.bogie_left.lo - eps);
      EXPECT_LE(e.bogie_left.hi, b.bogie_left.hi + eps);
      EXPECT_GE(e.clearance, b.clearance - eps);
    }
  }
}

TEST(Ace, EvaluatePoseUsesTheHeightMap)
{
  const RoverParams p = params();
  Grid<float> flat(0.05, -5.0, -5.0, 200, 200, 0.0f);
  EXPECT_EQ(evaluatePose(p, buildAceMaps(p, flat), {0.0, 0.0, 0.7}), AceResult::kSafe);

  Grid<float> unknown(0.05, -5.0, -5.0, 200, 200, kUnknown);
  EXPECT_EQ(evaluatePose(p, buildAceMaps(p, unknown), {0.0, 0.0, 0.0}), AceResult::kUnknown);

  // A 0.3 m rock under the body center.
  Grid<float> rock = flat;
  int i;
  int j;
  rock.cellOf(0.0, 0.0, i, j);
  for (int dj = -2; dj <= 2; ++dj) {
    for (int di = -2; di <= 2; ++di) rock.at(i + di, j + dj) = 0.3f;
  }
  EXPECT_EQ(evaluatePose(p, buildAceMaps(p, rock), {0.0, 0.0, 0.0}), AceResult::kClearance);
  // Drive past it: the rock is now beside the rover, between nothing.
  EXPECT_EQ(evaluatePose(p, buildAceMaps(p, rock), {0.0, 1.5, 0.0}), AceResult::kSafe);
}

}  // namespace
}  // namespace rover_core
