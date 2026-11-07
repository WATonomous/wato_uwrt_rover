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

#include <array>
#include <string>

#include "rover_core/grid.hpp"
#include "rover_core/height_map.hpp"
#include "rover_core/params.hpp"

namespace rover_core
{

// Approximate Clearance Evaluation (Otsu et al., JFR 2020).
//
// Instead of solving the rocker-bogie kinematics for an exact pose, ACE takes an interval
// [lo, hi] for the ground height under each wheel and pushes the intervals through the
// suspension linkage in closed form. The result bounds body tilt, suspension angles and
// belly clearance; a pose is safe only if the worst case of every bound is within limits.
// It is conservative (it may reject a safe pose, never accept an unsafe one, as far as the
// height map is right) and costs a fixed handful of arithmetic per pose.

struct Interval
{
  double lo;
  double hi;
};

struct AceBounds
{
  Interval pitch;  // body pitch, nose up positive
  Interval roll;  // body roll, left side up positive
  Interval rocker;  // rocker angle relative to the body (left side; right is the negative)
  Interval bogie_left;  // bogie angle relative to its rocker
  Interval bogie_right;
  double clearance;  // worst-case belly clearance
};

enum class AceResult
{
  kSafe,
  kUnknown,  // a wheel patch has never been observed
  kTilt,
  kRocker,
  kBogie,
  kClearance,
};

const char * toString(AceResult r);

// Wheel order: left front, left middle, left rear, right front, right middle, right rear.
// ground: ground height intervals under each wheel; belly_terrain: highest terrain under the belly.
AceBounds aceBounds(const RoverParams & p, const std::array<Interval, 6> & ground, double belly_terrain);

// Worst-case check of the bounds against the ACE limits.
AceResult classify(const RoverParams & p, const AceBounds & b);

// Max-filtered height maps ACE samples per pose. Build once per planning cycle.
struct AceMaps
{
  Grid<float> wheel_inner;  // window covering the wheel contact patch
  Grid<float> wheel_outer;  // contact patch plus pose uncertainty
  Grid<float> belly;  // small window, sampled on a 4 x 2 grid under the body
};

AceMaps buildAceMaps(const RoverParams & p, const Grid<float> & elevation);

// Evaluate a rover pose (odom frame) against the height map.
AceResult evaluatePose(const RoverParams & p, const AceMaps & maps, const Pose2 & pose);

}  // namespace rover_core
