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

#include <string>

#include <gtest/gtest.h>

#include "rover_core/fault_monitor.hpp"

namespace rover_core
{
namespace
{

const RoverParams::FaultLimits kLimits{0.44, 0.42, 0.57, 0.6, 0.2, 2.0};

FaultInputs nominal()
{
  return {0.0, 0.0, {0.0, 0.0}, {0.0, 0.0}, std::nullopt, {2, 2, 2, 2, 2, 2}, {2, 2, 2, 2, 2, 2}, 0.05};
}

TEST(FaultMonitor, NominalDrivingIsFine)
{
  FaultMonitor m(kLimits);
  std::string detail;
  EXPECT_EQ(m.update(nominal(), true, detail), Fault::kNone);
}

TEST(FaultMonitor, TiltAndSuspension)
{
  FaultMonitor m(kLimits);
  std::string detail;
  FaultInputs in = nominal();
  in.roll = 0.35;
  in.pitch = 0.3;  // combined tilt ~26 deg
  EXPECT_EQ(m.update(in, true, detail), Fault::kTilt);
  in = nominal();
  in.bogie[1] = -0.6;
  EXPECT_EQ(m.update(in, true, detail), Fault::kSuspension);
}

TEST(FaultMonitor, SlipOnlyWhenChecked)
{
  FaultMonitor m(kLimits);
  std::string detail;
  FaultInputs in = nominal();
  in.slip = 0.8;
  EXPECT_EQ(m.update(in, false, detail), Fault::kNone);
  EXPECT_EQ(m.update(in, true, detail), Fault::kSlip);
}

TEST(FaultMonitor, StallNeedsToPersist)
{
  FaultMonitor m(kLimits);
  std::string detail;
  FaultInputs in = nominal();
  in.wheel_measured[4] = 0.1;
  for (int k = 0; k < 39; ++k) EXPECT_EQ(m.update(in, true, detail), Fault::kNone);  // 1.95 s
  m.update(in, true, detail);  // exactly at the limit
  EXPECT_EQ(m.update(in, true, detail), Fault::kStall);
  m.reset();
  EXPECT_EQ(m.update(in, true, detail), Fault::kNone);
}

}  // namespace
}  // namespace rover_core
