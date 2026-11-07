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
#include <cstdint>
#include <optional>
#include <string>

#include "rover_core/params.hpp"

namespace rover_core
{

// Values match rover_msgs/MobilityStatus FAULT_*.
enum class Fault : uint8_t
{
  kNone = 0,
  kTilt = 1,
  kSuspension = 2,
  kSlip = 3,
  kStall = 4,
  kNoSafePath = 5,
};

const char * toString(Fault f);

struct FaultInputs
{
  double roll;
  double pitch;
  std::array<double, 2> rocker;  // joint angles, left / right
  std::array<double, 2> bogie;
  std::optional<double> slip;  // nullopt when there is no valid estimate
  std::array<double, 6> wheel_command;  // rad/s
  std::array<double, 6> wheel_measured;  // rad/s
  double dt;
};

// Mobility fault protection: stop the rover when the vehicle state leaves its safe envelope.
class FaultMonitor
{
public:
  explicit FaultMonitor(const RoverParams::FaultLimits & limits) : limits_(limits) {}

  // Returns the first fault found (and why), or kNone. Slip is skipped when check_slip is
  // false, e.g. while backing out of the sand that caused the slip fault.
  Fault update(const FaultInputs & in, bool check_slip, std::string & detail);

  void reset() { stall_time_.fill(0.0); }

private:
  RoverParams::FaultLimits limits_;
  std::array<double, 6> stall_time_{};
};

}  // namespace rover_core
