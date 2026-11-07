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

#include "rover_core/fault_monitor.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace rover_core
{

namespace
{

constexpr double kMinStallCommand = 0.3;  // rad/s, below this a wheel is not really commanded

std::string format(const char * fmt, double a, double b)
{
  char buf[128];
  std::snprintf(buf, sizeof(buf), fmt, a, b);
  return buf;
}

}  // namespace

const char * toString(Fault f)
{
  switch (f) {
    case Fault::kNone:
      return "none";
    case Fault::kTilt:
      return "tilt";
    case Fault::kSuspension:
      return "suspension";
    case Fault::kSlip:
      return "slip";
    case Fault::kStall:
      return "stall";
    case Fault::kNoSafePath:
      return "no_safe_path";
  }
  throw std::invalid_argument("unknown Fault");
}

Fault FaultMonitor::update(const FaultInputs & in, bool check_slip, std::string & detail)
{
  const double tilt = std::acos(std::cos(in.roll) * std::cos(in.pitch));
  if (tilt > limits_.tilt) {
    detail = format("tilt %.1f deg > %.1f deg", tilt * 180.0 / M_PI, limits_.tilt * 180.0 / M_PI);
    return Fault::kTilt;
  }
  for (int side = 0; side < 2; ++side) {
    if (std::abs(in.rocker[side]) > limits_.rocker) {
      detail = format("rocker %.2f rad > %.2f rad", std::abs(in.rocker[side]), limits_.rocker);
      return Fault::kSuspension;
    }
    if (std::abs(in.bogie[side]) > limits_.bogie) {
      detail = format("bogie %.2f rad > %.2f rad", std::abs(in.bogie[side]), limits_.bogie);
      return Fault::kSuspension;
    }
  }
  if (check_slip && in.slip && *in.slip > limits_.slip) {
    detail = format("slip %.0f%% > %.0f%%", *in.slip * 100.0, limits_.slip * 100.0);
    return Fault::kSlip;
  }
  for (size_t k = 0; k < 6; ++k) {
    const double cmd = std::abs(in.wheel_command[k]);
    const bool stalled = cmd > kMinStallCommand && std::abs(in.wheel_measured[k]) < limits_.stall_ratio * cmd;
    stall_time_[k] = stalled ? stall_time_[k] + in.dt : 0.0;
    if (stall_time_[k] > limits_.stall_time) {
      detail = format("wheel %.0f stalled for %.1f s", static_cast<double>(k), stall_time_[k]);
      return Fault::kStall;
    }
  }
  return Fault::kNone;
}

}  // namespace rover_core
