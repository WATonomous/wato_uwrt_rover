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

#include <Eigen/Geometry>

#include "rover_core/grid.hpp"

namespace rover_core
{

// Rover geometry and limits, loaded from rover_description/config/rover.yaml.
struct RoverParams
{
  double wheel_radius;
  double wheel_width;
  double track_width;

  double body_length;
  double body_width;
  double body_height;
  double body_center_z;

  double rocker_pivot_x;
  double front_wheel_x;  // relative to the rocker pivot
  double front_wheel_z;
  double bogie_pivot_x;  // relative to the rocker pivot
  double bogie_pivot_z;
  double bogie_half_length;
  double bogie_drop;

  struct Navcam
  {
    double x;
    double z;
    double pitch;
    double near;
    double far;
  } navcam;

  struct Drive
  {
    double speed;
    double turn_rate;
  } drive;

  struct AceLimits
  {
    double tilt;
    double rocker;
    double bogie;
    double belly_clearance;
    double wheel_margin;
  } ace;

  struct FaultLimits
  {
    double tilt;
    double rocker;
    double bogie;
    double slip;
    double stall_ratio;
    double stall_time;
  } faults;

  // Height of base_link (the rocker pivots) above flat ground.
  double pivotHeight() const { return wheel_radius - front_wheel_z; }

  // Wheel x positions in base_link: front, middle, rear.
  std::array<double, 3> wheelX() const
  {
    const double bogie_x = rocker_pivot_x + bogie_pivot_x;
    return {rocker_pivot_x + front_wheel_x, bogie_x + bogie_half_length, bogie_x - bogie_half_length};
  }

  // Pose of the navcam optical frame (z forward, x right, y down) in base_link.
  Eigen::Isometry3d navcamOptical() const;
};

struct WorldParams
{
  double sun_azimuth;  // rad, counter-clockwise from +x
  double sun_elevation;  // rad
  Pose2 spawn;

  // Unit vector pointing from the ground towards the sun, in the map frame.
  Eigen::Vector3d sunDirection() const;
};

std::string roverConfigPath();
std::string worldConfigPath();
std::string orbitalMapPath();

RoverParams loadRoverParams(const std::string & path);
WorldParams loadWorldParams(const std::string & path);

// Orbital DEM (map frame), see rover_description/tools/gen_world.py.
Grid<float> loadOrbitalMap(const std::string & yaml_path);

}  // namespace rover_core
