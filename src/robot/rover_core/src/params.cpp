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

#include "rover_core/params.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order): clang-format groups it after other headers

namespace rover_core
{

Eigen::Isometry3d RoverParams::navcamOptical() const
{
  // Camera link: x forward, y left, z up, pitched down by `pitch` about y.
  // Optical frame axes expressed in the camera link: x = -y_link, y = -z_link, z = x_link.
  Eigen::Matrix3d link_from_optical;
  link_from_optical << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(navcam.x, 0.0, navcam.z);
  pose.linear() = Eigen::AngleAxisd(navcam.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() * link_from_optical;
  return pose;
}

Eigen::Vector3d WorldParams::sunDirection() const
{
  return {
    std::cos(sun_elevation) * std::cos(sun_azimuth),
    std::cos(sun_elevation) * std::sin(sun_azimuth),
    std::sin(sun_elevation)};
}

std::string roverConfigPath()
{
  return ament_index_cpp::get_package_share_directory("rover_description") + "/config/rover.yaml";
}

std::string worldConfigPath()
{
  return ament_index_cpp::get_package_share_directory("rover_description") + "/config/world.yaml";
}

std::string orbitalMapPath()
{
  return ament_index_cpp::get_package_share_directory("rover_description") + "/orbital/orbital_map.yaml";
}

RoverParams loadRoverParams(const std::string & path)
{
  const YAML::Node y = YAML::LoadFile(path);
  RoverParams p;
  p.wheel_radius = y["wheel"]["radius"].as<double>();
  p.wheel_width = y["wheel"]["width"].as<double>();
  p.track_width = y["track_width"].as<double>();
  p.body_length = y["body"]["length"].as<double>();
  p.body_width = y["body"]["width"].as<double>();
  p.body_height = y["body"]["height"].as<double>();
  p.body_center_z = y["body"]["center_z"].as<double>();
  p.rocker_pivot_x = y["rocker"]["pivot_x"].as<double>();
  p.front_wheel_x = y["rocker"]["front_wheel"][0].as<double>();
  p.front_wheel_z = y["rocker"]["front_wheel"][1].as<double>();
  p.bogie_pivot_x = y["rocker"]["bogie_pivot"][0].as<double>();
  p.bogie_pivot_z = y["rocker"]["bogie_pivot"][1].as<double>();
  p.bogie_half_length = y["bogie"]["half_length"].as<double>();
  p.bogie_drop = y["bogie"]["drop"].as<double>();
  if (std::abs(p.bogie_pivot_z - p.bogie_drop - p.front_wheel_z) > 1e-9) {
    throw std::runtime_error("rover.yaml: front wheel and bogie wheels must sit at the same height");
  }

  const YAML::Node cam = y["navcam"];
  p.navcam = {
    cam["x"].as<double>(),
    cam["z"].as<double>(),
    cam["pitch"].as<double>(),
    cam["near"].as<double>(),
    cam["far"].as<double>()};
  p.drive = {y["drive"]["speed"].as<double>(), y["drive"]["turn_rate"].as<double>()};

  const YAML::Node ace = y["ace_limits"];
  p.ace = {
    ace["tilt"].as<double>(),
    ace["rocker"].as<double>(),
    ace["bogie"].as<double>(),
    ace["belly_clearance"].as<double>(),
    ace["wheel_margin"].as<double>()};
  const YAML::Node f = y["fault_limits"];
  p.faults = {
    f["tilt"].as<double>(),
    f["rocker"].as<double>(),
    f["bogie"].as<double>(),
    f["slip"].as<double>(),
    f["stall_ratio"].as<double>(),
    f["stall_time"].as<double>(),
    f["persist_time"].as<double>()};
  return p;
}

WorldParams loadWorldParams(const std::string & path)
{
  const YAML::Node y = YAML::LoadFile(path);
  constexpr double kDeg = M_PI / 180.0;
  WorldParams w;
  w.sun_azimuth = y["sun"]["azimuth_deg"].as<double>() * kDeg;
  w.sun_elevation = y["sun"]["elevation_deg"].as<double>() * kDeg;
  w.spawn = {y["spawn"]["x"].as<double>(), y["spawn"]["y"].as<double>(), y["spawn"]["yaw_deg"].as<double>() * kDeg};
  return w;
}

Grid<float> loadOrbitalMap(const std::string & yaml_path)
{
  const YAML::Node y = YAML::LoadFile(yaml_path);
  Grid<float> dem(
    y["resolution"].as<double>(),
    y["origin_x"].as<double>(),
    y["origin_y"].as<double>(),
    y["width"].as<int>(),
    y["height"].as<int>(),
    0.0f);
  const std::string dir = yaml_path.substr(0, yaml_path.find_last_of('/') + 1);
  std::ifstream file(dir + y["file"].as<std::string>(), std::ios::binary);
  if (!file) throw std::runtime_error("cannot open orbital DEM next to " + yaml_path);
  file.read(reinterpret_cast<char *>(dem.data.data()), static_cast<std::streamsize>(dem.data.size() * sizeof(float)));
  if (file.gcount() != static_cast<std::streamsize>(dem.data.size() * sizeof(float))) {
    throw std::runtime_error("orbital DEM is smaller than its yaml header says");
  }
  return dem;
}

}  // namespace rover_core
