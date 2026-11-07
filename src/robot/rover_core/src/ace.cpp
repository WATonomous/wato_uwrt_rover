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

#include "rover_core/ace.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rover_core
{

namespace
{

double maxAbs(const Interval & v)
{
  return std::max(std::abs(v.lo), std::abs(v.hi));
}

double safeAsin(double v)
{
  return std::asin(std::clamp(v, -1.0, 1.0));
}

// Bounds for one rocker-bogie side, in the side's vertical (x, z) plane.
// Angles are nose-up positive. All heights are wheel-center heights.
struct SideBounds
{
  Interval rocker;  // rocker angle relative to the horizontal
  Interval pivot_z;  // rocker pivot (= base_link) height
  Interval bogie;  // bogie angle relative to the rocker
};

SideBounds sideBounds(const RoverParams & p, const Interval & front, const Interval & middle, const Interval & rear)
{
  // Bogie: middle and rear wheels sit 2c apart; the pivot is `drop` above their midpoint,
  // measured perpendicular to the bogie: z_b = (z_m + z_r) / 2 + drop * cos(theta_b).
  const double c = p.bogie_half_length;
  const Interval theta_b{std::atan2(middle.lo - rear.hi, 2.0 * c), std::atan2(middle.hi - rear.lo, 2.0 * c)};
  const double min_cos_b = std::cos(maxAbs(theta_b));
  const Interval z_b{(middle.lo + rear.lo) / 2.0 + p.bogie_drop * min_cos_b, (middle.hi + rear.hi) / 2.0 + p.bogie_drop};

  // Rocker: rigid link from the bogie pivot B to the front wheel F. In the rocker frame the
  // vector B->F is (dx, dz) at angle phi0; in the world it is at phi = theta_r + phi0 with
  // z_f - z_b = |BF| sin(phi), so theta_r follows from the height difference alone.
  const double dx = p.front_wheel_x - p.bogie_pivot_x;
  const double dz = p.front_wheel_z - p.bogie_pivot_z;
  const double length = std::hypot(dx, dz);
  const double phi0 = std::atan2(dz, dx);
  const Interval theta_r{
    safeAsin((front.lo - z_b.hi) / length) - phi0, safeAsin((front.hi - z_b.lo) / length) - phi0};

  // Pivot height: P = B - R(theta_r) * b with b the pivot->bogie-pivot vector, so
  // z_p = z_b + g(theta_r), g(t) = -(b_x sin t + b_z cos t). Bound g over the interval.
  const double bx = p.bogie_pivot_x;
  const double bz = p.bogie_pivot_z;
  auto g = [&](double t) { return -(bx * std::sin(t) + bz * std::cos(t)); };
  double g_lo = std::min(g(theta_r.lo), g(theta_r.hi));
  double g_hi = std::max(g(theta_r.lo), g(theta_r.hi));
  const double critical = std::atan2(bx, bz);  // g'(t) = 0
  for (double t : {critical - M_PI, critical, critical + M_PI}) {
    if (t > theta_r.lo && t < theta_r.hi) {
      g_lo = std::min(g_lo, g(t));
      g_hi = std::max(g_hi, g(t));
    }
  }
  return {theta_r, {z_b.lo + g_lo, z_b.hi + g_hi}, {theta_b.lo - theta_r.hi, theta_b.hi - theta_r.lo}};
}

}  // namespace

const char * toString(AceResult r)
{
  switch (r) {
    case AceResult::kSafe:
      return "safe";
    case AceResult::kUnknown:
      return "unknown";
    case AceResult::kTilt:
      return "tilt";
    case AceResult::kRocker:
      return "rocker";
    case AceResult::kBogie:
      return "bogie";
    case AceResult::kClearance:
      return "clearance";
  }
  throw std::invalid_argument("unknown AceResult");
}

AceBounds aceBounds(const RoverParams & p, const std::array<Interval, 6> & ground, double belly_terrain)
{
  std::array<Interval, 6> wheel;
  for (size_t k = 0; k < 6; ++k) wheel[k] = {ground[k].lo + p.wheel_radius, ground[k].hi + p.wheel_radius};
  const SideBounds left = sideBounds(p, wheel[0], wheel[1], wheel[2]);
  const SideBounds right = sideBounds(p, wheel[3], wheel[4], wheel[5]);

  // The differential holds the rockers at equal and opposite angles to the body, so the body
  // pitches to the average of the two rocker angles.
  AceBounds b;
  b.pitch = {(left.rocker.lo + right.rocker.lo) / 2.0, (left.rocker.hi + right.rocker.hi) / 2.0};
  b.rocker = {(left.rocker.lo - right.rocker.hi) / 2.0, (left.rocker.hi - right.rocker.lo) / 2.0};
  b.roll = {
    safeAsin((left.pivot_z.lo - right.pivot_z.hi) / p.track_width),
    safeAsin((left.pivot_z.hi - right.pivot_z.lo) / p.track_width)};
  b.bogie_left = left.bogie;
  b.bogie_right = right.bogie;

  // Lowest belly point: the belly plane sits (center_z - height / 2) below base_link and its
  // corners drop further when the body tilts.
  const double base_z_lo = (left.pivot_z.lo + right.pivot_z.lo) / 2.0;
  const double belly_lo = base_z_lo + (p.body_center_z - p.body_height / 2.0) -
                          p.body_length / 2.0 * std::sin(maxAbs(b.pitch)) -
                          p.body_width / 2.0 * std::sin(maxAbs(b.roll));
  b.clearance = belly_lo - belly_terrain;
  return b;
}

AceResult classify(const RoverParams & p, const AceBounds & b)
{
  const double tilt = std::acos(std::cos(maxAbs(b.roll)) * std::cos(maxAbs(b.pitch)));
  if (tilt > p.ace.tilt) return AceResult::kTilt;
  if (maxAbs(b.rocker) > p.ace.rocker) return AceResult::kRocker;
  if (std::max(maxAbs(b.bogie_left), maxAbs(b.bogie_right)) > p.ace.bogie) return AceResult::kBogie;
  if (b.clearance < p.ace.belly_clearance) return AceResult::kClearance;
  return AceResult::kSafe;
}

AceMaps buildAceMaps(const RoverParams & p, const Grid<float> & elevation)
{
  const double res = elevation.resolution;
  const int inner = std::max(1, static_cast<int>(std::lround(p.wheel_width / 2.0 / res)));
  const int outer = static_cast<int>(std::ceil((p.wheel_width / 2.0 + p.ace.wheel_margin) / res));
  const int belly = static_cast<int>(std::ceil(p.body_width / 4.0 / res));
  return {maxFilter(elevation, inner), maxFilter(elevation, outer), maxFilter(elevation, belly)};
}

AceResult evaluatePose(const RoverParams & p, const AceMaps & maps, const Pose2 & pose)
{
  const auto xs = p.wheelX();
  std::array<Interval, 6> ground;
  for (int side = 0; side < 2; ++side) {
    const double y = (side == 0 ? 1.0 : -1.0) * p.track_width / 2.0;
    for (int k = 0; k < 3; ++k) {
      double wx;
      double wy;
      transformPoint(pose, xs[k], y, wx, wy);
      const float lo = maps.wheel_inner.valueAt(wx, wy, kUnknown);
      const float hi = maps.wheel_outer.valueAt(wx, wy, kUnknown);
      if (std::isnan(lo) || std::isnan(hi)) return AceResult::kUnknown;
      ground[side * 3 + k] = {lo, hi};
    }
  }

  // A 4 x 2 grid of small windows covers the belly footprint. Small windows matter: at any
  // heading their corners stay inside the body, so rocks under the wheels never count as
  // "under the belly".
  const double dx = p.body_length / 8.0;
  const double dy = p.body_width / 4.0;
  float belly = -kLethal;
  for (double bx : {-3.0 * dx, -dx, dx, 3.0 * dx}) {
    for (double by : {-dy, dy}) {
      double wx;
      double wy;
      transformPoint(pose, bx, by, wx, wy);
      const float v = maps.belly.valueAt(wx, wy, kUnknown);
      if (std::isnan(v)) return AceResult::kUnknown;
      belly = std::max(belly, v);
    }
  }
  return classify(p, aceBounds(p, ground, belly));
}

}  // namespace rover_core
