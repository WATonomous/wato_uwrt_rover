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

// Rocker-bogie differential. A real rover links its two rockers through a differential
// bar so that theta_left = -theta_right and the body pitches to the average of the two.
// Fortress has no mimic joints, so this system enforces the constraint with a stiff
// spring-damper on (theta_left + theta_right), applying the same torque to both rockers.

#include <memory>
#include <stdexcept>
#include <string>

#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>

namespace rover_sim
{

namespace gz = ignition::gazebo;

class RockerDifferential : public gz::System, public gz::ISystemConfigure, public gz::ISystemPreUpdate
{
public:
  void Configure(
    const gz::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::EntityComponentManager & ecm,
    gz::EventManager &) override
  {
    gz::Model model(entity);
    left_ = model.JointByName(ecm, sdf->Get<std::string>("left_joint"));
    right_ = model.JointByName(ecm, sdf->Get<std::string>("right_joint"));
    stiffness_ = sdf->Get<double>("stiffness");
    damping_ = sdf->Get<double>("damping");
    if (left_ == gz::kNullEntity || right_ == gz::kNullEntity) {
      throw std::runtime_error("RockerDifferential: rocker joints not found");
    }
    for (auto joint : {left_, right_}) {
      ecm.CreateComponent(joint, gz::components::JointPosition());
      ecm.CreateComponent(joint, gz::components::JointVelocity());
      ecm.CreateComponent(joint, gz::components::JointForceCmd({0.0}));
    }
  }

  void PreUpdate(const gz::UpdateInfo & info, gz::EntityComponentManager & ecm) override
  {
    if (info.paused) return;
    const auto & pos_l = ecm.Component<gz::components::JointPosition>(left_)->Data();
    const auto & pos_r = ecm.Component<gz::components::JointPosition>(right_)->Data();
    const auto & vel_l = ecm.Component<gz::components::JointVelocity>(left_)->Data();
    const auto & vel_r = ecm.Component<gz::components::JointVelocity>(right_)->Data();
    if (pos_l.empty() || pos_r.empty() || vel_l.empty() || vel_r.empty()) return;

    const double torque = -stiffness_ * (pos_l[0] + pos_r[0]) - damping_ * (vel_l[0] + vel_r[0]);
    for (auto joint : {left_, right_}) {
      ecm.Component<gz::components::JointForceCmd>(joint)->Data() = {torque};
    }
  }

private:
  gz::Entity left_{gz::kNullEntity};
  gz::Entity right_{gz::kNullEntity};
  double stiffness_{0.0};
  double damping_{0.0};
};

}  // namespace rover_sim

IGNITION_ADD_PLUGIN(
  rover_sim::RockerDifferential,
  ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPreUpdate)
