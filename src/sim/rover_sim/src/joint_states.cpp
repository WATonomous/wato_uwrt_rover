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

// Joint states (wheel encoders, suspension angles) at a fixed rate. Fortress's own
// JointStatePublisher publishes every physics step (hundreds of Hz), which every ROS node then
// has to process; real encoders report at tens of Hz.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/msgs/model.pb.h>  // NOLINT(build/include_order): clang-format sorts it with the .hh headers
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace rover_sim
{

namespace gz = ignition::gazebo;

class JointStates : public gz::System, public gz::ISystemConfigure, public gz::ISystemPostUpdate
{
public:
  void Configure(
    const gz::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::EntityComponentManager & ecm,
    gz::EventManager &) override
  {
    gz::Model model(entity);
    name_ = model.Name(ecm);
    period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / sdf->Get<double>("rate")));
    joints_ = model.Joints(ecm);
    for (auto joint : joints_) {
      if (!ecm.Component<gz::components::JointPosition>(joint)) {
        ecm.CreateComponent(joint, gz::components::JointPosition());
      }
      if (!ecm.Component<gz::components::JointVelocity>(joint)) {
        ecm.CreateComponent(joint, gz::components::JointVelocity());
      }
    }
    pub_ = node_.Advertise<ignition::msgs::Model>(sdf->Get<std::string>("topic"));
  }

  void PostUpdate(const gz::UpdateInfo & info, const gz::EntityComponentManager & ecm) override
  {
    if (info.paused || info.simTime - last_ < period_) return;
    last_ = info.simTime;

    ignition::msgs::Model msg;
    msg.set_name(name_);
    *msg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(info.simTime);
    for (auto joint : joints_) {
      const auto & pos = ecm.Component<gz::components::JointPosition>(joint)->Data();
      const auto & vel = ecm.Component<gz::components::JointVelocity>(joint)->Data();
      if (pos.empty() || vel.empty()) continue;
      auto * j = msg.add_joint();
      j->set_name(ecm.Component<gz::components::Name>(joint)->Data());
      j->mutable_axis1()->set_position(pos[0]);
      j->mutable_axis1()->set_velocity(vel[0]);
    }
    pub_.Publish(msg);
  }

private:
  std::string name_;
  std::vector<gz::Entity> joints_;
  std::chrono::steady_clock::duration period_{};
  std::chrono::steady_clock::duration last_{};
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher pub_;
};

}  // namespace rover_sim

IGNITION_ADD_PLUGIN(
  rover_sim::JointStates,
  ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPostUpdate)
