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

// Sim time at a fixed rate. Gazebo's own /clock goes out every physics step (250 Hz), and every
// ROS node running on sim time wakes up for each one. 100 Hz is plenty for 2-50 Hz loops.

#include <chrono>
#include <memory>
#include <string>

#include <ignition/gazebo/System.hh>
#include <ignition/msgs/clock.pb.h>  // NOLINT(build/include_order): clang-format sorts it with the .hh headers
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace rover_sim
{

namespace gz = ignition::gazebo;

class SimClock : public gz::System, public gz::ISystemConfigure, public gz::ISystemPostUpdate
{
public:
  void Configure(
    const gz::Entity &,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::EntityComponentManager &,
    gz::EventManager &) override
  {
    period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / sdf->Get<double>("rate")));
    pub_ = node_.Advertise<ignition::msgs::Clock>(sdf->Get<std::string>("topic"));
  }

  void PostUpdate(const gz::UpdateInfo & info, const gz::EntityComponentManager &) override
  {
    if (info.simTime - last_ < period_) return;
    last_ = info.simTime;
    ignition::msgs::Clock msg;
    *msg.mutable_sim() = ignition::msgs::Convert(info.simTime);
    pub_.Publish(msg);
  }

private:
  std::chrono::steady_clock::duration period_{};
  std::chrono::steady_clock::duration last_{};
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher pub_;
};

}  // namespace rover_sim

IGNITION_ADD_PLUGIN(
  rover_sim::SimClock,
  ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPostUpdate)
