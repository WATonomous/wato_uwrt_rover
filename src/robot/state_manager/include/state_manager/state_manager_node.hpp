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

#ifndef STATE_MANAGER_NODE_HPP_
#define STATE_MANAGER_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_state_msgs/msg/rover_state.hpp"
#include "state_manager/state_manager_core.hpp"
#include "std_srvs/srv/set_bool.hpp"

// Single owner of the rover's operating state.
//
// Every other node treats /rover_state as read-only truth; this is the only
// node permitted to change it.
class StateManagerNode : public rclcpp::Node
{
public:
  StateManagerNode();

private:
  void processParameters();

  // ---- To implement ------------------------------------------------------
  // Service entry points and the transition path. See the .cpp for what each
  // one is expected to do.
  void onSetAutonomy(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  bool setState(robot::RoverStateId requested, const std::string & reason);

  void onHeartbeat();

  // ---- Provided plumbing -------------------------------------------------
  void publishState(const std::string & reason);
  void haltMotion();

  // The state machine itself. All transition rules live in the core.
  robot::StateMachine state_machine_;

  rclcpp::Publisher<rover_state_msgs::msg::RoverState>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr halt_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_autonomy_srv_;

  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Configuration parameters
  std::string state_topic_;
  std::string cmd_vel_topic_;
  std::string set_autonomy_service_;
  std::string initial_state_;

  double heartbeat_rate_;
  bool halt_on_wait_;
};

#endif  // STATE_MANAGER_NODE_HPP_
