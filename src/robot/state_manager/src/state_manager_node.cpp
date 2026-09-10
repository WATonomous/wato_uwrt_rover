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

#include "state_manager/state_manager_node.hpp"

#include <chrono>
#include <memory>
#include <string>

StateManagerNode::StateManagerNode()
: Node("state_manager_node")
, state_machine_(robot::RoverStateId::WAIT)
{
  processParameters();

  // Apply the configured boot state. Anything other than WAIT means autonomy
  // is armed the moment this container starts, so it is called out loudly.
  robot::RoverStateId initial = robot::RoverStateId::WAIT;
  if (!robot::StateMachine::fromString(initial_state_, initial)) {
    RCLCPP_ERROR(
      this->get_logger(), "Unknown initial_state '%s', booting into WAIT", initial_state_.c_str());
    initial = robot::RoverStateId::WAIT;
  }
  if (initial != robot::RoverStateId::WAIT) {
    RCLCPP_WARN(
      this->get_logger(), "Booting directly into %s - autonomy is armed at startup",
      robot::StateMachine::toString(initial));
  }
  state_machine_ = robot::StateMachine(initial);

  // Transient-local so a node that starts (or restarts) after this one still
  // receives the current state on connection instead of waiting for the next
  // publish. Subscribers must request matching durability or they will not
  // connect at all.
  const auto state_qos = rclcpp::QoS(1).transient_local().reliable();
  state_pub_ = this->create_publisher<rover_state_msgs::msg::RoverState>(state_topic_, state_qos);

  // Direct line to the drivetrain topic, used only to force a stop on
  // entering WAIT. Deliberately not routed through any gate.
  halt_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  set_autonomy_srv_ = this->create_service<std_srvs::srv::SetBool>(
    set_autonomy_service_,
    std::bind(
      &StateManagerNode::onSetAutonomy, this, std::placeholders::_1, std::placeholders::_2));

  if (heartbeat_rate_ > 0.0) {
    int period_ms = static_cast<int>(1000.0 / heartbeat_rate_);
    heartbeat_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms), std::bind(&StateManagerNode::onHeartbeat, this));
  }

  // Announce the boot state so consumers are never in the dark about it.
  publishState("startup");

  RCLCPP_INFO(this->get_logger(), "State Manager Node initialized");
  RCLCPP_INFO(
    this->get_logger(), "  Initial state: %s",
    robot::StateMachine::toString(state_machine_.state()));
  RCLCPP_INFO(this->get_logger(), "  State topic: %s", state_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Set service: %s", set_autonomy_service_.c_str());
}

void StateManagerNode::processParameters()
{
  // Declare parameters
  this->declare_parameter<std::string>("state_topic", "/rover_state");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<std::string>("set_autonomy_service", "/rover/set_autonomy");
  this->declare_parameter<std::string>("initial_state", "WAIT");
  this->declare_parameter<double>("heartbeat_rate", 1.0);
  this->declare_parameter<bool>("halt_on_wait", true);

  // Get parameters
  state_topic_ = this->get_parameter("state_topic").as_string();
  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
  set_autonomy_service_ = this->get_parameter("set_autonomy_service").as_string();
  initial_state_ = this->get_parameter("initial_state").as_string();
  heartbeat_rate_ = this->get_parameter("heartbeat_rate").as_double();
  halt_on_wait_ = this->get_parameter("halt_on_wait").as_bool();
}

// ---------------------------------------------------------------------------
// To implement
// ---------------------------------------------------------------------------

void StateManagerNode::onSetAutonomy(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  // TODO(implementer): the explicit set. request->data true means CONTROL,
  //do i need to check if request is empty like i do messages?
  robot::RoverStateId requested;
  if(request->data) //true = control
  {
    requested = robot::RoverStateId::CONTROL;
  }
  else //false = wait
  {
    requested = robot::RoverStateId::WAIT;
  }


  response->success = setState(requested,"user");

  if (response->success) {
    response->message =
      std::string("rover is now in ") + robot::StateMachine::toString(state_machine_.state());
  } 
  else {
    response->message =
      std::string("request refused; rover remains in ") +
      robot::StateMachine::toString(state_machine_.state());
  }
  //   3. Fill response->success with the result and response->message with
  //      something an operator can act on - on rejection, say why, not just
  //      "failed".
  //
  // Keep this callback cheap. It runs on the same single-threaded executor as
  // the heartbeat timer, so anything that blocks here stalls state publishing.

}

bool StateManagerNode::setState(robot::RoverStateId requested, const std::string & reason)
{
  // Capture the current name before the core mutates it, so the log line can
  // show both ends of the transition. toString returns string literals, so
  // holding the pointer across the call is safe.
  const char * previous = robot::StateMachine::toString(state_machine_.state());

  // The core needs a mutable string to write its rejection into. `reason` is
  // the caller's origin tag ("user", "startup") and is const, so it cannot
  // double as that out-parameter.
  std::string rejection_reason;

  if (!state_machine_.requestTransition(requested, rejection_reason)) {
    RCLCPP_WARN(
      this->get_logger(), "Transition %s -> %s rejected: %s (requested by: %s)", previous,
      robot::StateMachine::toString(requested), rejection_reason.c_str(), reason.c_str());

    // A rejected WAIT request still means somebody asked the rover to stop.
    // Honour that regardless of whether the state actually changed, so a
    // repeated stop is never a no-op at the drivetrain.
    if (requested == robot::RoverStateId::WAIT && halt_on_wait_) {
      haltMotion();
    }

    // Nothing changed, so nothing is published.
    return false;
  }

  // Stop first, publish second: the zero twist should be in flight before
  // anything downstream reacts to the state change.
  if (state_machine_.state() == robot::RoverStateId::WAIT && halt_on_wait_) {
    haltMotion();
  }

  publishState(reason);

  RCLCPP_INFO(
    this->get_logger(), "State %s -> %s (%s)", previous,
    robot::StateMachine::toString(state_machine_.state()), reason.c_str());

  return true;
}

void StateManagerNode::onHeartbeat()
{
  // TODO(implementer): republish the current state so consumers can detect a
  // dead state manager by message age.
  //
  // The reason string here should mark it as a heartbeat rather than a
  // transition ("heartbeat"), so log readers can tell the two apart.
  publishState("heartbeat");
}

// ---------------------------------------------------------------------------
// Provided plumbing
// ---------------------------------------------------------------------------

void StateManagerNode::publishState(const std::string & reason)
{
  rover_state_msgs::msg::RoverState msg;
  msg.header.stamp = this->now();
  msg.state = robot::StateMachine::toMsg(state_machine_.state());
  msg.reason = reason;
  state_pub_->publish(msg);
}

void StateManagerNode::haltMotion()
{
  // An all-zero Twist. Sent once, directly to the drivetrain topic; holding
  // the rover stopped afterwards is the gate's job, not this node's.
  geometry_msgs::msg::Twist stop_cmd;
  halt_pub_->publish(stop_cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateManagerNode>());
  rclcpp::shutdown();
  return 0;
}
