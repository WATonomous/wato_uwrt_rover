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

#include "state_manager/state_manager_core.hpp"

#include <string>

namespace robot
{

StateMachine::StateMachine(RoverStateId initial)
: state_(initial)
{}

bool StateMachine::requestTransition(RoverStateId requested, std::string & rejection_reason)
{
  // TODO(implementer): decide whether `requested` is reachable from state_.
  //
  // With only WAIT and CONTROL the rules are nearly trivial, but put them
  // here rather than in the node so they stay testable and so adding E_STOP
  // or TELEOP later is a change in one place:
  //
  //   1. requested == state_        -> accept as a no-op, or reject with
  //                                    "already in <state>"; pick one and be
  //                                    consistent, because the service
  //                                    response is what the operator sees.
  if (requested == state_)
  {
    rejection_reason = "Already in requested state";
    return false;
  }
  //right now as long as request isn't the current state, accept, but later will add conditions
  //needed to go from wait to control(ie everything is initialized properly)
  state_ = requested;
  return true;

  // On acceptance, assign state_ = requested and return true.
  // On rejection, leave state_ untouched, set rejection_reason, return false.
}

bool StateMachine::isAutonomyEnabled() const
{
  return state_ == RoverStateId::CONTROL;
}

const char * StateMachine::toString(RoverStateId state)
{
  switch (state) {
    case RoverStateId::WAIT:
      return "WAIT";
    case RoverStateId::CONTROL:
      return "CONTROL";
  }
  return "UNKNOWN";
}

uint8_t StateMachine::toMsg(RoverStateId state)
{
  return static_cast<uint8_t>(state);
}

bool StateMachine::fromString(const std::string & name, RoverStateId & out)
{
  if (name == "WAIT") {
    out = RoverStateId::WAIT;
    return true;
  }
  if (name == "CONTROL") {
    out = RoverStateId::CONTROL;
    return true;
  }
  return false;
}

}  // namespace robot
