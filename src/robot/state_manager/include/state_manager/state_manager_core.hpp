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

#ifndef STATE_MANAGER_CORE_HPP_
#define STATE_MANAGER_CORE_HPP_

#include <cstdint>
#include <string>

namespace robot
{

// Coarse operating mode of the rover. Values match the constants in
// rover_state_msgs/msg/RoverState.msg and must stay in sync with them.
enum class RoverStateId : uint8_t
{
  WAIT = 0,     // autonomy halted; no node may command motion
  CONTROL = 1,  // autonomy permitted
};

// Pure state machine: no ROS types, no I/O, no clock. Everything that decides
// *whether* a transition is allowed lives here so it can be unit tested
// directly. The node layer only translates between this and ROS.
class StateMachine
{
public:
  explicit StateMachine(RoverStateId initial = RoverStateId::WAIT);

  // Attempt to move to `requested`.
  //
  // Returns true if the transition was accepted, in which case the internal
  // state has been updated. Returns false if it was rejected, leaving the
  // state untouched and writing an operator-readable explanation into
  // `rejection_reason`.
  bool requestTransition(RoverStateId requested, std::string & rejection_reason);

  RoverStateId state() const
  {
    return state_;
  }

  // True only in states where autonomy nodes are permitted to act.
  bool isAutonomyEnabled() const;

  // Human-readable name, for logs and service responses.
  static const char * toString(RoverStateId state);

  // Value to place in RoverState.state.
  static uint8_t toMsg(RoverStateId state);

  // Parse a state name (e.g. from a parameter). Returns false on an
  // unrecognised name, leaving `out` untouched.
  static bool fromString(const std::string & name, RoverStateId & out);

private:
  RoverStateId state_;
};

}  // namespace robot

#endif  // STATE_MANAGER_CORE_HPP_
