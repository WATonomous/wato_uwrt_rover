# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.10)
project(rover_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

add_library(RockerDifferential SHARED src/rocker_differential.cpp)
target_link_libraries(RockerDifferential
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::register
)

add_library(JointStates SHARED src/joint_states.cpp)
target_link_libraries(JointStates
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::register
)

add_library(SimClock SHARED src/sim_clock.cpp)
target_link_libraries(SimClock
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::register
)

install(TARGETS RockerDifferential JointStates SimClock LIBRARY DESTINATION lib)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
