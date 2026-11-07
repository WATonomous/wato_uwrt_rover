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
project(rover_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# The world, rover model and orbital map are generated from the two config files.
set(GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
add_custom_command(
  OUTPUT ${GEN_DIR}/worlds/mars.sdf
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_world.py
    --rover ${CMAKE_CURRENT_SOURCE_DIR}/config/rover.yaml
    --world ${CMAKE_CURRENT_SOURCE_DIR}/config/world.yaml
    --out ${GEN_DIR}
  DEPENDS tools/gen_world.py config/rover.yaml config/world.yaml
  COMMENT "Generating Mars world, rover model and orbital map"
)
add_custom_target(generate_world ALL DEPENDS ${GEN_DIR}/worlds/mars.sdf)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${GEN_DIR}/models ${GEN_DIR}/worlds ${GEN_DIR}/orbital DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/gen_world.py DESTINATION lib/${PROJECT_NAME})

ament_package()
