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
project(eval)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_core REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(eval_node src/eval_node.cpp)
target_link_libraries(eval_node rover_core::rover_core)
ament_target_dependencies(eval_node nav_msgs rclcpp rover_msgs tf2_ros)

install(TARGETS eval_node DESTINATION lib/${PROJECT_NAME})

ament_package()
