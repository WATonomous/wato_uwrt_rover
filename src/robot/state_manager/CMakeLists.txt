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
project(state_manager)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED) # ROS2 build tool
find_package(rclcpp REQUIRED)      # ROS2 C++ package
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rover_state_msgs REQUIRED)

# Compiles source files into a library
# The core holds the pure state machine, with no ROS I/O, so the transition
# rules can be unit tested without spinning a node.
add_library(state_manager_lib
  src/state_manager_core.cpp)
# Indicate to compiler where to search for header files
target_include_directories(state_manager_lib
  PUBLIC include)
# Add ROS2 dependencies required by package
ament_target_dependencies(state_manager_lib
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  rover_state_msgs
)

# Create ROS2 node executable from source files
add_executable(state_manager_node src/state_manager_node.cpp)
# Link to the previously built library
target_link_libraries(state_manager_node state_manager_lib)

# Copy executable to installation location
install(TARGETS
  state_manager_node
  DESTINATION lib/${PROJECT_NAME})

# Copy config files to installation location
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
