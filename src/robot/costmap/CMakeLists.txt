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
project(costmap)

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
find_package(geometry_msgs REQUIRED)      # ROS2 C++ package
find_package(sensor_msgs REQUIRED)      # ROS2 C++ package
find_package(nav_msgs REQUIRED)      # ROS2 C++ package
find_package(tf2 REQUIRED)                # tf2::Transform / Quaternion / Matrix3x3
find_package(tf2_ros REQUIRED)            # Buffer + TransformListener in the node
find_package(tf2_geometry_msgs REQUIRED)  # tf2::fromMsg for geometry_msgs types

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(costmap_lib
  src/costmap_core.cpp)
# Indicate to compiler where to search for header files
target_include_directories(costmap_lib
  PUBLIC include)
# Add ROS2 dependencies required by package
# costmap_core.cpp itself uses tf2 and tf2_geometry_msgs, so the LIBRARY needs
# them -- not just the node target.
ament_target_dependencies(costmap_lib
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Create ROS2 node executable from source files
add_executable(costmap_node src/costmap_node.cpp)
# Link to the previously built library to access costmap classes and methods
target_link_libraries(costmap_node costmap_lib)
# tf2_ros::Buffer and TransformListener are used directly in costmap_node.cpp.
# ament_target_dependencies on the lib is PUBLIC-ish for includes but the node
# still needs its own declaration to link cleanly.
ament_target_dependencies(costmap_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Copy executable to installation location
install(TARGETS
  costmap_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
