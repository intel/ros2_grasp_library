# Copyright (c) 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(handeye_target_detection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (WITH_OPENCL OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED)
if((${OpenCV_VERSION} LESS 3.3))
  message(WARNING "handeye_target_detection works better with OpenCV version >= 3.3")
endif()

# Set include directory path
include_directories(
  include 
  ${rclcpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(SOURCES_POSEESTIMATION
  src/pose_estimation_node.cpp 
  src/pose_estimator.cpp
 	)

add_executable(pose_estimation ${SOURCES_POSEESTIMATION})
ament_target_dependencies(pose_estimation rclcpp tf2 tf2_msgs tf2_ros std_msgs sensor_msgs cv_bridge image_transport)
target_link_libraries(pose_estimation
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBS}
)

# Install target files
install(
  TARGETS pose_estimation
  DESTINATION lib/${PROJECT_NAME}/
)

# Install header files  
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files.
install(DIRECTORY
  launch cfg data
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
