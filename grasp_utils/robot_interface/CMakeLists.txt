# Copyright (c) 2018 Intel Corporation
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
project(robot_interface)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_library(
  ur_modern_driver_LIBRARIES 
  NAMES ur_driver_lib
  HINTS /usr/local/lib/ur_modern_driver)
find_path(ur_modern_driver_INCLUDE_DIRS ur_modern_driver/tcp_socket.h)

# Set include directory path
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${ur_modern_driver_INCLUDE_DIRS})

# Add robot interface library
set(${PROJECT_NAME}_LIB_SOURCES
  src/control_base.cpp
) 
add_library(${PROJECT_NAME}_lib ${${PROJECT_NAME}_LIB_SOURCES})

# Add UR robot interface library
set(UR_LIB_SOURCES
  src/ur_control.cpp
)
add_library(ur_lib ${UR_LIB_SOURCES})
ament_target_dependencies(ur_lib rclcpp sensor_msgs geometry_msgs)
target_link_libraries(ur_lib ${PROJECT_NAME}_lib ${ur_modern_driver_LIBRARIES})

# Add test of UR robot interface library
set(TEST_SOURCE
  test/ur_test.cpp)

add_executable(ur_test ${TEST_SOURCE})
ament_target_dependencies(ur_test rclcpp sensor_msgs)
target_link_libraries(ur_test ur_lib ${PROJECT_NAME}_lib ${ur_modern_driver_LIBRARIES})

# Install library and executables
install(TARGETS ur_lib ur_test
  DESTINATION lib/${PROJECT_NAME})

# Install header files  
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
