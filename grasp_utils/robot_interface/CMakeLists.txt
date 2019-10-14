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
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
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
  HINTS /usr/local/lib)
find_path(ur_modern_driver_INCLUDE_DIRS ur_modern_driver/tcp_socket.h)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# Set include directory path
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${ur_modern_driver_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

# Add robot interface library
set(${PROJECT_NAME}_SOURCES
  src/control_base.cpp
  src/control_ur.cpp
) 
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs geometry_msgs)
target_link_libraries(${PROJECT_NAME} ${ur_modern_driver_LIBRARIES})

# Add test of UR robot interface library
set(TEST_SOURCE
  test/ur_test.cpp)

add_executable(ur_test ${TEST_SOURCE})
ament_target_dependencies(ur_test rclcpp sensor_msgs geometry_msgs)
target_link_libraries(ur_test ${PROJECT_NAME} ${ur_modern_driver_LIBRARIES})

ament_export_include_directories(include)
ament_export_interfaces(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(TARGETS ur_test
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
