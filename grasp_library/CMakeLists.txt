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
project(grasp_library)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_BUILD_TYPE EQUAL "RELEASE")
  message(STATUS "Create Release Build.")
  set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
else()
  message(STATUS "Create Debug Build.")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grasp_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs)
find_package(visualization_msgs)
find_package(tf2)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(InferenceEngine)

# GPG
find_library(gpg_LIBRARIES grasp_candidates_generator)
find_path(gpg_INCLUDE_DIRS gpg/grasp.h)
# GPD
find_library(gpd_LIBRARIES grasp_pose_detection)
find_path(gpd_INCLUDE_DIRS gpd/grasp_detector.h)

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${builtin_interfaces_INCLUDE_DIRS}
  ${grasp_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${moveit_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${gpg_INCLUDE_DIRS}
  ${gpd_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/consts.cpp
  src/grasp_detector_gpd.cpp
  src/grasp_planner.cpp
  src/ros_params.cpp
  src/grasp_composition.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "builtin_interfaces"
  "grasp_msgs"
  "sensor_msgs"
  "moveit_msgs"
  "visualization_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "trajectory_msgs"
  "pcl_conversions"
)

target_link_libraries(${PROJECT_NAME}
  ${ament_LIBRARIES}
  ${gpg_LIBRARIES}
  ${gpd_LIBRARIES}
  ${PCL_LIBRARIES}
  ${InferenceEngine_LIBRARIES}
)

# Install binaries
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Flags
if(UNIX OR APPLE)
  # Linker flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # GCC specific flags. ICC is compatible with them.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack -z relro -z now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack -z relro -z now")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # In Clang, -z flags are not compatible, they need to be passed to linker via -Wl.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} \
      -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} \
      -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
  endif()

  # Compiler flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    # GCC specific flags.
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR
        CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector-strong")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
    endif()
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # Clang is compatbile with some of the flags.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # Same as above, with exception that ICC compilation crashes with -fPIE option, even
    # though it uses -pie linker option that require -fPIE during compilation. Checksec
    # shows that it generates correct PIE anyway if only -pie is provided.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")
  endif()

  # Generic flags.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fno-operator-names -Wformat -Wformat-security \
    -Wall -fopenmp")
  set( CUDA_PROPAGATE_HOST_FLAGS OFF )
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D_FORTIFY_SOURCE=2")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(tests)
endif()

ament_package()
