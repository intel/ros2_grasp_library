// Copyright (c) 2019 Intel Corporation
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

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "grasp_library/ros2/grasp_detector_gpd.hpp"
#include "grasp_library/ros2/grasp_planner.hpp"

using GraspDetectorGPD = grasp_ros2::GraspDetectorGPD;
using GraspDetectorBase = grasp_ros2::GraspDetectorBase;
using GraspPlanner = grasp_ros2::GraspPlanner;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  auto detect_node = std::make_shared<GraspDetectorGPD>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  exec.add_node(detect_node);
  GraspDetectorBase * grasp_detector = dynamic_cast<GraspDetectorBase *>(detect_node.get());
  auto plan_node = std::make_shared<GraspPlanner>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true), grasp_detector);
  exec.add_node(plan_node);
  exec.spin();

  detect_node = nullptr;
  plan_node = nullptr;
  rclcpp::shutdown();
  return 0;
}
