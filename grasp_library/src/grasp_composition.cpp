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

#include "grasp_library/grasp_detector_gpd.hpp"
#include "grasp_library/grasp_planner.hpp"

rclcpp::Node::SharedPtr detector_node, planner_node;

void thread(rclcpp::Node::SharedPtr node)
{
  rclcpp::spin(node);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  detector_node = std::make_shared<GraspDetectorGPD>();

  GraspDetectorBase * grasp_detector = dynamic_cast<GraspDetectorBase *>(detector_node.get());

  planner_node = std::make_shared<GraspPlanner>(grasp_detector);

  std::thread th(thread, planner_node);
  th.detach();

  rclcpp::spin(detector_node);

  detector_node = nullptr;
  planner_node = nullptr;
  rclcpp::shutdown();
  return 0;
}
