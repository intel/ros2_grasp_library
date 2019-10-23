// Copyright (c) 2018 Intel Corporation. All Rights Reserved
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

#ifndef GRASP_LIBRARY__ROS2__ROS_PARAMS_HPP_
#define GRASP_LIBRARY__ROS2__ROS_PARAMS_HPP_

// ROS2 core
#include <rclcpp/rclcpp.hpp>

// ROS2 projects
#include <gpd/grasp_detector.h>
#include "grasp_library/ros2/grasp_planner.hpp"

namespace grasp_ros2
{

/** ROSParameters class
 *
 * \brief A class to bridge parameters passed from ROS.
 *
*/
class ROSParameters
{
public:
  static void getDetectionParams(
    rclcpp::Node * node,
    GraspDetector::GraspDetectionParameters & param);
  static void getPlanningParams(rclcpp::Node * Node, GraspPlanner::GraspPlanningParameters & param);
};

}  // namespace grasp_ros2

#endif  // GRASP_LIBRARY__ROS2__ROS_PARAMS_HPP_
