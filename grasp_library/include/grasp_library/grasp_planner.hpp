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

#ifndef GRASP_LIBRARY__GRASP_PLANNER_HPP_
#define GRASP_LIBRARY__GRASP_PLANNER_HPP_

// ROS2
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

// this project (messages)
#include <grasp_msgs/msg/grasp_config_list.hpp>
#include <moveit_msgs/srv/grasp_planning.hpp>

// system
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

/** GraspPlanner class
 *
 * \brief A MoveIt grasp planner
 *
 * This class provide ROS service for MoveIt grasp planning
 *
*/
class GraspPlanner
{
public:
  struct GraspPlanningParameters
  {
    /** offset of the grasp along the approach vector*/
    double grasp_offset_;
    /** grasps older than this threshold are not considered anymore*/
    int32_t grasp_cache_time_threshold_;
  };

  /**
   * \brief Constructor.
  */
  GraspPlanner(rclcpp::Node * node, GraspPlanningParameters & param);

  /**
   * \brief Destructor.
  */
  ~GraspPlanner()
  {
  }

  void grasp_callback(const grasp_msgs::msg::GraspConfigList::SharedPtr msg);

  /**
   * \brief Grasp planning service handler
  */
  void grasp_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<moveit_msgs::srv::GraspPlanning::Request> req,
    const std::shared_ptr<moveit_msgs::srv::GraspPlanning::Response> res);

private:
  void jointValuesToJointTrajectory(
    std::map<std::string, double> target_values, rclcpp::Duration duration,
    trajectory_msgs::msg::JointTrajectory & grasp_pose);

  geometry_msgs::msg::Pose grasp_to_pose(grasp_msgs::msg::GraspConfig & grasp);

  /** ROS service for grasp planning*/
  rclcpp::Service<moveit_msgs::srv::GraspPlanning>::SharedPtr grasp_srv_;
  rclcpp::Logger logger_ = rclcpp::get_logger("GraspPlanner");
  GraspPlanningParameters param_;

  moveit_msgs::msg::Grasp grasp_candidate_;
  std::deque<std::pair<moveit_msgs::msg::Grasp, builtin_interfaces::msg::Time>> grasp_candidates_;
  std::mutex m_;

  // frame of the grasp
  std::string frame_id_;

  // todo table top boundry check
};

#endif  // GRASP_LIBRARY__GRASP_PLANNER_HPP_
