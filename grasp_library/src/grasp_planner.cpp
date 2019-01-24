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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include "grasp_library/grasp_planner.hpp"
#include "grasp_library/ros_params.hpp"

GraspPlanner::GraspPlanner(rclcpp::Node * node, GraspPlanningParameters & param)
{
  param_ = param;

  // Setting variables in grasp_candidate_ that are the same for every grasp
  grasp_candidate_.id = "grasp";

  grasp_candidate_.pre_grasp_approach.min_distance = 0.08;
  grasp_candidate_.pre_grasp_approach.desired_distance = 0.1;

  grasp_candidate_.post_grasp_retreat.min_distance = 0.13;
  grasp_candidate_.post_grasp_retreat.desired_distance = 0.15;
  // todo frame_id from MoveIt
  grasp_candidate_.post_grasp_retreat.direction.header.frame_id = "move_group_arm_frame_id";
  grasp_candidate_.post_grasp_retreat.direction.vector.z = 1.0;

  std::map<std::string, double> gripper_joint_values;
  gripper_joint_values["open"] = 0.7;  // todo joint value from MoveIt
  gripper_joint_values["closed"] = 0.05;  // todo joint value from MoveIt
  jointValuesToJointTrajectory(gripper_joint_values, rclcpp::Duration(
      1.0), grasp_candidate_.pre_grasp_posture);
  jointValuesToJointTrajectory(gripper_joint_values, rclcpp::Duration(
      2.0), grasp_candidate_.grasp_posture);
  RCLCPP_INFO(logger_, "Gripper joint value to trajectory...");

  auto service = [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<moveit_msgs::srv::GraspPlanning::Request> req,
      const std::shared_ptr<moveit_msgs::srv::GraspPlanning::Response> res) -> void {
      this->grasp_service(request_header, req, res);
    };
  grasp_srv_ = node->create_service<moveit_msgs::srv::GraspPlanning>("plan_grasps", service);

  RCLCPP_INFO(logger_, "ROS2 Grasp Planning Service up...");
}

void GraspPlanner::grasp_callback(const grasp_msgs::msg::GraspConfigList::SharedPtr msg)
{
  RCLCPP_INFO(logger_, "Received grasp callback");

  std::lock_guard<std::mutex> lock(m_);
  rclcpp::Time grasp_stamp = msg->header.stamp;
  frame_id_ = msg->header.frame_id;
  grasp_candidate_.grasp_pose.header.frame_id = frame_id_;
  grasp_candidate_.pre_grasp_approach.direction.header.frame_id = frame_id_;

  for (auto grasp : msg->grasps) {
    // shift the grasp according to the offset parameter
    grasp.top.x = grasp.top.x + param_.grasp_offset_ * grasp.approach.x;
    grasp.top.y = grasp.top.y + param_.grasp_offset_ * grasp.approach.y;
    grasp.top.z = grasp.top.z + param_.grasp_offset_ * grasp.approach.z;

    // todo grasp_boundry_check
    {
      grasp_candidate_.grasp_pose.pose = grasp_to_pose(grasp);

      grasp_candidate_.grasp_quality = grasp.score.data;
      grasp_candidate_.pre_grasp_approach.direction.vector.x = grasp.approach.x;
      grasp_candidate_.pre_grasp_approach.direction.vector.y = grasp.approach.y;
      grasp_candidate_.pre_grasp_approach.direction.vector.z = grasp.approach.z;

      grasp_candidates_.push_front(std::make_pair(grasp_candidate_, grasp_stamp));
    }
  }
}

geometry_msgs::msg::Pose GraspPlanner::grasp_to_pose(grasp_msgs::msg::GraspConfig & grasp)
{
  RCLCPP_INFO(logger_, "Converting grasp msgs to geometry msgs...");
  geometry_msgs::msg::Pose pose;
  tf2::Matrix3x3 orientation(grasp.approach.x, grasp.binormal.x, grasp.axis.x,
    grasp.approach.y, grasp.binormal.y, grasp.axis.y,
    grasp.approach.z, grasp.binormal.z, grasp.axis.z);

  tf2::Quaternion orientation_quat;
  orientation.getRotation(orientation_quat);
  orientation_quat.normalize();
  pose.orientation.x = orientation_quat.x();
  pose.orientation.y = orientation_quat.y();
  pose.orientation.z = orientation_quat.z();
  pose.orientation.w = orientation_quat.w();

  pose.position = grasp.top;

  return pose;
}

void GraspPlanner::grasp_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<moveit_msgs::srv::GraspPlanning::Request> req,
  const std::shared_ptr<moveit_msgs::srv::GraspPlanning::Response> res)
{
  RCLCPP_INFO(logger_, "Received Grasp Planning request");
  (void)request_header;
  (void)req;

  {
    std::lock_guard<std::mutex> lock(m_);
    for (auto grasp_candidate : grasp_candidates_) {
      // after the first grasp older than the set amount of seconds is found, break the loop
      builtin_interfaces::msg::Time ros_now = rclcpp::Clock(RCL_ROS_TIME).now();
      if (grasp_candidate.second.sec < ros_now.sec - param_.grasp_cache_time_threshold_) {
        // break;
      }
      res->grasps.push_back(grasp_candidate.first);
    }
    grasp_candidates_.clear();
  }

  if (res->grasps.empty()) {
    RCLCPP_INFO(logger_, "No valid grasp found.");
    res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  } else {
    RCLCPP_INFO(logger_, "%ld grasps found.", res->grasps.size());
    res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
}

void GraspPlanner::jointValuesToJointTrajectory(
  std::map<std::string, double> target_values, rclcpp::Duration duration,
  trajectory_msgs::msg::JointTrajectory & grasp_pose)
{
  grasp_pose.joint_names.reserve(target_values.size());
  grasp_pose.points.resize(1);
  grasp_pose.points[0].positions.reserve(target_values.size());

  for (std::map<std::string, double>::iterator it =
    target_values.begin(); it != target_values.end();
    ++it)
  {
    grasp_pose.joint_names.push_back(it->first);
    grasp_pose.points[0].positions.push_back(it->second);
  }
  grasp_pose.points[0].time_from_start = duration;
}
