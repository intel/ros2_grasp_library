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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "grasp_library/consts.hpp"
#include "grasp_library/grasp_planner.hpp"
#include "grasp_library/ros_params.hpp"

using GraspPlanning = moveit_msgs::srv::GraspPlanning;

GraspPlanner::GraspPlanner(GraspDetectorBase * grasp_detector)
: Node("GraspPlanner", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  GraspCallback(), grasp_detector_(grasp_detector)
{
  ROSParameters::getPlanningParams(this, param_);
  auto service = [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<GraspPlanning::Request> req,
      const std::shared_ptr<GraspPlanning::Response> res) -> void {
      this->grasp_service(request_header, req, res);
    };
  grasp_srv_ = this->create_service<GraspPlanning>("plan_grasps", service);

  grasp_detector_->add_callback(this);

  tfBuffer_ = new tf2_ros::Buffer(this->get_clock());

  RCLCPP_INFO(logger_, "ROS2 Grasp Planning Service up...");
}

void GraspPlanner::grasp_callback(const grasp_msgs::msg::GraspConfigList::SharedPtr msg)
{
  RCLCPP_INFO(logger_, "Received grasp callback");

  static bool tf_available = tfBuffer_->canTransform(param_.grasp_frame_id_, msg->header.frame_id,
      tf2_ros::fromMsg(msg->header.stamp), tf2::durationFromSec(0));
  std_msgs::msg::Header header;
  header.frame_id = tf_available ? param_.grasp_frame_id_ : msg->header.frame_id;
  header.stamp = msg->header.stamp;
  grasp_msgs::msg::GraspConfig to_grasp;

  std::unique_lock<std::mutex> lock(m_);
  for (auto from_grasp : msg->grasps) {
    // skip low score grasp
    if (from_grasp.score.data < param_.grasp_score_threshold_) {
      RCLCPP_DEBUG(logger_, "skip low score grasps");
      continue;
    }
    // transform grasp to grasp_frame_id
    if (tf_available) {
      if (!transform(from_grasp, to_grasp, msg->header)) {
        // skip failed grasp
        continue;
      }
    }
    if (param_.grasp_approach_angle_ != M_PI) {
      // skip unacceptable approach
      tf2::Vector3 approach(to_grasp.approach.x, to_grasp.approach.y, to_grasp.approach.z);
      double ang = tf2::tf2Angle(approach, param_.grasp_approach_);
      if (std::isnan(ang) ||
        ang < -param_.grasp_approach_angle_ || ang > param_.grasp_approach_angle_)
      {
        RCLCPP_INFO(logger_, "skip unacceptable approach");
        continue;
      }
    }
    // apply grasp offset
    to_grasp.bottom.x += param_.grasp_offset_[0];
    to_grasp.bottom.y += param_.grasp_offset_[1];
    to_grasp.bottom.z += param_.grasp_offset_[2];
    // skip out of boundary grasps
    if (!tf_available || check_boundry(to_grasp.bottom)) {
      // translate into moveit grasp
      moveit_grasps_.push_back(toMoveIt(to_grasp, header));
    }
  }

  if (!moveit_grasps_.empty()) {
    cv_.notify_all();
  }
}

bool GraspPlanner::transform(
  grasp_msgs::msg::GraspConfig & from, grasp_msgs::msg::GraspConfig & to,
  const std_msgs::msg::Header & header)
{
  static tf2_ros::TransformListener tfListener(*tfBuffer_);
  geometry_msgs::msg::PointStamped from_top, to_top, from_surface, to_surface,
    from_bottom, to_bottom;
  geometry_msgs::msg::Vector3Stamped from_approach, to_approach, from_binormal, to_binormal,
    from_axis, to_axis;

  to = from;

  from_top.point = from.top;
  from_top.header = header;
  from_surface.point = from.surface;
  from_surface.header = header;
  from_bottom.point = from.bottom;
  from_bottom.header = header;
  from_approach.vector = from.approach;
  from_approach.header = header;
  from_binormal.vector = from.binormal;
  from_binormal.header = header;
  from_axis.vector = from.axis;
  from_axis.header = header;
  try {
    tfBuffer_->transform(from_top, to_top, param_.grasp_frame_id_);
    tfBuffer_->transform(from_surface, to_surface, param_.grasp_frame_id_);
    tfBuffer_->transform(from_bottom, to_bottom, param_.grasp_frame_id_);
    tfBuffer_->transform(from_approach, to_approach, param_.grasp_frame_id_);
    tfBuffer_->transform(from_binormal, to_binormal, param_.grasp_frame_id_);
    tfBuffer_->transform(from_axis, to_axis, param_.grasp_frame_id_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "transform exception");
    return false;
  }

  to.top = to_top.point;
  to.surface = to_surface.point;
  to.bottom = to_bottom.point;
  to.approach = to_approach.vector;
  to.binormal = to_binormal.vector;
  to.axis = to_axis.vector;
  return true;
}

bool GraspPlanner::check_boundry(const geometry_msgs::msg::Point & p)
{
  RCLCPP_INFO(logger_, "point [%f %f %f]", p.x, p.y, p.z);
  return p.x >= param_.grasp_boundry_[0] && p.x <= param_.grasp_boundry_[1] &&
         p.y >= param_.grasp_boundry_[2] && p.y <= param_.grasp_boundry_[3] &&
         p.z >= param_.grasp_boundry_[4] && p.z <= param_.grasp_boundry_[5];
}

moveit_msgs::msg::Grasp GraspPlanner::toMoveIt(
  grasp_msgs::msg::GraspConfig & grasp,
  const std_msgs::msg::Header & header)
{
  moveit_msgs::msg::Grasp msg;
  msg.grasp_pose.header = header;
  msg.grasp_quality = grasp.score.data;

  // set grasp position
  msg.grasp_pose.pose.position = grasp.bottom;
  // set grasp position, translation from hand-base to the parent-link of EEF
  msg.grasp_pose.pose.position.x = grasp.bottom.x - grasp.approach.x * param_.eef_offset;
  msg.grasp_pose.pose.position.y = grasp.bottom.y - grasp.approach.y * param_.eef_offset;
  msg.grasp_pose.pose.position.z = grasp.bottom.z - grasp.approach.z * param_.eef_offset;

  // rotation matrix https://github.com/atenpas/gpd/blob/master/tutorials/hand_frame.png
  tf2::Matrix3x3 r(
    grasp.binormal.x, grasp.axis.x, grasp.approach.x,
    grasp.binormal.y, grasp.axis.y, grasp.approach.y,
    grasp.binormal.z, grasp.axis.z, grasp.approach.z);
  tf2::Quaternion quat;
  r.getRotation(quat);
  // EEF yaw-offset to its parent-link (last link of arm)
  quat *= tf2::Quaternion(tf2::Vector3(0, 0, 1), param_.eef_yaw_offset);
  quat.normalize();
  // set grasp orientation
  msg.grasp_pose.pose.orientation = tf2::toMsg(quat);

  // set pre-grasp approach
  msg.pre_grasp_approach.direction.header = header;
  msg.pre_grasp_approach.direction.vector = grasp.approach;
  msg.pre_grasp_approach.min_distance = param_.grasp_min_distance_;
  msg.pre_grasp_approach.desired_distance = param_.grasp_desired_distance_;

  // set post-grasp retreat
  msg.post_grasp_retreat.direction.header = header;
  msg.post_grasp_retreat.direction.vector.x = -grasp.approach.x;
  msg.post_grasp_retreat.direction.vector.y = -grasp.approach.y;
  msg.post_grasp_retreat.direction.vector.z = -grasp.approach.z;
  msg.post_grasp_retreat.min_distance = param_.grasp_min_distance_;
  msg.post_grasp_retreat.desired_distance = param_.grasp_desired_distance_;

  // set pre-grasp posture
  msg.pre_grasp_posture.joint_names = param_.finger_joint_names_;
  msg.pre_grasp_posture.points.push_back(param_.finger_points_open_);

  // set grasp posture
  msg.grasp_posture.joint_names = param_.finger_joint_names_;
  msg.grasp_posture.points.push_back(param_.finger_points_close_);

  return msg;
}

void GraspPlanner::grasp_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GraspPlanning::Request> req,
  const std::shared_ptr<GraspPlanning::Response> res)
{
  (void)request_header;
  (void)req;
  RCLCPP_INFO(logger_, "Received Grasp Planning request");

  {
    std::unique_lock<std::mutex> lock(m_);
    moveit_grasps_.clear();
    grasp_detector_->start();
    cv_.wait_for(lock, std::chrono::seconds(param_.grasp_service_timeout_));
  }
  res->grasps = moveit_grasps_;
  grasp_detector_->stop();

  if (res->grasps.empty()) {
    RCLCPP_INFO(logger_, "No expected grasp found.");
    res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  } else {
    RCLCPP_INFO(logger_, "%ld grasps found.", res->grasps.size());
    res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
}
