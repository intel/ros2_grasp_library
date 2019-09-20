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

#ifndef GRASP_LIBRARY_ROS2_GRASP_PLANNER_HPP_
#define GRASP_LIBRARY_ROS2_GRASP_PLANNER_HPP_

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <grasp_msgs/msg/grasp_config_list.hpp>
#include <moveit_msgs/msg/grasp.h>
#include <moveit_msgs/srv/grasp_planning.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "grasp_library/ros2/grasp_detector_base.hpp"

namespace grasp_ros2
{

/** GraspPlanner class
 *
 * \brief A MoveIt grasp planner
 *
 * This class provide ROS service for MoveIt grasp planning. Grasp Planner drives grasp detection
 * and takes the results from Grasp Detector.
*/
class GraspPlanner : public rclcpp::Node, public GraspCallback
{
public:
  struct GraspPlanningParameters
  {
    /** timeout in seconds for a service request waiting for grasp detection result*/
    int grasp_service_timeout_;
    /** minimum score expected for grasps returned from this service*/
    int grasp_score_threshold_;
    /** frame id expected for grasps returned from this service*/
    std::string grasp_frame_id_;
    /** approach direction in grasp_frame_id_ expected for grasps*/
    tf2::Vector3 grasp_approach_;
    /** maxmimum angle in radian acceptable between the expected 'approach_' and
     * the real approach returned from this service*/
    double grasp_approach_angle_;
    /** offset [x, y, z] in metres applied to the grasps detected*/
    std::vector<double> grasp_offset_;
    /** boundry cube in grasp_frame_id_ expected for grasps returned from this service*/
    std::vector<double> grasp_boundry_;
    /** offset in metres from the gripper base (finger root) to the parent link of gripper*/
    double eef_offset;
    /** gripper yaw offset to its parent link, in radian (e.g. 0.0, or M_PI/4)*/
    double eef_yaw_offset;
    /** minimum distance in metres for a grasp to approach and retreat*/
    double grasp_min_distance_;
    /** desired distance in metres for a grasp to approach and retreat*/
    double grasp_desired_distance_;
    /** joint names of gripper fingers*/
    std::vector<std::string> finger_joint_names_;
    /** trajectory points in 'open' status, for joints in the same order as 'finger_joint_names_'*/
    trajectory_msgs::msg::JointTrajectoryPoint finger_points_open_;
    /** trajectory points in 'close' status, for joints in the same order as 'finger_joint_names_'*/
    trajectory_msgs::msg::JointTrajectoryPoint finger_points_close_;
  };

  /**
   * \brief Constructor.
   * \param grasp_detector Grasp Detector used by this planner.
  */
  explicit GraspPlanner(const rclcpp::NodeOptions & options, GraspDetectorBase * grasp_detector = nullptr);

  /**
   * \brief Destructor.
  */
  ~GraspPlanner()
  {
    delete tfBuffer_;
  }

  void grasp_callback(const grasp_msgs::msg::GraspConfigList::SharedPtr msg);

  /**
   * \brief Grasp planning service handler.
   * When a grasp service request comes, Grasp Planner tells the Grasp Detector to start grasp
   * detection, waits for grasp callback arrival or till a configurable timeout period, then stops
   * grasp detection, skips grasps with low scores, transforms grasps into the specified frame_id
   * (if TF available), applies the configured offset, skips grasps out of boundry, and returns the
   * results via grasp service response.
  */
  void grasp_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<moveit_msgs::srv::GraspPlanning::Request> req,
    const std::shared_ptr<moveit_msgs::srv::GraspPlanning::Response> res);

private:
  /**
   * \brief Transform a grasp from original frame to the 'grasp_frame_id_' frame.
   * Keep 'to' grasp identical to 'from' grasp, in case of transform missing or failure.
   * \param from The grasp to transform.
   * \param to The transformed output.
   * \param header Message header for the frame of the 'from' grasp.
   * \return true if transformation success, otherwise false.
   */
  bool transform(
    grasp_msgs::msg::GraspConfig & from, grasp_msgs::msg::GraspConfig & to,
    const std_msgs::msg::Header & header);

  /**
   * \brief Check if the grasp position is in boundary.
   * \param p Grasp position.
   * \return True if the grasp position in boundary, otherwise False.
   */
  bool check_boundry(const geometry_msgs::msg::Point & p);

  /**
   * \brief Translate a grasp message to MoveIt message.
   * 'Grasp.grasp_pose.pose.position' was translated from 'GraspConfig.bottom', which is the
   * position closest to the 'parent_link' of the end-effector.
   * \param grasp Grasp message to be translated.
   * \header Message header for the frame where the 'grasp' was detected.
   * \return MoveIt message
   */
  moveit_msgs::msg::Grasp toMoveIt(
    grasp_msgs::msg::GraspConfig & grasp,
    const std_msgs::msg::Header & header);

  std::mutex m_;
  std::condition_variable cv_;
  GraspPlanningParameters param_;
  rclcpp::Logger logger_ = rclcpp::get_logger("GraspPlanner");
  /** buffer for grasps to be returned from this service*/
  std::vector<moveit_msgs::msg::Grasp> moveit_grasps_;
  rclcpp::Service<moveit_msgs::srv::GraspPlanning>::SharedPtr grasp_srv_; /**< grasp service*/
  tf2_ros::Buffer * tfBuffer_; /**< buffer for transformation listener*/
  GraspDetectorBase * grasp_detector_; /**< grasp detector node*/
};

}  // namespace grasp_ros2

#endif  // GRASP_LIBRARY_ROS2_GRASP_PLANNER_HPP_
