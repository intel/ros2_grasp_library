// Copyright (c) 2019 Intel Corporation. All Rights Reserved
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

/**
 * @file control_base.cpp 
 */

#include <robot_interface/control_base.hpp>
#include <chrono>
#include <thread>

void ArmControlBase::publishTFGoal()
{
  while (rclcpp::ok())
  {
    broadcaster_.sendTransform(tf_msg_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void ArmControlBase::updateTFGoal(const geometry_msgs::msg::PoseStamped& pose_stamped)
{
  std::unique_lock<std::mutex> lock(m_);
  tf_msg_.transform.translation.x = pose_stamped.pose.position.x;
  tf_msg_.transform.translation.y = pose_stamped.pose.position.y;
  tf_msg_.transform.translation.z = pose_stamped.pose.position.z;
  tf_msg_.transform.rotation.x = pose_stamped.pose.orientation.x;
  tf_msg_.transform.rotation.y = pose_stamped.pose.orientation.y;
  tf_msg_.transform.rotation.z = pose_stamped.pose.orientation.z;
  tf_msg_.transform.rotation.w = pose_stamped.pose.orientation.w;
  tf_msg_.header.stamp = this->now();
  tf_msg_.header.frame_id = pose_stamped.header.frame_id;
}

bool ArmControlBase::moveToTcpPose(const Eigen::Isometry3d& pose, double vel, double acc)
{
  TcpPose tcp_pose;
  toTcpPose(pose, tcp_pose);
  return this->moveToTcpPose(tcp_pose.x, tcp_pose.y, tcp_pose.z, 
                             tcp_pose.alpha, tcp_pose.beta, tcp_pose.gamma, vel, acc);
}

bool ArmControlBase::moveToTcpPose(const geometry_msgs::msg::PoseStamped& pose_stamped, double vel, double acc)
{
  updateTFGoal(pose_stamped);

  TcpPose tcp_pose;
  toTcpPose(pose_stamped, tcp_pose);
  return this->moveToTcpPose(tcp_pose.x, tcp_pose.y, tcp_pose.z, 
                             tcp_pose.alpha, tcp_pose.beta, tcp_pose.gamma, vel, acc);
}

void ArmControlBase::toTcpPose(const geometry_msgs::msg::PoseStamped& pose_stamped, TcpPose& tcp_pose)
{
  tcp_pose.x = pose_stamped.pose.position.x;
  tcp_pose.y = pose_stamped.pose.position.y;
  tcp_pose.z = pose_stamped.pose.position.z;

  tf2::Matrix3x3 r(tf2::Quaternion(pose_stamped.pose.orientation.x, 
                    pose_stamped.pose.orientation.y, 
                    pose_stamped.pose.orientation.z, 
                    pose_stamped.pose.orientation.w));
  r.getRPY(tcp_pose.alpha, tcp_pose.beta, tcp_pose.gamma);
}

void ArmControlBase::toTcpPose(const Eigen::Isometry3d& pose, TcpPose& tcp_pose)
{
  tcp_pose.x = pose.translation().x();
  tcp_pose.y = pose.translation().y();
  tcp_pose.z = pose.translation().z();

  Eigen::Vector3d euler_angles = pose.rotation().matrix().eulerAngles(0, 1, 2);
  tcp_pose.alpha = euler_angles[0];
  tcp_pose.beta = euler_angles[1];
  tcp_pose.gamma = euler_angles[2]; 
}

Eigen::Vector3d ArmControlBase::getUnitApproachVector(const double& alpha, const double& beta, const double& gamma)
{
  tf2::Quaternion q;
  q.setRPY(alpha, beta, gamma);
  tf2::Matrix3x3 r(q);

  tf2::Vector3 approach_vector = r * tf2::Vector3(0, 0, 1);
  approach_vector = approach_vector.normalize();
  return Eigen::Vector3d(approach_vector[0], approach_vector[1], approach_vector[2]);
}

bool ArmControlBase::pick(double x, double y, double z, 
                          double alpha, double beta, double gamma, 
                          double vel, double acc, double vel_scale, double approach)
{
  Eigen::Vector3d pre_grasp_origin = Eigen::Vector3d(x, y, z) - getUnitApproachVector(alpha, beta, gamma) * approach;

  Eigen::Isometry3d grasp, orientation, pre_grasp;
  orientation = Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ());
  grasp = Eigen::Translation3d(x, y, z) * orientation;
  pre_grasp = Eigen::Translation3d(pre_grasp_origin) * orientation;

  if (// Move to pre_grasp
      moveToTcpPose(pre_grasp, vel, acc) &&
      // Open gripper
      open() &&
      // Move to grasp
      moveToTcpPose(grasp, vel*vel_scale, acc*vel_scale) &&
      // Close gripper
      close() &&
      // Move to pre_grasp
      moveToTcpPose(pre_grasp, vel*vel_scale, acc*vel_scale))
  {
    std::cout << "Pick finished." << std::endl;
    return true;
  }
  else
  {
    std::cerr << "Pick failed." << std::endl;
    return false;
  }
}

bool ArmControlBase::pick(const geometry_msgs::msg::PoseStamped& pose_stamped, 
          double vel, double acc, double vel_scale, double approach)
{
  updateTFGoal(pose_stamped);

  TcpPose tcp_pose;
  toTcpPose(pose_stamped, tcp_pose);
  return pick(tcp_pose.x, tcp_pose.y, tcp_pose.z, 
              tcp_pose.alpha, tcp_pose.beta, tcp_pose.gamma, vel, acc, vel_scale, approach);
}

bool ArmControlBase::place(double x, double y, double z, 
                           double alpha, double beta, double gamma,
                           double vel, double acc, double vel_scale, double retract)
{
  Eigen::Vector3d pre_place_origin = Eigen::Vector3d(x, y, z) - getUnitApproachVector(alpha, beta, gamma) * retract;

  Eigen::Isometry3d place, orientation, pre_place;
  orientation = Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ()); 
  place = Eigen::Translation3d(x, y, z) * orientation;
  pre_place = Eigen::Translation3d(pre_place_origin) * orientation;

  
  if (// Move to pre_place
      moveToTcpPose(pre_place, vel, acc) &&
      // Move to place
      moveToTcpPose(place, vel*vel_scale, acc*vel_scale) &&
      // Open gripper
      open() &&
      // Move to pre_grasp
      moveToTcpPose(pre_place, vel*vel_scale, acc*vel_scale))
  {
    std::cout << "Place finished." << std::endl;
    return true;
  }
  else
  {
    std::cerr << "Place failed." << std::endl;
    return false;
  }
}

bool ArmControlBase::place(const geometry_msgs::msg::PoseStamped& pose_stamped, 
          double vel, double acc, double vel_scale, double retract)
{
  updateTFGoal(pose_stamped);

  TcpPose tcp_pose;
  toTcpPose(pose_stamped, tcp_pose);
  return place(tcp_pose.x, tcp_pose.y, tcp_pose.z, 
               tcp_pose.alpha, tcp_pose.beta, tcp_pose.gamma, vel, acc, vel_scale, retract);
}

bool ArmControlBase::checkTcpGoalArrived(Eigen::Isometry3d& tcp_goal)
{
  bool wait = true;
  bool arrived = false;

  auto start = std::chrono::high_resolution_clock::now();
  while(wait)
  {
    std::unique_lock<std::mutex> lock(m_);
    Eigen::Vector3d t(tcp_pose_.x, tcp_pose_.y, tcp_pose_.z);
    if (tcp_goal.translation().isApprox(t, 0.01))
    {
      wait = false;
      arrived = true;
    }
    else
    {
      auto finish = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = finish - start;
      if (elapsed.count() > time_out_)
      {
        wait = false;
        arrived = false;
        std::cerr << "Motion timeout" << std::endl;
        printf("Tcp pose: (%f %f %f %f %f %f). \n", tcp_pose_.x, tcp_pose_.y, tcp_pose_.z, 
                                        tcp_pose_.alpha, tcp_pose_.beta, tcp_pose_.gamma);
      }
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return arrived;
}

bool ArmControlBase::checkJointValueGoalArrived(const std::vector<double>& joint_goal)
{
  bool wait = true;
  bool arrived = false;

  if (joint_goal.size() != joint_values_.size())
  {
    std::cerr << "Num of joints of goal dosen't match current joint state." << std::endl;
    wait = false;
  }

  auto start = std::chrono::high_resolution_clock::now();
  while(wait)
  {
    std::unique_lock<std::mutex> lock(m_);
    double num_joints = joint_goal.size();
    Eigen::Map<const Eigen::VectorXd> goal(joint_goal.data(), num_joints);
    Eigen::Map<const Eigen::VectorXd> current(joint_values_.data(), num_joints);
    if (current.isApprox(goal, 0.01))
    {
      wait = false;
      arrived = true;
    }
    else
    {
      auto finish = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = finish - start;
      if (elapsed.count() > time_out_)
      {
        wait = false;
        arrived = false;
        std::cerr << "Motion timeout" << std::endl;
        std::stringstream ss;
        ss << "Current joint values: ";
        for (auto value : joint_values_)
          ss << value << " ";
        std::cerr << ss.str() << std::endl;
      }
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return arrived;
}