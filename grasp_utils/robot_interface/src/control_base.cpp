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

#include <robot_interface/control_base.hpp>
#include <chrono>
#include <thread>

bool ArmControlBase::moveToTcpPose(const Eigen::Isometry3d& pose, double vel, double acc)
{
  TcpPose tcp_pose;
  toTcpPose(pose, tcp_pose);
  return this->moveToTcpPose(tcp_pose.x, tcp_pose.y, tcp_pose.z, 
                             tcp_pose.alpha, tcp_pose.beta, tcp_pose.gamma, vel, acc);
}

void ArmControlBase::toTcpPose(const geometry_msgs::msg::PoseStamped& pose_stamped, TcpPose& tcp_pose)
{
  Eigen::Isometry3d pose_transform;
  tf2::fromMsg(pose_stamped.pose, pose_transform);
  toTcpPose(pose_transform, tcp_pose);
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

bool ArmControlBase::pick(double x, double y, double z, 
                          double alpha, double beta, double gamma, 
                          double vel, double acc, double vel_scale, double approach)
{
  Eigen::Isometry3d grasp;
  grasp = Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ());
  grasp = Eigen::Translation3d(x, y, z) * grasp;

  Eigen::Isometry3d pre_grasp;
  pre_grasp = grasp * Eigen::Translation3d(0, 0, -approach);

  
  if (// Move to pre_grasp
      moveToTcpPose(pre_grasp, vel, acc) && checkTcpGoalArrived(pre_grasp) &&
      // Open gripper
      open() &&
      // Move to grasp
      moveToTcpPose(grasp, vel*vel_scale, acc*vel_scale) && checkTcpGoalArrived(grasp) &&
      // Close gripper
      close() &&
      // Move to pre_grasp
      moveToTcpPose(pre_grasp, vel*vel_scale, acc*vel_scale) && checkTcpGoalArrived(pre_grasp))
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
  TcpPose tcp_pose;
  toTcpPose(pose_stamped, tcp_pose);
  return pick(tcp_pose.x, tcp_pose.y, tcp_pose.z, 
              tcp_pose.alpha, tcp_pose.beta, tcp_pose.gamma, vel, acc, vel_scale, approach);
}

bool ArmControlBase::place(double x, double y, double z, 
                           double alpha, double beta, double gamma,
                           double vel, double acc, double vel_scale, double retract)
{
  Eigen::Isometry3d place;
  place = Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX())
              * Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY())
              * Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ()); 
  place = Eigen::Translation3d(x, y, z) * place;

  Eigen::Isometry3d pre_place;
  pre_place = place * Eigen::Translation3d(0, 0, -retract);

  
  if (// Move to pre_place
      moveToTcpPose(pre_place, vel, acc) && checkTcpGoalArrived(pre_place) &&
      // Move to place
      moveToTcpPose(place, vel*vel_scale, acc*vel_scale) && checkTcpGoalArrived(place) &&
      // Open gripper
      open() &&
      // Move to pre_grasp
      moveToTcpPose(pre_place, vel*vel_scale, acc*vel_scale) && checkTcpGoalArrived(pre_place))
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
  TcpPose tcp_pose;
  toTcpPose(pose_stamped, tcp_pose);
  return place(tcp_pose.x, tcp_pose.y, tcp_pose.z, 
               tcp_pose.alpha, tcp_pose.beta, tcp_pose.gamma, vel, acc, vel_scale, retract);
}

bool ArmControlBase::checkTcpGoalArrived(Eigen::Isometry3d& tcp_goal)
{
  bool wait = true;
  bool arrived = false;

  std::vector<std::vector<double>> record;

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