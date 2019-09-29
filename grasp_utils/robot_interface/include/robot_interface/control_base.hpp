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

#pragma once

#include <mutex>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

struct TcpPose
{
  double x, y, z;
  double alpha, beta, gamma;
};

class ArmControlBase: public rclcpp::Node
{
public:

  ArmControlBase(const std::string node_name, const rclcpp::NodeOptions & options)
  : Node(node_name, options)
  {
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    time_out_ = 15.0;
  }

  ~ArmControlBase()
  {
  }

  virtual bool moveToTcpPose(double x, double y, double z, 
                             double alpha, double beta, double gamma, 
                             double vel, double acc) = 0;

  virtual bool moveToTcpPose(const Eigen::Isometry3d& pose, double vel, double acc);

  virtual bool open(const double distance = 0) = 0;

  virtual bool close(const double distance = 0) = 0;

  virtual bool pick(double x, double y, double z, 
                    double alpha, double beta, double gamma, 
                    double vel, double acc, double vel_scale, double approach);
  
  virtual bool pick(const geometry_msgs::msg::PoseStamped& pose_stamped, 
                    double vel, double acc, double vel_scale, double approach);

  virtual bool place(double x, double y, double z, 
                     double alpha, double beta, double gamma,
                     double vel, double acc, double vel_scale, double retract);

  virtual bool place(const geometry_msgs::msg::PoseStamped& pose_stamped,
                     double vel, double acc, double vel_scale, double retract);

  void toTcpPose(const geometry_msgs::msg::PoseStamped& pose_stamped, TcpPose& tcp_pose);

  void toTcpPose(const Eigen::Isometry3d& pose, TcpPose& tcp_pose);

  virtual bool checkTcpGoalArrived(Eigen::Isometry3d& tcp_goal);

protected:
  // Joint state publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  // Joint names
  std::vector<std::string> joint_names_;
  // Current tcp pose
  TcpPose tcp_pose_;
  // Mutex to guard the tcp_pose usage
  std::mutex m_;
  // Motion running duration timeout
  double time_out_;
};