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
 * @file ur_test.cpp 
 */

#include <robot_interface/control_ur.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<ArmControlBase> arm_control(new URControl("ur_test",  rclcpp::NodeOptions()
                                                                          .allow_undeclared_parameters(true)
                                                                          .automatically_declare_parameters_from_overrides(true)));

  arm_control->parseArgs();
  arm_control->startLoop();

  rclcpp::sleep_for(std::chrono::seconds(2));

  while(rclcpp::ok())
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "base";
    pose_stamped.header.stamp = arm_control->now();
    pose_stamped.pose.position.x = -0.068673; 
    pose_stamped.pose.position.y = -0.595636; 
    pose_stamped.pose.position.z = 0.201606;
    pose_stamped.pose.orientation.x = -0.311507;
    pose_stamped.pose.orientation.y =  0.950216;
    pose_stamped.pose.orientation.z = -0.004305;
    pose_stamped.pose.orientation.w =  0.005879;

    arm_control->moveToTcpPose(pose_stamped, 0.3, 0.3);

    pose_stamped.header.frame_id = "base";
    pose_stamped.header.stamp = arm_control->now();
    pose_stamped.pose.position.x = -0.157402; 
    pose_stamped.pose.position.y = -0.679509; 
    pose_stamped.pose.position.z = 0.094437;
    pose_stamped.pose.orientation.x = 0.190600;
    pose_stamped.pose.orientation.y = 0.948295;
    pose_stamped.pose.orientation.z = 0.239947;
    pose_stamped.pose.orientation.w = 0.082662;

    arm_control->pick(pose_stamped, 1.05, 1.4, 0.5, 0.1);
    arm_control->place(pose_stamped, 1.05, 1.4, 0.5, 0.1);

    arm_control->moveToTcpPose(-0.350, -0.296, 0.12, 3.14159, 0, 0, 0.3, 0.3);

    arm_control->moveToJointValues(std::vector<double>{0.87, -1.44, 1.68, -1.81, -1.56, 0}, 1.05, 1.4);

    arm_control->pick(-0.153, -0.433, 0.145, 2.8, -0.144, 0.0245, 1.05, 1.4, 0.5, 0.1);
    arm_control->place(-0.350, -0.296, 0.145, 3.14159, 0, 0, 1.05, 1.4, 0.5, 0.1);
  }
  rclcpp::shutdown();
  return 0;
}