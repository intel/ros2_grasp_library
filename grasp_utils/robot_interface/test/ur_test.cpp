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

  arm_control->moveToTcpPose(-0.350, -0.296, 0.12, 3.14159, 0, 0, 0.3, 0.3);

  while(rclcpp::ok())
  {
    arm_control->pick(-0.153, -0.433, 0.145, 2.8, -0.144, 0.0245, 1.05, 1.4, 0.5, 0.1);

    arm_control->place(-0.350, -0.296, 0.145, 3.14159, 0, 0, 1.05, 1.4, 0.5, 0.1);
  }
  rclcpp::shutdown();
  return 0;
}