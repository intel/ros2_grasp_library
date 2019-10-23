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

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/place_location.hpp>

int main(int argc, char ** argv) {
  std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  auto node = rclcpp::Node::make_shared("PlacePublisher");
  auto pub = node->create_publisher<moveit_msgs::msg::PlaceLocation>("/recognize_pick/place", 10);

  rclcpp::Clock clock(RCL_ROS_TIME);

  moveit_msgs::msg::PlaceLocation p;
  if (args.size() < 5) {
    p.place_pose.pose.position.x = -0.45;
    p.place_pose.pose.position.y = -0.30;
    p.place_pose.pose.position.z = 0.125;
  } else {
    p.place_pose.pose.position.x = atof(args[2].c_str());
    p.place_pose.pose.position.y = atof(args[3].c_str());
    p.place_pose.pose.position.z = atof(args[4].c_str());
  }
  if (args.size() < 2) {
    RCLCPP_INFO(node->get_logger(), "Place publisher specifying object name and place position.");
    RCLCPP_INFO(node->get_logger(), "Usage: place_publisher object_name [x y z]");
    RCLCPP_INFO(node->get_logger(), "Example: place_publisher sports_ball");
    RCLCPP_INFO(node->get_logger(), "Example: place_publisher sports_ball -0.45 -0.30 0.125");
    rclcpp::shutdown();
    return 0;
  } else {
    p.id = args[1];
  }

  RCLCPP_INFO(node->get_logger(), "place publisher %s [%f %f %f]",
    p.id.c_str(), p.place_pose.pose.position.x, p.place_pose.pose.position.y, p.place_pose.pose.position.z);

  while (rclcpp::ok()) {
    p.place_pose.header.stamp = clock.now();
    p.place_pose.header.frame_id = "base";
    pub->publish(p);
    rclcpp::Rate(0.5).sleep();
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
