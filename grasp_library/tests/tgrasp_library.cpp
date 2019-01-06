// Copyright (c) 2018 Intel Corporation
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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <grasp_msgs/msg/grasp_config_list.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/grasp_planning.hpp>

#include <cassert>
#include <memory>
#include <string>

static bool received_topic = false;
static int num_grasps = 0;
static rclcpp::Rate rate(0.5);

static void topic_cb(const grasp_msgs::msg::GraspConfigList::SharedPtr msg)
{
  received_topic = true;
  num_grasps = msg->grasps.size();
}

TEST(GraspLibraryTests, TestGraspTopic) {
  received_topic = false;
  num_grasps = 0;
  auto node = rclcpp::Node::make_shared("GraspLibraryTests");
  auto sub = node->create_subscription<grasp_msgs::msg::GraspConfigList>(
    "/grasp_library/clustered_grasps", topic_cb);

  rate.sleep();
  rclcpp::spin_some(node);
  EXPECT_TRUE(received_topic);
  EXPECT_GT(num_grasps, 0);
}

TEST(GraspLibraryTests, TestGraspService) {
  auto node = rclcpp::Node::make_shared("GraspLibraryTests");
  auto client = node->create_client<moveit_msgs::srv::GraspPlanning>("plan_grasps");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
      exit(1);
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  }
  auto request = std::make_shared<moveit_msgs::srv::GraspPlanning::Request>();
  auto response = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, response) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed");
    exit(1);
  }

  auto result = response.get();
  EXPECT_TRUE(result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  EXPECT_GT(result->grasps.size(), uint32_t(0));
}

int main(int argc, char ** argv) try
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
} catch (...) {
}
