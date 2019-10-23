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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <grasp_msgs/msg/grasp_config_list.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/grasp_planning.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <cassert>
#include <memory>
#include <string>
#include <thread>

#include "grasp_library/ros2/consts.hpp"
#include "./tgrasp_ros2.h"

using Consts = grasp_ros2::Consts;
using GraspPlanning = moveit_msgs::srv::GraspPlanning;

static bool received_topic = false;
static int num_grasps = 0;
static bool pcd_stop = false;
static rclcpp::Node::SharedPtr node = nullptr;
static std::shared_ptr<GraspPlanning::Response> result = nullptr;

static void pcd_publisher()
{
  char path[512];
  snprintf(path, sizeof(path), "%s/table_top.pcd", RESOURCE_DIR);
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  if (0 != pcl::io::loadPCDFile<pcl::PointXYZRGBA>(path, cloud)) {
    return;
  }
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "camera_color_optical_frame";
  auto pcd_node = rclcpp::Node::make_shared("PCDPublisher");
  auto pcd_pub = pcd_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    Consts::kTopicPointCloud2, 10);
  rclcpp::Rate loop_rate(30);
  while (!pcd_stop && rclcpp::ok()) {
    pcd_pub->publish(msg);
    loop_rate.sleep();
  }
}

static void topic_cb(const grasp_msgs::msg::GraspConfigList::SharedPtr msg)
{
  RCLCPP_INFO(node->get_logger(), "Topic received");
  received_topic = true;
  num_grasps = msg->grasps.size();
}

TEST(GraspLibraryTests, TestGraspService) {
  EXPECT_TRUE(result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  EXPECT_GT(result->grasps.size(), uint32_t(0));
}

TEST(GraspLibraryTests, TestGraspTopic) {
  rclcpp::Rate(1).sleep();
  EXPECT_TRUE(received_topic);
  EXPECT_GT(num_grasps, 0);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::thread pcd_thread(pcd_publisher);
  pcd_thread.detach();

  node = rclcpp::Node::make_shared("GraspLibraryTest");
  auto sub = node->create_subscription<grasp_msgs::msg::GraspConfigList>(
    Consts::kTopicDetectedGrasps, rclcpp::QoS(rclcpp::KeepLast(1)), topic_cb);

  auto client = node->create_client<GraspPlanning>("plan_grasps");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Client interrupted");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Wait for service");
  }
  auto request = std::make_shared<GraspPlanning::Request>();
  auto result_future = client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Request sent");

  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Request failed");
    return 1;
  }
  result = result_future.get();
  RCLCPP_INFO(node->get_logger(), "Response received %d", result->error_code.val);

  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();

  pcd_stop = true;
  rclcpp::Rate(3).sleep();
  // pcd_thread.join() disabled. It causes runtest exit abnormally

  node = nullptr;
  rclcpp::shutdown();
  return ret;
}
