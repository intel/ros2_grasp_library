/** Copyright (c) 2019 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "PoseEstimator.h"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  // Start the ros node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "pose_estimation",
      rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

  // Initialize parameter client
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  while (!parameters_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  // Get parameters
  std::string pattern = parameters_client->get_parameter<std::string>("pattern", "ARUCO");
  std::string image_topic = parameters_client->get_parameter<std::string>("image_topic", "/camera/color/image_raw");
  std::string camera_info_topic =
      parameters_client->get_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
  std::string publish_image_topic =
      parameters_client->get_parameter<std::string>("publish_image_topic", "/image/detected");
  int width = parameters_client->get_parameter("width", 5);
  int height = parameters_client->get_parameter("height", 7);
  std::string dictionary = parameters_client->get_parameter<std::string>("dictionary", "DICT_4X4_50");
  double chessboard_square_size = parameters_client->get_parameter("chessboard_square_size", 0.026);
  double circle_grid_seperation = parameters_client->get_parameter("circle_grid_seperation", 0.035);
  double aruco_board_marker_size = parameters_client->get_parameter("aruco_board_marker_size", 0.035);
  double aruco_board_marker_seperation = parameters_client->get_parameter("aruco_board_marker_seperation", 0.007);
  double charuco_board_marker_size = parameters_client->get_parameter("charuco_board_marker_size", 0.022);
  double charuco_board_square_size = parameters_client->get_parameter("charuco_board_square_size", 0.037);

  PoseEstimator pe(node, pattern, image_topic, camera_info_topic, publish_image_topic, width, height, dictionary,
                   chessboard_square_size, circle_grid_seperation, aruco_board_marker_size,
                   aruco_board_marker_seperation, charuco_board_marker_size, charuco_board_square_size);

  rclcpp::spin(node);
  RCLCPP_INFO(node->get_logger(), "Node calibration_pattern_pose_estimation exited.");
  return 0;
}