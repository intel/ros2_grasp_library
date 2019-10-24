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

#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H

// ROS include
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// OpenCV include
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// System include
#include <iostream>
#include <string>

namespace tf2
{
void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::msg::TransformStamped& msg,
                       builtin_interfaces::msg::Time stamp, const std::string& frame_id,
                       const std::string& child_frame_id);
}

class PoseEstimator
{
public:
  PoseEstimator(std::shared_ptr<rclcpp::Node>& node, std::string pattern, std::string image_topic,
                std::string camera_info_topic, std::string publish_image_topic, int width, int height,
                std::string dictionary, double chessboard_square_size, double circle_grid_seperation,
                double aruco_board_marker_size, double aruco_board_marker_seperation, double charuco_board_marker_size,
                double charuco_board_square_size);

  ~PoseEstimator()
  {
  }

  void imageCB_CHESSBOARD(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void imageCB_ASYMMETRIC_CIRCLES_GRID(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void imageCB_ARUCO(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void imageCB_CHARUCO(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void caminfoCB(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void draw(cv::Mat img, std::vector<cv::Point2f> corners, cv::Mat imgpts);
  void rotationVectorToTF2Quaternion(tf2::Quaternion&, cv::Vec3d&);

private:
  // ROS variables
  std::shared_ptr<rclcpp::Node> node_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camerainfo_sub_;
  tf2_ros::TransformBroadcaster broadcaster_;

  // Native variables
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool run_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string publish_image_topic_;
  int width_;
  int height_;
  double chessboard_square_size_;
  double circle_grid_seperation_;
  double aruco_board_marker_size_;
  double aruco_board_marker_seperation_;
  double charuco_board_marker_size_;
  double charuco_board_square_size_;
  enum Patterns
  {
    NOT_EXISTING,
    CHESSBOARD,
    ASYMMETRIC_CIRCLES_GRID,
    CHARUCO,
    ARUCO
  };
  Patterns calibration_pattern_;
  std::map<std::string, Patterns> pattern_map_;
  cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_;
  std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> disctionary_map_;
  std::string path_;
};

#endif