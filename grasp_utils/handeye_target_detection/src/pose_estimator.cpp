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

#include "PoseEstimator.h"
using std::placeholders::_1;

PoseEstimator::PoseEstimator(std::shared_ptr<rclcpp::Node>& node, std::string pattern, std::string image_topic,
                             std::string camera_info_topic, std::string publish_image_topic, int width, int height,
                             std::string dictionary, double chessboard_square_size, double circle_grid_seperation,
                             double aruco_board_marker_size, double aruco_board_marker_seperation,
                             double charuco_board_marker_size, double charuco_board_square_size)
  : node_(node)
  , it_(node_)
  , broadcaster_(node_)
  , run_(false)
  , width_(width)
  , height_(height)
  , chessboard_square_size_(chessboard_square_size)
  , circle_grid_seperation_(circle_grid_seperation)
  , aruco_board_marker_size_(aruco_board_marker_size)
  , aruco_board_marker_seperation_(aruco_board_marker_seperation)
  , charuco_board_marker_size_(charuco_board_marker_size)
  , charuco_board_square_size_(charuco_board_square_size)
{
  // Parse dictionary parameter
  disctionary_map_["DICT_4X4_50"] = cv::aruco::DICT_4X4_50;
  disctionary_map_["DICT_4X4_100"] = cv::aruco::DICT_4X4_100;
  disctionary_map_["DICT_4X4_250"] = cv::aruco::DICT_4X4_250;
  disctionary_map_["DICT_4X4_1000"] = cv::aruco::DICT_4X4_1000;
  disctionary_map_["DICT_5X5_50"] = cv::aruco::DICT_5X5_50;
  disctionary_map_["DICT_5X5_100"] = cv::aruco::DICT_5X5_100;
  disctionary_map_["DICT_5X5_250"] = cv::aruco::DICT_5X5_250;
  disctionary_map_["DICT_5X5_1000"] = cv::aruco::DICT_5X5_1000;
  disctionary_map_["DICT_6X6_50"] = cv::aruco::DICT_6X6_50;
  disctionary_map_["DICT_6X6_100"] = cv::aruco::DICT_6X6_100;
  disctionary_map_["DICT_6X6_250"] = cv::aruco::DICT_6X6_250;
  disctionary_map_["DICT_6X6_1000"] = cv::aruco::DICT_6X6_1000;
  disctionary_map_["DICT_7X7_50"] = cv::aruco::DICT_7X7_50;
  disctionary_map_["DICT_7X7_100"] = cv::aruco::DICT_7X7_100;
  disctionary_map_["DICT_7X7_250"] = cv::aruco::DICT_7X7_250;
  disctionary_map_["DICT_7X7_1000"] = cv::aruco::DICT_7X7_1000;
  std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME>::iterator it = disctionary_map_.find(dictionary);
  if (it != disctionary_map_.end())
    dictionary_ = disctionary_map_[dictionary];
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid dictionary input: %s, default dictionary DICT_6X6_250 used.",
                 dictionary);
    dictionary_ = cv::aruco::DICT_6X6_250;
  }

  // Initialize subscribers and publishers
  image_pub_ = it_.advertise(publish_image_topic, 1);
  camerainfo_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, 1, std::bind(&PoseEstimator::caminfoCB, this, _1));

  calibration_pattern_ = NOT_EXISTING;
  pattern_map_ = { { "CHESSBOARD", CHESSBOARD },
                   { "ASYMMETRIC_CIRCLES_GRID", ASYMMETRIC_CIRCLES_GRID },
                   { "ARUCO", ARUCO },
                   { "CHARUCO", CHARUCO } };
  std::map<std::string, Patterns>::iterator it_pattern = pattern_map_.find(pattern);
  if (it_pattern != pattern_map_.end())
    calibration_pattern_ = pattern_map_[pattern];
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid pattern input: %s.", pattern);
    calibration_pattern_ = NOT_EXISTING;
  }

  switch (calibration_pattern_)
  {
    case CHESSBOARD:
      image_sub_ = it_.subscribe<PoseEstimator>(image_topic, 1, &PoseEstimator::imageCB_CHESSBOARD, this);
      break;
    case ASYMMETRIC_CIRCLES_GRID:
      image_sub_ = it_.subscribe(image_topic, 1, &PoseEstimator::imageCB_ASYMMETRIC_CIRCLES_GRID, this);
      break;
    case ARUCO:
      image_sub_ = it_.subscribe(image_topic, 1, &PoseEstimator::imageCB_ARUCO, this);
      break;
    case CHARUCO:
      image_sub_ = it_.subscribe(image_topic, 1, &PoseEstimator::imageCB_CHARUCO, this);
      break;
    default:
      break;
  }

  // Initialize camera intrinsic parameters
  camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
}

void PoseEstimator::imageCB_CHESSBOARD(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (run_)
  {
    if (!msg)
    {
      RCLCPP_ERROR(node_->get_logger(), "The pointer to image message is NULL.");
      return;
    }

    try
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

      // Find the chessboard pattern
      std::vector<cv::Point2f> pointBuf;  // corners
      cv::Size patternsize;
      patternsize.width = width_;
      patternsize.height = height_;
      int chessBoardFlags;
      chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
      bool found = cv::findChessboardCorners(cv_ptr->image, patternsize, pointBuf, chessBoardFlags);

      if (found)
      {
        // Correct corner points
        cv::cornerSubPix(cv_ptr->image, pointBuf, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

        // Find the parameters of transform between the calibration plate and
        // camera plane
        std::vector<int> sizeOjbPnts = { static_cast<int>(pointBuf.size()), 3 };
        cv::Mat objectPoints(sizeOjbPnts, CV_64F);
        for (int i = 0; i < sizeOjbPnts[0]; i++)
        {
          objectPoints.at<double>(i, 0) = i % patternsize.width * chessboard_square_size_;
          objectPoints.at<double>(i, 1) = i / patternsize.width * chessboard_square_size_;
          objectPoints.at<double>(i, 2) = 0;
        }
        cv::Vec3d tvect, rvect;
        bool solved = cv::solvePnPRansac(objectPoints, pointBuf, camera_matrix_, dist_coeffs_, rvect, tvect);
        if (solved)
        {
          tf2::Quaternion q;
          rotationVectorToTF2Quaternion(q, rvect);
          geometry_msgs::msg::TransformStamped transform_stamped;
          tf2::transformTF2ToMsg(tf2::Transform(q, tf2::Vector3(tvect[0], tvect[1], tvect[2])), transform_stamped,
                                 node_->now(), msg->header.frame_id, "calib_board");
          broadcaster_.sendTransform(transform_stamped);
        }

        // Project the axis points
        cv::Mat axis = cv::Mat::zeros(3, 3, CV_64F);
        axis.at<double>(0, 0) = 3 * chessboard_square_size_;
        axis.at<double>(1, 1) = 3 * chessboard_square_size_;
        axis.at<double>(2, 2) = -3 * chessboard_square_size_;
        cv::Mat imageAxisPoints;
        cv::projectPoints(axis, rvect, tvect, camera_matrix_, dist_coeffs_, imageAxisPoints);

        // Draw axis to image
        cv_bridge::CvImage cv_image_color(msg->header, sensor_msgs::image_encodings::RGB8);
        cv::cvtColor(cv_ptr->image, cv_image_color.image, cv::COLOR_GRAY2RGB);
        cv::drawChessboardCorners(cv_image_color.image, patternsize, cv::Mat(pointBuf), found);
        draw(cv_image_color.image, pointBuf, imageAxisPoints);

        // Output stream
        image_pub_.publish(cv_image_color.toImageMsg());
      }
      else
        // Output stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exeption: %s", e.what());
    }
  }
}

void PoseEstimator::imageCB_ASYMMETRIC_CIRCLES_GRID(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (run_)
  {
    if (!msg)
    {
      RCLCPP_INFO(node_->get_logger(), "The pointer to image message is NULL.");
      return;
    }
    try
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

      // Find the circlesgrid pattern
      std::vector<cv::Point2f> pointBuf;  // corners
      cv::Size patternsize;
      patternsize.width = width_;
      patternsize.height = height_;
      int chessBoardFlags;
      chessBoardFlags = cv::CALIB_CB_ASYMMETRIC_GRID;
      bool found = cv::findCirclesGrid(cv_ptr->image, patternsize, pointBuf, chessBoardFlags);

      if (found)
      {
        // Correct corner points
        std::vector<cv::Point2f> corners2;
        patternsize.height = (patternsize.height + 1) / 2;
        for (int i = 0; i < patternsize.height; i++)
        {
          for (int j = 0; j < patternsize.width; j++)
            corners2.push_back(pointBuf[i * patternsize.width * 2 + j]);
        }
        pointBuf.clear();
        for (size_t i = 0; i < corners2.size(); i++)
          pointBuf.push_back(corners2[i]);

        // Find the parameters of transform between the calibration plate and
        // camera plane
        std::vector<int> sizeOjbPnts = { static_cast<int>(pointBuf.size()), 3 };
        cv::Mat objectPoints(sizeOjbPnts, CV_64F);
        for (int i = 0; i < sizeOjbPnts[0]; i++)
        {
          objectPoints.at<double>(i, 0) = i % patternsize.width * circle_grid_seperation_;
          objectPoints.at<double>(i, 1) = i / patternsize.width * circle_grid_seperation_;
          objectPoints.at<double>(i, 2) = 0;
        }
        cv::Vec3d tvect, rvect;
        bool solved = cv::solvePnPRansac(objectPoints, pointBuf, camera_matrix_, dist_coeffs_, rvect, tvect);
        if (solved)
        {
          tf2::Quaternion q;
          rotationVectorToTF2Quaternion(q, rvect);
          geometry_msgs::msg::TransformStamped transform_stamped;
          tf2::transformTF2ToMsg(tf2::Transform(q, tf2::Vector3(tvect[0], tvect[1], tvect[2])), transform_stamped,
                                 node_->now(), msg->header.frame_id, "calib_board");
          broadcaster_.sendTransform(transform_stamped);
        }

        // Project the axis points
        cv::Mat axis = cv::Mat::zeros(3, 3, CV_64F);
        axis.at<double>(0, 0) = 3 * circle_grid_seperation_;
        axis.at<double>(1, 1) = 3 * circle_grid_seperation_;
        axis.at<double>(2, 2) = -3 * circle_grid_seperation_;
        cv::Mat imageAxisPoints;
        cv::projectPoints(axis, rvect, tvect, camera_matrix_, dist_coeffs_, imageAxisPoints);

        // Draw axis to image
        cv_bridge::CvImage cv_image_color(msg->header, sensor_msgs::image_encodings::RGB8);
        cv::cvtColor(cv_ptr->image, cv_image_color.image, cv::COLOR_GRAY2RGB);
        cv::drawChessboardCorners(cv_image_color.image, patternsize, cv::Mat(pointBuf), found);
        draw(cv_image_color.image, pointBuf, imageAxisPoints);

        // Output stream
        image_pub_.publish(cv_image_color.toImageMsg());
      }
      else
        // Output stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exeption: %s", e.what());
    }
  }
}

void PoseEstimator::imageCB_ARUCO(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (run_)
  {
    if (!msg)
    {
      RCLCPP_INFO(node_->get_logger(), "The pointer to image message is NULL.");
      return;
    }
    try
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

      // Detect aruco board
      cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_);
      cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(width_, height_, aruco_board_marker_size_,
                                                                         aruco_board_marker_seperation_, dictionary);
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);

      if (ids.size() > 0)
      {
        cv::Mat imageColor;
        cv::cvtColor(cv_ptr->image, imageColor, cv::COLOR_GRAY2RGB);
        std::vector<std::vector<cv::Point2f>> rejectedCorners;
        cv::aruco::refineDetectedMarkers(cv_ptr->image, board, corners, ids, rejectedCorners, camera_matrix_,
                                         dist_coeffs_);
        cv::aruco::drawDetectedMarkers(imageColor, corners, ids);

        // Estimate the pose of aruco board
        cv::Vec3d rvect, tvect;
        int valid = cv::aruco::estimatePoseBoard(corners, ids, board, camera_matrix_, dist_coeffs_, rvect, tvect);

        // If at least one board marker detected
        if (valid > 0)
        {
          cv::aruco::drawAxis(imageColor, camera_matrix_, dist_coeffs_, rvect, tvect, 0.1);
          tf2::Quaternion q;
          rotationVectorToTF2Quaternion(q, rvect);
          geometry_msgs::msg::TransformStamped transform_stamped;
          tf2::transformTF2ToMsg(tf2::Transform(q, tf2::Vector3(tvect[0], tvect[1], tvect[2])), transform_stamped,
                                 node_->now(), msg->header.frame_id, "calib_board");
          broadcaster_.sendTransform(transform_stamped);
        }

        // Output stream
        cv_bridge::CvImage cv_image_color(msg->header, sensor_msgs::image_encodings::RGB8, imageColor);
        image_pub_.publish(cv_image_color.toImageMsg());
      }
      else
        // Output stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exeption: %s", e.what());
    }
  }
}

void PoseEstimator::imageCB_CHARUCO(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (run_)
  {
    if (!msg)
    {
      RCLCPP_ERROR(node_->get_logger(), "The pointer to image message is NULL.");
      return;
    }
    try
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

      // Detect ChArUco
      cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_);
      cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(
          width_, height_, charuco_board_square_size_, charuco_board_marker_size_, dictionary);
      cv::Ptr<cv::aruco::DetectorParameters> params_ptr(new cv::aruco::DetectorParameters());
#if CV_MINOR_VERSION == 2
      params_ptr->doCornerRefinement = true;
#else
      params_ptr->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
#endif
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, params_ptr);
      if (ids.size() > 0)
      {
        cv::Mat imageColor;
        cv::cvtColor(cv_ptr->image, imageColor, cv::COLOR_GRAY2RGB);

        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        cv::aruco::interpolateCornersCharuco(corners, ids, cv_ptr->image, board, charucoCorners, charucoIds);
        if (charucoIds.size() > 0)
        {
          cv::aruco::drawDetectedCornersCharuco(imageColor, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
          // Estimate charuco pose
          cv::Vec3d rvect, tvect;
          bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, camera_matrix_,
                                                           dist_coeffs_, rvect, tvect);
          if (valid)
          {
            cv::aruco::drawAxis(imageColor, camera_matrix_, dist_coeffs_, rvect, tvect, 0.1);
            tf2::Quaternion q;
            rotationVectorToTF2Quaternion(q, rvect);
            geometry_msgs::msg::TransformStamped transform_stamped;
            tf2::transformTF2ToMsg(tf2::Transform(q, tf2::Vector3(tvect[0], tvect[1], tvect[2])), transform_stamped,
                                   node_->now(), msg->header.frame_id, "calib_board");
            broadcaster_.sendTransform(transform_stamped);
          }
        }

        // Output stream
        cv_bridge::CvImage cv_image_color(msg->header, sensor_msgs::image_encodings::RGB8, imageColor);
        image_pub_.publish(cv_image_color.toImageMsg());
      }
      else
        // Output stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exeption: %s", e.what());
    }
  }
}

void PoseEstimator::caminfoCB(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!run_)
  {
    if (msg->k.size() == 9 && msg->d.size() == 5)
    {
      // Store camera matrix info
      for (size_t i = 0; i < 3; i++)
        for (size_t j = 0; j < 3; j++)
          camera_matrix_.at<double>(i, j) = msg->k[i * 3 + j];

      // Store camera distortion info
      for (size_t i = 0; i < 5; i++)
        dist_coeffs_.at<double>(i, 0) = msg->d[i];

      // Set the flag to start processing the image
      run_ = true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Got invalid camera info.");
      run_ = false;
    }
  }
}

void PoseEstimator::draw(cv::Mat img, std::vector<cv::Point2f> corners, cv::Mat imgpts)
{
  cv::Point corner(corners[0]);
  cv::Point axis_point_x(imgpts.ptr<double>(0)[0], imgpts.ptr<double>(0)[1]);
  cv::Point axis_point_y(imgpts.ptr<double>(1)[0], imgpts.ptr<double>(1)[1]);
  cv::Point axis_point_z(imgpts.ptr<double>(2)[0], imgpts.ptr<double>(2)[1]);
  cv::line(img, corner, axis_point_x, cv::Scalar(255, 0, 0), 6);
  cv::line(img, corner, axis_point_y, cv::Scalar(0, 255, 0), 6);
  cv::line(img, corner, axis_point_z, cv::Scalar(0, 0, 255), 6);
}

void PoseEstimator::rotationVectorToTF2Quaternion(tf2::Quaternion& q, cv::Vec3d& rvect)
{
  q.setRPY(0.0, 0.0, 0.0);
  cv::Mat rm;
  cv::Rodrigues(rvect, rm);
  tf2::Matrix3x3 m(rm.ptr<double>(0)[0], rm.ptr<double>(0)[1], rm.ptr<double>(0)[2], rm.ptr<double>(1)[0],
                   rm.ptr<double>(1)[1], rm.ptr<double>(1)[2], rm.ptr<double>(2)[0], rm.ptr<double>(2)[1],
                   rm.ptr<double>(2)[2]);
  m.getRotation(q);
}
