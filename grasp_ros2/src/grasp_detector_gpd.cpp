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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "grasp_library/ros2/grasp_detector_gpd.hpp"
#include "grasp_library/ros2/ros_params.hpp"

namespace grasp_ros2
{

GraspDetectorGPD::GraspDetectorGPD(const rclcpp::NodeOptions & options)
: Node("GraspDetectorGPD", options),
  GraspDetectorBase(), cloud_camera_(NULL), has_cloud_(false), frame_(""),
  object_msg_(nullptr), object_sub_(nullptr), filtered_pub_(nullptr), grasps_rviz_pub_(nullptr)
{
  std::vector<double> camera_position;
  this->get_parameter_or("camera_position", camera_position,
    std::vector<double>(std::initializer_list<double>({0, 0, 0})));
  view_point_ << camera_position[0], camera_position[1], camera_position[2];
  this->get_parameter_or("auto_mode", auto_mode_, true);
  std::string cloud_topic, grasp_topic, rviz_topic, tabletop_topic, object_topic;
  this->get_parameter_or("cloud_topic", cloud_topic,
    std::string(Consts::kTopicPointCloud2));
  bool rviz, object_detect;
  this->get_parameter_or("rviz", rviz, false);
  this->get_parameter_or("plane_remove", plane_remove_, false);
  this->get_parameter_or("object_detect", object_detect, false);

  callback_group_subscriber1_ = this->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  auto sub1_opt = rclcpp::SubscriptionOptions();
  sub1_opt.callback_group = callback_group_subscriber1_;

  auto callback = [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
      this->cloud_callback(msg);
    };
  cloud_sub_ =
    this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic,
      rclcpp::QoS(10), callback, sub1_opt);

  grasps_pub_ = this->create_publisher<grasp_msgs::msg::GraspConfigList>(
    Consts::kTopicDetectedGrasps, 10);
  if (rviz) {
    grasps_rviz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      Consts::kTopicVisualGrasps, 10);
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      Consts::kTopicTabletop, 10);
  }
  if (object_detect) {
    callback_group_subscriber2_ = this->create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    auto sub2_opt = rclcpp::SubscriptionOptions();
    sub2_opt.callback_group = callback_group_subscriber2_;

    this->get_parameter_or("object_topic", object_topic,
      std::string(Consts::kTopicDetectedObjects));
    auto callback = [this](const people_msgs::msg::ObjectsInMasks::SharedPtr msg) -> void {
        this->object_callback(msg);
      };
    object_sub_ =
      this->create_subscription<people_msgs::msg::ObjectsInMasks>(object_topic,
        rclcpp::QoS(10), callback, sub2_opt);
  }

  GraspDetector::GraspDetectionParameters detection_param;
  ROSParameters::getDetectionParams(this, detection_param);
  grasp_detector_ = std::make_shared<GraspDetector>(detection_param);
  RCLCPP_INFO(logger_, "ROS2 Grasp Library node up...");

  detector_thread_ = new std::thread(&GraspDetectorGPD::onInit, this);
  detector_thread_->detach();
}

void GraspDetectorGPD::onInit()
{
  rclcpp::Rate rate(100);
  RCLCPP_INFO(logger_, "Waiting for point cloud to arrive ...");

  while (rclcpp::ok()) {
    if (has_cloud_) {
      // detect grasps in point cloud
      std::vector<Grasp> grasps = detectGraspPosesInTopic();
      // visualize grasps in rviz
      if (grasps_rviz_pub_) {
        const HandSearch::Parameters & params = grasp_detector_->getHandSearchParameters();
        grasps_rviz_pub_->publish(convertToVisualGraspMsg(grasps, params.hand_outer_diameter_,
          params.hand_depth_,
          params.finger_width_, params.hand_height_, frame_));
      }

      // reset the system
      has_cloud_ = false;
      RCLCPP_INFO(logger_, "Waiting for point cloud to arrive ...");
    }

    // rclcpp::spin(shared_from_this());
    rate.sleep();
  }
}

std::vector<Grasp> GraspDetectorGPD::detectGraspPosesInTopic()
{
  // detect grasp poses
  std::vector<Grasp> grasps;

  {
    // preprocess the point cloud
    grasp_detector_->preprocessPointCloud(*cloud_camera_);
    // detect grasps in the point cloud
    grasps = grasp_detector_->detectGrasps(*cloud_camera_);
  }

  // Publish the selected grasps.
  grasp_msgs::msg::GraspConfigList selected_grasps_msg = createGraspListMsg(grasps);
  if (grasp_cb_) {
    grasp_cb_->grasp_callback(
      std::make_shared<grasp_msgs::msg::GraspConfigList>(selected_grasps_msg));
  }
  grasps_pub_->publish(selected_grasps_msg);
  RCLCPP_INFO(logger_, "Published %d highest-scoring grasps.", selected_grasps_msg.grasps.size());

  return grasps;
}

void GraspDetectorGPD::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  people_msgs::msg::ObjectsInMasks::SharedPtr object_msg;

  if (!auto_mode_ && !started_) {return;}

  if (object_sub_) {
    if (object_name_.empty()) {
      RCLCPP_INFO(logger_, "Waiting for object name...");
      return;
    }
    object_msg = object_msg_;
    object_msg_ = nullptr;
    if (nullptr == object_msg || object_msg->objects_vector.empty()) {
      RCLCPP_INFO(logger_, "Waiting for object callback...");
      return;
    }
  }

  RCLCPP_DEBUG(logger_, "PCD callback...");
  if (!has_cloud_) {
    delete cloud_camera_;
    cloud_camera_ = NULL;
    Eigen::Matrix3Xd view_points(3, 1);
    view_points.col(0) = view_point_;

    if (msg->fields.size() == 6 && msg->fields[3].name == "normal_x" &&
      msg->fields[4].name == "normal_y" &&
      msg->fields[5].name == "normal_z")
    {
      PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
      pcl::fromROSMsg(*msg, *cloud);
      cloud_camera_ = new CloudCamera(cloud, 0, view_points);
      cloud_camera_header_ = msg->header;
    } else {
      PointCloudRGBA::Ptr cloud(new PointCloudRGBA), filter1(new PointCloudRGBA), filter2(
        new PointCloudRGBA);
      // remove table plane
      if (plane_remove_ || object_sub_) {
        pcl::fromROSMsg(*msg, *filter1);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.015);
        seg.setInputCloud(filter1);
        seg.segment(*inliers, *coefficients);
        for (size_t i = 0; i < inliers->indices.size(); ++i) {
          filter1->points[inliers->indices[i]].x = std::numeric_limits<float>::quiet_NaN();
          filter1->points[inliers->indices[i]].y = std::numeric_limits<float>::quiet_NaN();
          filter1->points[inliers->indices[i]].z = std::numeric_limits<float>::quiet_NaN();
        }
      }
      if (plane_remove_) {
        cloud = filter1;
      } else {
        pcl::fromROSMsg(*msg, *cloud);
      }

      // filter object location
      if (object_sub_) {
        bool found = false;
        for (auto obj : object_msg->objects_vector) {
          if (0 == obj.object_name.compare(object_name_)) {
            RCLCPP_INFO(logger_, "obj name %s prob %f roi [%d %d %d %d] %d %d",
              obj.object_name.c_str(), obj.probability, obj.roi.x_offset, obj.roi.y_offset,
              obj.roi.width, obj.roi.height, msg->width, msg->height);
            std::vector<int> indices;
            for (size_t i = 0; i < obj.roi.height; i++) {  // rows
              int idx = (i + obj.roi.y_offset) * msg->width + obj.roi.x_offset;
              for (size_t j = 0; j < obj.roi.width; j++) {  // columns
                // todo use mask_array from from object msg
                if (!isnan(filter1->points[idx + j].x) &&
                  !isnan(filter1->points[idx + j].y) &&
                  !isnan(filter1->points[idx + j].z))
                {
                  indices.push_back(idx + j);
                }
              }
            }
            pcl::ExtractIndices<pcl::PointXYZRGBA> filter;
            filter.setInputCloud(filter1);
            filter.setIndices(boost::make_shared<std::vector<int>>(indices));
            filter.filter(*filter2);
            Eigen::Matrix3Xf xyz =
              filter2->getMatrixXfMap(3, sizeof(pcl::PointXYZRGBA) / sizeof(float), 0);
            RCLCPP_INFO(logger_, "*************** %f %f, %f %f, %f %f",
              xyz.row(0).minCoeff(), xyz.row(0).maxCoeff(),
              xyz.row(1).minCoeff(), xyz.row(1).maxCoeff(),
              xyz.row(2).minCoeff(), xyz.row(2).maxCoeff());
            grasp_ws_ = {xyz.row(0).minCoeff(), xyz.row(0).maxCoeff(),
              xyz.row(1).minCoeff(), xyz.row(1).maxCoeff(),
              xyz.row(2).minCoeff(), xyz.row(2).maxCoeff()};
            found = true;
            cloud = filter2;
            break;
          }
        }
        if (!found) {return;}
      }
      if (filtered_pub_) {
        sensor_msgs::msg::PointCloud2 msg2;
        pcl::toROSMsg(*cloud, msg2);
        // workaround rviz rgba
        msg2.fields[3].name = "rgb";
        msg2.fields[3].datatype = 7;
        filtered_pub_->publish(msg2);
      }
      cloud_camera_ = new CloudCamera(cloud, 0, view_points);
      cloud_camera_header_ = msg->header;
    }
    RCLCPP_INFO(logger_, "Received cloud with %d points and normals.",
      cloud_camera_->getCloudProcessed()->size());

    has_cloud_ = true;
    frame_ = msg->header.frame_id;
  }
}

void GraspDetectorGPD::object_callback(const people_msgs::msg::ObjectsInMasks::SharedPtr msg)
{
  RCLCPP_INFO(logger_, "Object callback *************************[%d]", msg->objects_vector.size());
  for (auto obj : msg->objects_vector) {
    RCLCPP_INFO(logger_, "obj name %s prob %f roi[%d %d %d %d]",
      obj.object_name.c_str(), obj.probability,
      obj.roi.x_offset, obj.roi.y_offset, obj.roi.width, obj.roi.height);
    if (0 == obj.object_name.compare("orange")) {
      for (size_t i = 0; i < obj.roi.height; i++) {       // rows
        // std::cout << "\n";
        for (size_t j = 0; j < obj.roi.width; j++) {       // columns
          // int a = obj.mask_array[i * obj.roi.width + j] * 10;
          // if (a>5) std::cout << a; else std::cout << "*";
        }
      }
    }
  }
  if (msg->objects_vector.size() > 0) {
    object_msg_ = msg;
  }
}

grasp_msgs::msg::GraspConfigList GraspDetectorGPD::createGraspListMsg(
  const std::vector<Grasp> & hands)
{
  grasp_msgs::msg::GraspConfigList msg;

  for (uint32_t i = 0; i < hands.size(); i++) {
    msg.grasps.push_back(convertToGraspMsg(hands[i]));
  }

  msg.header = cloud_camera_header_;
  msg.object_name = object_name_;

  return msg;
}

grasp_msgs::msg::GraspConfig GraspDetectorGPD::convertToGraspMsg(const Grasp & hand)
{
  grasp_msgs::msg::GraspConfig msg;
  pointEigenToMsg(hand.getGraspBottom(), msg.bottom);
  pointEigenToMsg(hand.getGraspTop(), msg.top);
  pointEigenToMsg(hand.getGraspSurface(), msg.surface);
  vectorEigenToMsg(hand.getApproach(), msg.approach);
  vectorEigenToMsg(hand.getBinormal(), msg.binormal);
  vectorEigenToMsg(hand.getAxis(), msg.axis);
  msg.width.data = hand.getGraspWidth();
  msg.score.data = hand.getScore();
  pointEigenToMsg(hand.getSample(), msg.sample);

  return msg;
}

visualization_msgs::msg::MarkerArray GraspDetectorGPD::convertToVisualGraspMsg(
  const std::vector<Grasp> & hands,
  double outer_diameter, double hand_depth, double finger_width, double hand_height,
  const std::string & frame_id)
{
  double width = outer_diameter;
  double hw = 0.5 * width;

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker left_finger, right_finger, base, approach;
  Eigen::Vector3d left_bottom, right_bottom, left_top, right_top, left_center, right_center,
    approach_center,
    base_center;

  for (uint32_t i = 0; i < hands.size(); i++) {
    left_bottom = hands[i].getGraspBottom() - (hw - 0.5 * finger_width) * hands[i].getBinormal();
    right_bottom = hands[i].getGraspBottom() + (hw - 0.5 * finger_width) * hands[i].getBinormal();
    left_top = left_bottom + hand_depth * hands[i].getApproach();
    right_top = right_bottom + hand_depth * hands[i].getApproach();
    left_center = left_bottom + 0.5 * (left_top - left_bottom);
    right_center = right_bottom + 0.5 * (right_top - right_bottom);
    base_center = left_bottom + 0.5 * (right_bottom - left_bottom) - 0.01 * hands[i].getApproach();
    approach_center = base_center - 0.04 * hands[i].getApproach();

    base = createHandBaseMarker(left_bottom, right_bottom,
        hands[i].getFrame(), 0.02, hand_height, i, frame_id);
    left_finger = createFingerMarker(left_center,
        hands[i].getFrame(), hand_depth, finger_width, hand_height, i * 3, frame_id);
    right_finger = createFingerMarker(right_center,
        hands[i].getFrame(), hand_depth, finger_width, hand_height, i * 3 + 1, frame_id);
    approach = createFingerMarker(approach_center,
        hands[i].getFrame(), 0.08, finger_width, hand_height, i * 3 + 2, frame_id);

    marker_array.markers.push_back(left_finger);
    marker_array.markers.push_back(right_finger);
    marker_array.markers.push_back(approach);
    marker_array.markers.push_back(base);
  }

  return marker_array;
}

visualization_msgs::msg::Marker GraspDetectorGPD::createFingerMarker(
  const Eigen::Vector3d & center,
  const Eigen::Matrix3d & frame, double length, double width, double height, int id,
  const std::string & frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  marker.ns = "finger";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = rclcpp::Duration(20.0, 0);

  // use orientation of hand frame
  Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand frame (unit: meters)
  marker.scale.x = length;  // forward direction
  marker.scale.y = width;  // hand closing direction
  marker.scale.z = height;  // hand vertical direction

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.5;

  return marker;
}

visualization_msgs::msg::Marker GraspDetectorGPD::createHandBaseMarker(
  const Eigen::Vector3d & start,
  const Eigen::Vector3d & end, const Eigen::Matrix3d & frame, double length, double height, int id,
  const std::string & frame_id)
{
  Eigen::Vector3d center = start + 0.5 * (end - start);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  marker.ns = "hand_base";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = rclcpp::Duration(20.0, 0);

  // use orientation of hand frame
  Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand frame (unit: meters)
  marker.scale.x = length;  // forward direction
  marker.scale.y = (end - start).norm();  // hand closing direction
  marker.scale.z = height;  // hand vertical direction

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  return marker;
}

}  // namespace grasp_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(grasp_ros2::GraspDetectorGPD)
