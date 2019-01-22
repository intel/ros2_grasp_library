// Copyright (c) 2018 Intel Corporation. All Rights Reserved
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
#include <memory>
#include <string>
#include <vector>
#include "grasp_library/grasp_library_node.h"
#include "grasp_library/ros_params.h"

GraspLibraryNode::GraspLibraryNode()
: Node("GraspLibraryNode"), size_left_cloud_(0),
  has_cloud_(false), has_normals_(false), has_samples_(true), frame_("")
{
  cloud_camera_ = NULL;
  // set camera viewpoint to default origin
  std::vector<double> camera_position;
  // this->get_parameter("camera_position", camera_position);
  view_point_ << 0, 0, 0;  // todo passed from launch
  std::string cloud_topic, grasp_topic, rviz_topic, tabletop_topic;
  this->get_parameter_or("cloud_topic", cloud_topic, std::string("/camera/depth_registered/points"));
  this->get_parameter_or("grasp_topic", grasp_topic, std::string("/grasp_library/clustered_grasps"));
  this->get_parameter_or("rviz_topic", rviz_topic, std::string("/grasp_library/grasps_rviz"));
  this->get_parameter_or("tabletop_topic", tabletop_topic, std::string("/grasp_library/tabletop_points"));

  auto callback = [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
      this->cloud_callback(msg);
    };
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, callback);
  grasps_pub_ = this->create_publisher<grasp_msgs::msg::GraspConfigList>(grasp_topic, 10);
  if (!tabletop_topic.empty())
  {
    tabletop_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(tabletop_topic, 1);
  }
  if (!rviz_topic.empty())
  {
    grasps_rviz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(rviz_topic, 1);
  }

  GraspDetector::GraspDetectionParameters detection_param;
  ROSParameters::getDetectionParams(this, detection_param);
  grasp_detector_ = new GraspDetector(detection_param);

  GraspPlanner::GraspPlanningParameters planning_param;
  ROSParameters::getPlanningParams(this, planning_param);
  grasp_planner_ = new GraspPlanner(this, planning_param);

  // this->get_parameter("workspace", workspace_);
  std::initializer_list<double> workspace = {-1, 1, -1, 1, -1, 1};  // todo passed from launch
  workspace_ = workspace;
  RCLCPP_INFO(logger_, "ROS2 Grasp Library node up...");
}

void GraspLibraryNode::onInit()
{
  rclcpp::Rate rate(100);
  RCLCPP_INFO(logger_, "Waiting for point cloud to arrive ...");

  while (rclcpp::ok()) {
    if (has_cloud_) {
      // detect grasps in point cloud
      std::vector<Grasp> grasps = detectGraspPosesInTopic();
      // visualize grasps in rviz
      if (grasps_rviz_pub_)
      {
        const HandSearch::Parameters& params = grasp_detector_->getHandSearchParameters();
        grasps_rviz_pub_->publish(convertToVisualGraspMsg(grasps, params.hand_outer_diameter_, params.hand_depth_,
                                                         params.finger_width_, params.hand_height_, frame_));
      }

      // reset the system
      has_cloud_ = false;
      RCLCPP_INFO(logger_, "Waiting for point cloud to arrive ...");
    }

    rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }
}

std::vector<Grasp> GraspLibraryNode::detectGraspPosesInTopic()
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
  grasp_planner_->grasp_callback(std::make_shared<grasp_msgs::msg::GraspConfigList>(
      selected_grasps_msg));
  grasps_pub_->publish(selected_grasps_msg);
  RCLCPP_INFO(logger_, "Published %d highest-scoring grasps.", selected_grasps_msg.grasps.size());

  return grasps;
}

void GraspLibraryNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
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
      PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
      pcl::fromROSMsg(*msg, *cloud);

      // remove table plane
      if (tabletop_pub_){
	      RCLCPP_INFO(logger_, "remove table top ============");
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.015);
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        for (size_t i = 0; i < inliers->indices.size (); ++i)
        {
          cloud->points[inliers->indices[i]].x = std::numeric_limits<float>::quiet_NaN();
          cloud->points[inliers->indices[i]].y = std::numeric_limits<float>::quiet_NaN();
          cloud->points[inliers->indices[i]].z = std::numeric_limits<float>::quiet_NaN();
        }
        sensor_msgs::msg::PointCloud2 msg2;
        msg2.header = msg->header;
        pcl::toROSMsg(*cloud, msg2);
        tabletop_pub_->publish(msg2);
      }
      cloud_camera_ = new CloudCamera(cloud, 0, view_points);
      cloud_camera_header_ = msg->header;
    }
    RCLCPP_INFO(logger_, "Received cloud with %d points and normals.", cloud_camera_->getCloudProcessed()->size());

    has_cloud_ = true;
    frame_ = msg->header.frame_id;
  }
}

grasp_msgs::msg::GraspConfigList GraspLibraryNode::createGraspListMsg(
  const std::vector<Grasp> & hands)
{
  grasp_msgs::msg::GraspConfigList msg;

  for (uint32_t i = 0; i < hands.size(); i++) {
    msg.grasps.push_back(convertToGraspMsg(hands[i]));
  }

  msg.header = cloud_camera_header_;

  return msg;
}

grasp_msgs::msg::GraspConfig GraspLibraryNode::convertToGraspMsg(const Grasp & hand)
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

visualization_msgs::msg::MarkerArray GraspLibraryNode::convertToVisualGraspMsg(const std::vector<Grasp>& hands,
  double outer_diameter, double hand_depth, double finger_width, double hand_height, const std::string& frame_id)
{
  double width = outer_diameter;
  double hw = 0.5 * width;

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker left_finger, right_finger, base, approach;
  Eigen::Vector3d left_bottom, right_bottom, left_top, right_top, left_center, right_center, approach_center,
    base_center;

  for (uint32_t i = 0; i < hands.size(); i++)
  {
    left_bottom = hands[i].getGraspBottom() - (hw - 0.5*finger_width) * hands[i].getBinormal();
    right_bottom = hands[i].getGraspBottom() + (hw - 0.5*finger_width) * hands[i].getBinormal();
    left_top = left_bottom + hand_depth * hands[i].getApproach();
    right_top = right_bottom + hand_depth * hands[i].getApproach();
    left_center = left_bottom + 0.5*(left_top - left_bottom);
    right_center = right_bottom + 0.5*(right_top - right_bottom);
    base_center = left_bottom + 0.5*(right_bottom - left_bottom) - 0.01*hands[i].getApproach();
    approach_center = base_center - 0.04*hands[i].getApproach();

    base = createHandBaseMarker(left_bottom, right_bottom, hands[i].getFrame(), 0.02, hand_height, i, frame_id);
    left_finger = createFingerMarker(left_center, hands[i].getFrame(), hand_depth, finger_width, hand_height, i*3, frame_id);
    right_finger = createFingerMarker(right_center, hands[i].getFrame(), hand_depth, finger_width, hand_height, i*3+1, frame_id);
    approach = createFingerMarker(approach_center, hands[i].getFrame(), 0.08, finger_width, hand_height, i*3+2, frame_id);

    marker_array.markers.push_back(left_finger);
    marker_array.markers.push_back(right_finger);
    marker_array.markers.push_back(approach);
    marker_array.markers.push_back(base);
  }

  return marker_array;
}

visualization_msgs::msg::Marker GraspLibraryNode::createFingerMarker(const Eigen::Vector3d& center,
  const Eigen::Matrix3d& frame, double length, double width, double height, int id, const std::string& frame_id)
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
  marker.lifetime = rclcpp::Duration(10, 0);

  // use orientation of hand frame
  Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand frame (unit: meters)
  marker.scale.x = length; // forward direction
  marker.scale.y = width; // hand closing direction
  marker.scale.z = height; // hand vertical direction

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.5;

  return marker;
}

visualization_msgs::msg::Marker GraspLibraryNode::createHandBaseMarker(const Eigen::Vector3d& start,
  const Eigen::Vector3d& end, const Eigen::Matrix3d& frame, double length, double height, int id,
  const std::string& frame_id)
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
  marker.lifetime = rclcpp::Duration(10);

  // use orientation of hand frame
  Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand frame (unit: meters)
  marker.scale.x = length; // forward direction
  marker.scale.y = (end - start).norm(); // hand closing direction
  marker.scale.z = height; // hand vertical direction

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  return marker;
}

int main(int argc, char ** argv)
{
  // initialize ROS
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraspLibraryNode>();
  node->onInit();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
