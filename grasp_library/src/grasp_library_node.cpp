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


/** constants for input point cloud types */
const int GraspLibraryNode::POINT_CLOUD_2 = 0; /**< sensor_msgs/PointCloud2*/
const int GraspLibraryNode::CLOUD_INDEXED = 1; /**< cloud with indices*/
const int GraspLibraryNode::CLOUD_SAMPLES = 2; /**< cloud with (x,y,z) samples*/


GraspLibraryNode::GraspLibraryNode()
: Node("GraspLibraryNode"), size_left_cloud_(0),
  has_cloud_(false), has_normals_(false), has_samples_(true), frame_("")
{
  cloud_camera_ = NULL;
  // set camera viewpoint to default origin
  std::vector<double> camera_position;
  // this->get_parameter("camera_position", camera_position);
  view_point_ << 0, 0, 0;  // todo passed from launch
  std::string cloud_topic;
  this->get_parameter_or("cloud_topic", cloud_topic, std::string("/camera/depth/color/points"));

  GraspDetector::GraspDetectionParameters detection_param;
  ROSParameters::getDetectionParams(this, detection_param);
  grasp_detector_ = new GraspDetector(detection_param);

  auto callback = [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
      this->cloud_callback(msg);
    };
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, callback);
  grasps_pub_ = this->create_publisher<grasp_msgs::msg::GraspConfigList>(
    "/grasp_library/clustered_grasps", 10);

  GraspPlanner::GraspPlanningParameters planning_param;
  ROSParameters::getPlanningParams(this, planning_param);
  grasp_planner_ = new GraspPlanner(this, planning_param);

  // this->get_parameter("workspace", workspace_);
  std::initializer_list<double> workspace = {-1, 1, -1, 1, -1, 1};  // todo passed from launch
  workspace_ = workspace;
  std::cout << "ROS2 Grasp Library node up...\n";
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

      // reset the system
      has_cloud_ = false;
      has_samples_ = false;
      has_normals_ = false;
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
  std::cout << "Published " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.\n";

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
      std::cout << "Received cloud with " << cloud_camera_->getCloudProcessed()->size() <<
        " points and normals.\n";
    } else {
      PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
      pcl::fromROSMsg(*msg, *cloud);
      cloud_camera_ = new CloudCamera(cloud, 0, view_points);
      cloud_camera_header_ = msg->header;
      std::cout << "Received cloud with " << cloud_camera_->getCloudProcessed()->size() <<
        " points.\n";
    }

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
