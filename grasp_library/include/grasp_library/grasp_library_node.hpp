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

#ifndef GRASP_LIBRARY__GRASP_LIBRARY_NODE_HPP_
#define GRASP_LIBRARY__GRASP_LIBRARY_NODE_HPP_

// ROS2
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// PCL
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// eigen
#include <Eigen/Geometry>

// GPG
#include <gpg/cloud_camera.h>

// this project (messages)
#include <gpd/grasp_detector.h>
#include <grasp_msgs/msg/grasp_config.hpp>
#include <grasp_msgs/msg/grasp_config_list.hpp>
#include <grasp_msgs/msg/samples_msg.hpp>

// system
#include <algorithm>
#include <string>
#include <vector>

#include "grasp_library/grasp_planner.hpp"

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;


/** GraspLibraryNode class
 *
 * \brief A ROS node that can detect grasp poses in a point cloud.
 *
 * This class is a ROS node that handles all the ROS topics.
 *
*/
class GraspLibraryNode : public rclcpp::Node
{
public:
  /**
   * \brief Constructor.
  */
  GraspLibraryNode();

  /**
   * \brief Destructor.
  */
  ~GraspLibraryNode()
  {
    delete cloud_camera_;

    delete grasp_detector_;

    delete grasp_planner_;
  }

  /**
   * \brief Run the ROS node. Loops while waiting for incoming ROS messages.
   */
  virtual void onInit();

  /**
   * \brief Detect grasp poses in a point cloud received from a ROS topic.
   * \return the list of grasp poses
   */
  std::vector<Grasp> detectGraspPosesInTopic();

private:
  /**
   * \brief Callback function for the ROS topic that contains the input point cloud.
   * \param msg the incoming ROS message
   */
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * \brief Create a ROS message that contains a list of grasp poses from a list of handles.
   * \param hands the list of grasps
   * \return the ROS message that contains the grasp poses
   */
  grasp_msgs::msg::GraspConfigList createGraspListMsg(const std::vector<Grasp> & hands);

  /**
   * \brief Convert GPD Grasp into grasp message.
   * \param hand A GPD grasp
   * \return The Grasp message converted
   */
  grasp_msgs::msg::GraspConfig convertToGraspMsg(const Grasp & hand);

  /**
   * \brief Convert GPD Grasps into visual grasp messages.
   */
  visualization_msgs::msg::MarkerArray convertToVisualGraspMsg(
    const std::vector<Grasp> & hands,
    double outer_diameter, double hand_depth, double finger_width, double hand_height,
    const std::string & frame_id);

  /**
   * \brief Create finger marker for visual grasp messages
   */
  visualization_msgs::msg::Marker createFingerMarker(
    const Eigen::Vector3d & center,
    const Eigen::Matrix3d & frame, double length, double width, double height, int id,
    const std::string & frame_id);

  /**
   * \brief Create hand base marker for visual grasp messages
   */
  visualization_msgs::msg::Marker createHandBaseMarker(
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & end, const Eigen::Matrix3d & frame, double length, double height,
    int id,
    const std::string & frame_id);

  /** Converts an Eigen Vector into a Point message. Todo ROS2 eigen_conversions*/
  void pointEigenToMsg(const Eigen::Vector3d & e, geometry_msgs::msg::Point & m)
  {
    m.x = e(0);
    m.y = e(1);
    m.z = e(2);
  }

  /** Converts an Eigen Vector into a Vector message. Todo ROS2 eigen_conversions*/
  void vectorEigenToMsg(const Eigen::Vector3d & e, geometry_msgs::msg::Vector3 & m)
  {
    m.x = e(0);
    m.y = e(1);
    m.z = e(2);
  }

  Eigen::Vector3d view_point_; /**< (input) view point of the camera onto the point cloud*/

  /** stores point cloud with (optional) camera information and surface normals*/
  CloudCamera * cloud_camera_;
  std_msgs::msg::Header cloud_camera_header_; /**< stores header of the point cloud*/
  /** (input) size of the left point cloud (when using two point clouds as input)*/
  int size_left_cloud_;
  /** status variables for received (input) messages*/
  bool has_cloud_, has_normals_, has_samples_;
  std::string frame_; /**< point cloud frame*/
  /** ROS subscriber for point cloud messages*/
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  /** ROS publisher for grasp list messages*/
  rclcpp::Publisher<grasp_msgs::msg::GraspConfigList>::SharedPtr grasps_pub_;
  /** ROS publisher for tabletop point clouds*/
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tabletop_pub_;
  /** ROS publisher for grasps in rviz (visualization)*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grasps_rviz_pub_;

  bool filter_grasps_; /**< if grasps are filtered on workspace and gripper aperture*/
  bool filter_half_antipodal_; /**< if half-antipodal grasps are filtered*/
  bool plot_filtered_grasps_; /**< if filtered grasps are plotted*/
  bool plot_selected_grasps_; /**< if selected grasps are plotted*/
  bool plot_normals_; /**< if normals are plotted*/
  bool plot_samples_; /**< if samples/indices are plotted*/
  std::vector<double> workspace_; /**< workspace limits*/

  GraspDetector * grasp_detector_; /**< used to run the grasp pose detection*/
  GraspPlanner * grasp_planner_; /**< grasp planning service*/
  rclcpp::Logger logger_ = rclcpp::get_logger("GraspLibraryNode");
};

#endif  // GRASP_LIBRARY__GRASP_LIBRARY_NODE_HPP_
