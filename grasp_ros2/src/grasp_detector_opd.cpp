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
#if 1 //fpfh
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#endif
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/PolygonMesh.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#if 0 //ppf
#include <pcl/features/ppf.h>
#include <pcl/registration/ppf_registration.h>
#endif
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/hdf.hpp>
#include <opencv2/opencv.hpp>

#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>
#include "grasp_library/ros2/grasp_detector_opd.hpp"
#include "grasp_library/ros2/ros_params.hpp"


namespace grasp_ros2
{

GraspDetectorOPD::GraspDetectorOPD(const rclcpp::NodeOptions & options)
: Node("GraspDetectorOPD", options),
  GraspDetectorBase(), cloud_camera_(NULL), has_cloud_(false), frame_(""),
#ifdef RECOGNIZE_PICK
  object_msg_(nullptr), object_sub_(nullptr),
#endif
  filtered_pub_(nullptr), grasps_rviz_pub_(nullptr),
  err_tran_sum_(0.0), err_tran_mean_(0.0), opd_count_(0), tfb_(this)
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
    reg_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/grasp_library/reg_points", 10);
  }
#ifdef RECOGNIZE_PICK
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
#endif
  // GraspDetector::GraspDetectionParameters detection_param;
  ROSParameters::getDetectionParams(this, detection_param_);
  grasp_detector_ = std::make_shared<GraspDetector>(detection_param_);
  RCLCPP_INFO(logger_, "ROS2 Grasp Library node up...");

  detector_thread_ = new std::thread(&GraspDetectorOPD::onInit, this);
  detector_thread_->detach();
}

void GraspDetectorOPD::onInit()
{
  rclcpp::Rate rate(100);
  RCLCPP_INFO(logger_, "Waiting for point cloud to arrive ...");

  while (rclcpp::ok()) {
    if (has_cloud_) {
      // detect grasps in point cloud
      std::vector<Grasp> grasps = detectGraspPosesInTopic();
      // reset the system
      has_cloud_ = false;
      RCLCPP_INFO(logger_, "Waiting for point cloud to arrive ...");
    }

    // rclcpp::spin(shared_from_this());
    rate.sleep();
  }
}

std::vector<Grasp> GraspDetectorOPD::detectGraspPosesInTopic()
{
  // detect grasp poses
  std::vector<Grasp> grasps;
#if 1
  // publish grasp pose 
  //r = (R0, R1, R2) = (x, y, z) same as 3D model
  tf2::Matrix3x3 r(obj_rot_(0, 0), obj_rot_(0, 1), obj_rot_(0, 2),
                   obj_rot_(1, 0), obj_rot_(1, 1), obj_rot_(1, 2),
                   obj_rot_(2, 0), obj_rot_(2, 1), obj_rot_(2, 2));
#else
  Eigen::Vector3d axis(obj_rot_(0, 0), obj_rot_(1, 0), obj_rot_(2, 0));
  Eigen::Vector3d axis0(axis[0], axis[1], 0);
  // angle-axis rotation
  Eigen::Vector3d binormal0 = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix() * axis0;
  Eigen::Vector3d approach0 = Eigen::AngleAxisd(-M_PI/2, axis).toRotationMatrix() * binormal0;
  tf2::Matrix3x3 r(
    approach0[0], binormal0[0], axis[0],
    approach0[1], binormal0[1], axis[1],
    approach0[2], binormal0[2], axis[2]);
//#else
  Eigen::Vector3d axis(obj_rot_(0, 0), obj_rot_(1, 0), obj_rot_(2, 0));
  Eigen::Vector3d axis0(axis[0], axis[1], 0);
  Eigen::Vector3d binormal0 = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix() * axis0;
  Eigen::Vector3d approach0 = Eigen::AngleAxisd(-M_PI/2, axis).toRotationMatrix() * binormal0;

  tf2::Matrix3x3 r(
    binormal0[0], axis[0], approach0[0],
    binormal0[1], axis[1], approach0[1],
    binormal0[2], axis[2], approach0[2]);
#endif
  tf2::Quaternion quat;
  r.getRotation(quat);
  quat.normalize();
  // broadcast grasp pose
  RCLCPP_INFO(logger_, "**********detected object pose [position %f %f %f]",
        obj_tran_(0), obj_tran_(1), obj_tran_(2));
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header = cloud_camera_header_;
  tf_msg.child_frame_id = "grasp_pose_opd";
  tf_msg.transform.translation.x = obj_tran_(0);
  tf_msg.transform.translation.y = obj_tran_(1);
  tf_msg.transform.translation.z = obj_tran_(2);
  tf_msg.transform.rotation = tf2::toMsg(quat);
  tfb_.sendTransform(tf_msg);
  // Grasps
  const HandSearch::Parameters & params = grasp_detector_->getHandSearchParameters();
  FingerHand finger_hand(params.finger_width_, params.hand_outer_diameter_, params.hand_depth_);
  Eigen::Vector3d tran = obj_tran_.cast<double>();
  Eigen::Matrix3d rot = obj_rot_.cast<double>();

  std::cout << tran << std::endl;
  std::cout << rot << std::endl;
  std::cout << "==================++++====="<< std::endl;
/*
  rot << r[0][0], r[0][1],r[0][2],
         r[1][0], r[1][1],r[1][2],
         r[2][0], r[2][1],r[2][2];*/

  grasp_msgs::msg::GraspConfigList selected_grasps_msg;
  selected_grasps_msg.header = cloud_camera_header_;
  selected_grasps_msg.object_name = object_name_;
  grasp_msgs::msg::GraspConfig msg;
  pointEigenToMsg(tran, msg.bottom);
  pointEigenToMsg(tran, msg.top);
  pointEigenToMsg(tran, msg.surface);
  vectorEigenToMsg(rot.col(0), msg.approach);
  vectorEigenToMsg(rot.col(1), msg.binormal);
  vectorEigenToMsg(rot.col(2), msg.axis);
  msg.width.data = 0.01;
  msg.score.data = 100; // todo: convert form fitness score
  pointEigenToMsg(tran, msg.sample);
  selected_grasps_msg.grasps.push_back(msg);

  // Publish the selected grasps.
  if (grasp_cb_) {
    grasp_cb_->grasp_callback(
      std::make_shared<grasp_msgs::msg::GraspConfigList>(selected_grasps_msg));
  }
  grasps_pub_->publish(selected_grasps_msg);
  RCLCPP_INFO(logger_, "Published %d highest-scoring grasps.", selected_grasps_msg.grasps.size());

  return grasps;
}

typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLVisualizer;
void plot(const PointCloudRGBA::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normal) {
  PCLVisualizer viewer(new pcl::visualization::PCLVisualizer("pcl viewer"));
  viewer->setPosition(100, 100);
  viewer->setSize(640, 480);
  viewer->setBackgroundColor(1.0, 1.0, 1.0);
  auto keyboardEventOccurred = [](const pcl::visualization::KeyboardEvent &event, void *viewer_void) -> void {
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
    if (event.getKeySym() == "a" && event.keyDown()) {
      if (viewer->contains("ref")) {
        viewer->removeCoordinateSystem("ref");
      } else {
        viewer->addCoordinateSystem(0.1, "ref");
      }
    }
  };
  viewer->registerKeyboardCallback(keyboardEventOccurred, (void *)viewer.get());
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, "registered point cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "registered point cloud");
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(1));
  }
  viewer->close();
}


fpfhFeature::Ptr compute_fpfh_feature(PointCloudRGB::Ptr input_cloud,pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree)
{
  pointnormal::Ptr point_normal (new pointnormal);
  pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> est_normal;
  est_normal.setInputCloud(input_cloud);
  est_normal.setSearchMethod(tree);
  //est_normal.setKSearch(10);
  est_normal.setRadiusSearch(0.05);
  est_normal.compute(*point_normal);
  // fpfh
  fpfhFeature::Ptr fpfh (new fpfhFeature);
  //pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
  pcl::FPFHEstimationOMP<pcl::PointXYZRGBA, pcl::Normal,pcl::FPFHSignature33> est_fpfh;
  est_fpfh.setNumberOfThreads(4);
  // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
  est_fpfh.setInputCloud(input_cloud);
  est_fpfh.setInputNormals(point_normal);
  est_fpfh.setSearchMethod(tree);
  //est_fpfh.setKSearch(10);
  est_fpfh.setRadiusSearch(0.05);
  est_fpfh.compute(*fpfh);

  return fpfh;
}

void removeNans(PointCloudRGB::Ptr cloud) {
  cloud->is_dense = false;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::removeNaNFromPointCloud(*cloud, inliers->indices);
  //printf("======== %d ====%zu %zu\n", cloud->is_dense, inliers->indices.size(), cloud->size());
  if (inliers->indices.size() < cloud->size()) {
    pcl::ExtractIndices<pcl::PointXYZRGBA> eifilter(true);
    eifilter.setInputCloud(cloud);
    eifilter.setIndices(inliers);
    eifilter.filter(*cloud);
    //printf("Cloud after removing NANs: %zu\n", cloud->size());
  }
}

Eigen::Matrix4f readPoseFromHDF5(
    const std::string &hdf5_filename, const std::string &dsname) {
  cv::Ptr<cv::hdf::HDF5> h5io = cv::hdf::open(hdf5_filename);
  cv::Mat mat_cv(4, 4, CV_32FC1);
  h5io->dsread(mat_cv, dsname);
  Eigen::Matrix4f mat_eigen;
  cv::cv2eigen(mat_cv, mat_eigen);
  h5io->close();

  return mat_eigen;
}

void GraspDetectorOPD::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!auto_mode_ && !started_) {return;}
#ifdef RECOGNIZE_PICK
  people_msgs::msg::ObjectsInMasks::SharedPtr object_msg;
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
#endif
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
      PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
#if 1
      pcl::fromROSMsg(*msg, *cloud);

      // filter workspace
      for (uint32_t i = 0; i < cloud->size(); i++) {
        if (cloud->points[i].x > detection_param_.workspace_[0] && cloud->points[i].x < detection_param_.workspace_[1] &&
            cloud->points[i].y > detection_param_.workspace_[2] && cloud->points[i].y < detection_param_.workspace_[3] &&
            cloud->points[i].z > detection_param_.workspace_[4] && cloud->points[i].z < detection_param_.workspace_[5]) {
          continue;
        } else {
          cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
          cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
          cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
        }
      }

      // remove table plane
      if (plane_remove_) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.025);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        for (size_t i = 0; i < inliers->indices.size(); ++i) {
          cloud->points[inliers->indices[i]].x = std::numeric_limits<float>::quiet_NaN();
          cloud->points[inliers->indices[i]].y = std::numeric_limits<float>::quiet_NaN();
          cloud->points[inliers->indices[i]].z = std::numeric_limits<float>::quiet_NaN();
        }
      }
#endif
#ifdef RECOGNIZE_PICK
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
                if (!isnan(cloud->points[idx + j].x) &&
                  !isnan(cloud->points[idx + j].y) &&
                  !isnan(cloud->points[idx + j].z))
                {
                  indices.push_back(idx + j);
                }
              }
            }
            pcl::ExtractIndices<pcl::PointXYZRGBA> filter;
            filter.setInputCloud(cloud);
            filter.setIndices(boost::make_shared<std::vector<int>>(indices));
            filter.filter(*cloud);
            Eigen::Matrix3Xf xyz =
              cloud->getMatrixXfMap(3, sizeof(pcl::PointXYZRGBA) / sizeof(float), 0);
            RCLCPP_INFO(logger_, "*************** %f %f, %f %f, %f %f",
              xyz.row(0).minCoeff(), xyz.row(0).maxCoeff(),
              xyz.row(1).minCoeff(), xyz.row(1).maxCoeff(),
              xyz.row(2).minCoeff(), xyz.row(2).maxCoeff());
            grasp_ws_ = {xyz.row(0).minCoeff(), xyz.row(0).maxCoeff(),
              xyz.row(1).minCoeff(), xyz.row(1).maxCoeff(),
              xyz.row(2).minCoeff(), xyz.row(2).maxCoeff()};
            found = true;
            break;
          }
        }
        if (!found) {return;}
      }
#endif

      clock_t t1, t2;
      // load mesh
      PointCloudRGBA::Ptr mesh_cloud(new PointCloudRGBA);

      std::string filename("bolt1.obj");
      pcl::PolygonMesh mesh;
      if (pcl::io::loadPolygonFileOBJ(filename, mesh) == -1) {
        printf("Couldn't read OBJ file: %s\n", filename.c_str());
      } else {
        pcl::fromPCLPointCloud2(mesh.cloud, *mesh_cloud);
        for (uint32_t i = 0; i < mesh_cloud->size(); i++) {
          mesh_cloud->points[i].x /= 1000;
          mesh_cloud->points[i].y /= 1000;
          mesh_cloud->points[i].z /= 1000;
        }
      }
      // down sampling
      pcl::VoxelGrid<pcl::PointXYZRGBA> vox;
      vox.setInputCloud(mesh_cloud);
      vox.setLeafSize(0.0009f, 0.0009f, 0.0009f);
      vox.filter(*mesh_cloud);
      vox.setInputCloud(cloud);
      vox.setLeafSize(0.00000005f, 0.00000005f, 0.00000005f);
      //vox.filter(*cloud);
      removeNans(mesh_cloud);
      printf("%zu points in mesh_cloud\n", mesh_cloud->size());
      removeNans(cloud);
      printf("%zu points in cloud\n", cloud->size());
      //plot(mesh_cloud, NULL);

      t1 = clock();
      // estimate normals
      pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimation;
      pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
      normal_estimation.setSearchMethod(search_tree);
      normal_estimation.setRadiusSearch(0.05);
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>()),
	      mesh_normals(new pcl::PointCloud<pcl::Normal>()),
	      reg_normals(new pcl::PointCloud<pcl::Normal>());
      normal_estimation.setInputCloud(cloud);
      normal_estimation.compute(*cloud_normals);
      normal_estimation.setInputCloud(mesh_cloud);
      normal_estimation.compute(*mesh_normals);

      PointCloudRGB::Ptr reg_cloud(new PointCloudRGB);
#if 1
      // sac align
      pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>());
      fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(mesh_cloud, tree);
      fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(cloud, tree);
      pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGBA, pcl::PointXYZRGBA, pcl::FPFHSignature33> sac_ia;
      sac_ia.setCorrespondenceRandomness(350);
      sac_ia.setSourceFeatures(source_fpfh);
      sac_ia.setTargetFeatures(target_fpfh);
      sac_ia.setInputSource(mesh_cloud);
      sac_ia.setInputTarget(cloud);
      sac_ia.align(*reg_cloud);
      auto t = sac_ia.getFinalTransformation();
      auto score = sac_ia.getFitnessScore();
#else
      // prepare point normals
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>()),
              mesh_xyz(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::copyPointCloud(*cloud, *cloud_xyz);
      pcl::copyPointCloud(*mesh_cloud, *mesh_xyz);
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>()),
              mesh_with_normals(new pcl::PointCloud<pcl::PointNormal>());
      pcl::concatenateFields(*cloud_xyz, *cloud_normals, *cloud_with_normals);
      pcl::concatenateFields(*mesh_xyz, *mesh_normals, *mesh_with_normals);
      // compute ppf features, global model
      pcl::PPFEstimation<pcl::PointXYZ, pcl::Normal, pcl::PPFSignature> ppf_estimator;
      pcl::PointCloud<pcl::PPFSignature>::Ptr source_ppf (new pcl::PointCloud<pcl::PPFSignature>());
      ppf_estimator.setInputCloud(mesh_xyz);
      ppf_estimator.setInputNormals(mesh_normals);
      ppf_estimator.compute(*source_ppf);
      // create hash map
      pcl::PPFHashMapSearch::Ptr hash_map_search (new pcl::PPFHashMapSearch(M_PI/15, 0.05));
      hash_map_search->setInputFeatureCloud(source_ppf);
      // do registration, local match
      pcl::PPFRegistration<pcl::PointNormal, pcl::PointNormal> ppf;
      //ppf.setMaxCorrespondenceDistance(0.0002);
      //ppf.setMaximumIterations(50);
      //ppf.setTransformationEpsilon(1e-8);
      //ppf.setEuclideanFitnessEpsilon(0.0002);
      ppf.setSceneReferencePointSamplingRate(1);
      ppf.setPositionClusteringThreshold(0.1);
      ppf.setRotationClusteringThreshold(M_PI/12);
      ppf.setSearchMethod(hash_map_search);
      ppf.setInputSource(mesh_with_normals);
      ppf.setInputTarget(cloud_with_normals);
      pcl::PointCloud<pcl::PointNormal>::Ptr reg_with_normals(new pcl::PointCloud<pcl::PointNormal>());
      while(!ppf.hasConverged()) {
        ppf.align(*reg_with_normals);
        ppf.setInputSource(reg_with_normals);
      }
      pcl::copyPointCloud(*reg_with_normals, *reg_cloud);
      auto t = ppf.getFinalTransformation();
      auto score = ppf.getFitnessScore()
#endif

      // do icp
      pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
      icp.setInputTarget(cloud);
      icp.setMaxCorrespondenceDistance(0.2);
      icp.setMaximumIterations(500);
      icp.setTransformationEpsilon(1e-8);
      icp.setEuclideanFitnessEpsilon(0.0002);
      while(!icp.hasConverged()) {
        icp.setInputCloud(reg_cloud);
        icp.align(*reg_cloud);
        t = icp.getFinalTransformation()*t;
      }
      score = icp.getFitnessScore();
      printf("==== icp fitness score %f\n", icp.getFitnessScore());

      t2 = clock();
      removeNans(reg_cloud);
      //printf("%zu points in reg_cloud\n", reg_cloud->size());
      //plot(reg_cloud, NULL);
      normal_estimation.setInputCloud(reg_cloud);
      normal_estimation.compute(*reg_normals);
      // get transform
      obj_rot_ = t.block<3,3>(0, 0);
      obj_tran_ = t.block<3,1>(0, 3);
      printf("==== Fitness score %f\n", score);
      std::cout << "====== Time for object pose detection: " << double(t2-t1)/CLOCKS_PER_SEC << std::endl;
      std::cout << "====transform " << std::endl;
      std::cout << obj_rot_ << std::endl;
      std::cout << obj_tran_ << std::endl;

      opd_count_++;
      err_tran_sum_ += score; //(obj_tran_ - tran_gt).norm();
      err_tran_mean_ = err_tran_sum_ / opd_count_;
      printf("==== object pose average tran err: %f mm\n", err_tran_mean_*1000);

      if (filtered_pub_) {
        sensor_msgs::msg::PointCloud2 msg2;
        pcl::toROSMsg(*cloud, msg2);
        msg2.header = msg->header;
        msg2.fields[3].name = "rgb";
        msg2.fields[3].datatype = 7;
        filtered_pub_->publish(msg2);

        sensor_msgs::msg::PointCloud2 msg3;
        pcl::toROSMsg(*reg_cloud, msg3);
        msg3.header = msg->header;
        msg2.fields[3].name = "rgb";
        msg2.fields[3].datatype = 7;
        reg_pub_->publish(msg3);
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

#ifdef RECOGNIZE_PICK
void GraspDetectorOPD::object_callback(const people_msgs::msg::ObjectsInMasks::SharedPtr msg)
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
#endif
grasp_msgs::msg::GraspConfigList GraspDetectorOPD::createGraspListMsg(
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

grasp_msgs::msg::GraspConfig GraspDetectorOPD::convertToGraspMsg(const Grasp & hand)
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

}  // namespace grasp_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(grasp_ros2::GraspDetectorOPD)
