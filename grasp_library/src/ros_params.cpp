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

#include <string>
#include "grasp_library/ros_params.h"

void ROSParameters::getDetectionParams(
  rclcpp::Node * node,
  GraspDetector::GraspDetectionParameters & param)
{
  // Read hand geometry parameters.
  node->get_parameter_or("finger_width", param.hand_search_params.finger_width_, 0.01);
  node->get_parameter_or("hand_outer_diameter", param.hand_search_params.hand_outer_diameter_,
    0.10);
  node->get_parameter_or("hand_depth", param.hand_search_params.hand_depth_, 0.08);
  node->get_parameter_or("hand_height", param.hand_search_params.hand_height_, 0.02);
  node->get_parameter_or("init_bite", param.hand_search_params.init_bite_, 0.01);

  // Read local hand search parameters.
  node->get_parameter_or("nn_radius", param.hand_search_params.nn_radius_frames_, 0.01);
  node->get_parameter_or("num_orientations", param.hand_search_params.num_orientations_, 8);
  node->get_parameter_or("num_samples", param.hand_search_params.num_samples_, 100);
  node->get_parameter_or("num_threads", param.hand_search_params.num_threads_, 4);
  node->get_parameter_or("rotation_axis", param.hand_search_params.rotation_axis_, 2);

  // Read plotting parameters.
  node->get_parameter_or("plot_samples", param.plot_samples_, false);
  node->get_parameter_or("plot_normals", param.plot_normals_, false);
  param.generator_params.plot_normals_ = param.plot_normals_;
  node->get_parameter_or("plot_filtered_grasps", param.plot_filtered_grasps_, false);
  node->get_parameter_or("plot_valid_grasps", param.plot_valid_grasps_, false);
  node->get_parameter_or("plot_clusters", param.plot_clusters_, false);
  node->get_parameter_or("plot_selected_grasps", param.plot_selected_grasps_, false);

  // Read general parameters.
  param.generator_params.num_samples_ = param.hand_search_params.num_samples_;
  param.generator_params.num_threads_ = param.hand_search_params.num_threads_;
  node->get_parameter_or("plot_candidates", param.generator_params.plot_grasps_, false);

  // Read preprocessing parameters.
  node->get_parameter_or("remove_outliers", param.generator_params.remove_statistical_outliers_,
    false);
  node->get_parameter_or("voxelize", param.generator_params.voxelize_, true);
  // todo passed from launch
  // node->get_parameter("workspace", param.generator_params.workspace_);
  std::initializer_list<double> workspace = {-1, 1, -1, 1, -1, 1};
  param.generator_params.workspace_ = workspace;
  // todo passed from launch
  // node->get_parameter("workspace_grasps", param.workspace_);
  std::initializer_list<double> ws_grasps = {0, 1.0, -1, 1.0, -1, 1.0};
  param.workspace_ = ws_grasps;

  // Read classification parameters and create classifier.
  node->get_parameter_or("model_file", param.model_file_, std::string(""));
  node->get_parameter_or("trained_file", param.weights_file_, std::string(""));
  node->get_parameter_or("min_score_diff", param.min_score_diff_, 500.0);
  node->get_parameter_or("create_image_batches", param.create_image_batches_, false);
  node->get_parameter_or("device", param.device_, 0);

  // Read grasp image parameters.
  node->get_parameter_or("image_outer_diameter", param.image_params.outer_diameter_,
    param.hand_search_params.hand_outer_diameter_);
  node->get_parameter_or("image_depth", param.image_params.depth_,
    param.hand_search_params.hand_depth_);
  node->get_parameter_or("image_height", param.image_params.height_,
    param.hand_search_params.hand_height_);
  node->get_parameter_or("image_size", param.image_params.size_, 60);
  node->get_parameter_or("image_num_channels", param.image_params.num_channels_, 15);

  // Read learning parameters.
  node->get_parameter_or("remove_plane_before_image_calculation", param.remove_plane_, false);

  // Read grasp filtering parameters
  node->get_parameter_or("filter_grasps", param.filter_grasps_, false);
  node->get_parameter_or("filter_half_antipodal", param.filter_half_antipodal_, false);
  param.gripper_width_range_.push_back(0.03);
  param.gripper_width_range_.push_back(0.07);
  // node->get_parameter("gripper_width_range", param.gripper_width_range_);

  // Read clustering parameters
  node->get_parameter_or("min_inliers", param.min_inliers_, 1);

  // Read grasp selection parameters
  node->get_parameter_or("num_selected", param.num_selected_, 5);
}

void ROSParameters::getPlanningParams(
  rclcpp::Node * node,
  GraspPlanner::GraspPlanningParameters & param)
{
  node->get_parameter_or("grasp_offset", param.grasp_offset_, -0.08);
  node->get_parameter_or("grasp_cache_time_threshold", param.grasp_cache_time_threshold_, 5);
}
