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

#ifndef GRASP_LIBRARY__ROS2__CONSTS_HPP_
#define GRASP_LIBRARY__ROS2__CONSTS_HPP_

#include <string>

namespace grasp_ros2
{

/** Consts class
 *
 * \brief A class contains global constatnts definition for grasp library.
 *
 */
class Consts
{
public:
  /** Topic name of "PointCloud2" message published by an RGBD sensor.*/
  static const char kTopicPointCloud2[];
  /** Topic name of "detected objects" message published by Object Detector.*/
  static const char kTopicDetectedObjects[];
  /** Topic name of "detected grasps" message published by this Grasp Detector.*/
  static const char kTopicDetectedGrasps[];
  /** Topic name of "rviz grasps" message published by this Grasp Detector.*/
  static const char kTopicVisualGrasps[];
  /** Topic name of "tabletop pointcloud" message published by this Grasp Detector.*/
  static const char kTopicTabletop[];
};

}  // namespace grasp_ros2

#endif  // GRASP_LIBRARY__ROS2__CONSTS_HPP_
