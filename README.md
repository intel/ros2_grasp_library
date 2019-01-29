# ROS2 OpenVINO Grasp Library

## Overview
ROS2 Grasp Library enables grasp detection algorithms on ROS2 for visual based industrial robot manipulation. This package provides two levels of interface. Application can decide which level to use, depending on whether MoveIt! framework is adopted.
* ROS2 grasp planning service as MoveIt plugin, used by MoveIt manipulation applications
* ROS2 topic conveys grasp detection results, used by other ROS/ROS2 manipulation applications

The above interfaces can be bridged back to ROS via [ros1_bridge](https://github.com/ros2/ros1_bridge/blob/master/README.md)

ROS2 Grasp Library keep enabling various grasp detection algorithms in the back-end.
- [Grasp Pose Detection](https://github.com/atenpas/gpd) detects 6-DOF grasp poses for a 2-finger grasp (e.g. a parallel jaw gripper) in 3D point clouds from RGBD sensor or PCD file. The grasp detection workloads are accelerated with Intel [OpenVINO™](https://software.intel.com/en-us/openvino-toolkit) toolkit for deployment across various Intel vision devices CPU, GPU, Movidius VPU, and FPGA.

  <img src="docs/img/ros2_grasp_library.png" width = 50% height = 50% alt="ROS2 Grasp Library" align=center />


## Requirements
### Hardware
* Host running ROS2/ROS
* RGBD sensor
### Software
We verified the software with Ubuntu 18.04 Bionic and ROS2 Crystal Clemmys release.
* Install ROS2 packages
  [ros-crystal-desktop](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians)

* Install non ROS packages
  ```bash
  sudo apt-get install libpcl-dev libeigen3-dev
  ```

* Install [Intel OpenVINO Toolkit](docs/install_openvino.md)
* Install [GPD](docs/install_gpd.md)

## Get source codes of Grasp Library
```bash
mkdir ~/ros2_ws/src -p
cd ~/ros2_ws/src
git clone https://github.com/intel/ros2_grasp_library.git
# copy GPD models
cp -a <path-to-gpd-repo>/models ros2_grasp_library/gpd
```

## Build Grasp Library
```bash
colcon build --symlink-install --packages-select grasp_msgs moveit_msgs grasp_library
```

## Launch Grasp Library
```bash
# Terminal 1, launch RGBD camera, e.g. [ROS2 Realsenes](https://github.com/intel/ros2_intel_realsense)
ros2 run realsense_ros2_camera realsense_ros2_camera
# Terminal 2, launch Grasp Library
ros2 launch grasp_library grasp_library.launch.py cloud_topic:=<name of point cloud topic>
# Terminal 3, Optionally, launch Rviz2 to illustrate detection results.
# NOTE You may customize the ".rviz" file for your own camera, like for the "cloud_topic" name
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/ros2_grasp_library/grasp_library/rviz2/grasp.rviz
```

### Launch Options
* cloud_topic: name of point cloud topic

## Test Grasp Library
Sanity test cases are very basic tests cover ROS2 topic and ROS2 service for Grasp Library. The tests take inputs from a pre-stored PointCloud file (.pcd). Thus it's unnecessary to launch an RGBD camera.
```bash
# Terminal 1, launch Grasp Library
ros2 launch grasp_library grasp_library.launch.py cloud_topic:=/camera/depth_registered/points
# Terminal 2, run tests
colcon test --packages-select grasp_msgs grasp_library
```
For failed cases check detailed logs at "ros2_ws/log/latest_test/<package_name>/stdout.log".

## ROS2 Interfaces
### Subscribed Topics
  * /camera/depth_registered/points ([sensor_msgs::msg::PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)), PointCloud2 messages from RGBD camera

### Published Topics
  * /grasp_library/clustered_grasps ([grasp_msgs::msg::GraspConfigList](https://github.com/intel/ros2_grasp_library/blob/master/grasp_msgs/msg/GraspConfigList.msg)), detected grasps from GPD

### Delivered Services
  * plan_grasps ([moveit_msgs::srv::GraspPlanning](https://github.com/intel/ros2_grasp_library/blob/master/moveit_msgs_light/srv/GraspPlanning.srv)), MoveIt! grasp planning service

## Known Issues
  * Cloud camera failed at "Invalid sizes when resizing a matrix or array" when dealing with XYZRGBA pointcloud from ROS2 Realsenes, tracked as [#6](https://github.com/atenpas/gpg/issues/6) of gpg, [patch](https://github.com/atenpas/gpg/pull/7) under review.

## Contribute to This Project
  It's welcomed to contribute to this project. Here're some recommended practices:
  * When adding a new feature it's expected to add tests covering the new functionalities.
  * Before submitting a patch, it's recommended to pass all existing tests to avoid regression.

###### *Any security issue should be reported using process at https://01.org/security*
