# ROS2 Grasp Library

## Overview
ROS2 Grasp Library enables state-of-the-art CNN based deep learning grasp detection algorithms on ROS2 for visual based industrial robot manipulation. This package provide ROS2 interfaces compliant with the [MoveIt](http://moveit.ros.org/) motion planning framework which is supported by most of the [robot models](https://moveit.ros.org/robots) in ROS industrial. This package delivers
* A ROS2 Grasp Planner providing grasp planning service, as an extensible capability of MoveIt ([moveit_msgs::srv::GraspPlanning](http://docs.ros.org/api/moveit_msgs/html/srv/GraspPlanning.html))
* A ROS2 Grasp Detector generic interface, collaborating with Grasp Planner for grasp detection. Also a specific back-end algorithm enabled under this interface: [Grasp Pose Detection](https://github.com/atenpas/gpd) with Intel® [OpenVINO™](https://software.intel.com/en-us/openvino-toolkit) technology
* Grasp transformation from camera frame to a specified target frame expected in the visual manipulation; Grasp translation to the MoveIt Interfaces ([moveit_msgs::msg::Grasp](http://docs.ros.org/api/moveit_msgs/html/msg/Grasp.html))
* A 'service-driven' grasp detection mechanism (via configure [auto_mode](docs/tutorials_3_grasp_ros2_launch_options.md)) to optimize CPU load for real-time processing

## Grasp Detection Algorithms
Grasp detection back-end algorithms enabled by this Grasp Library:
- [Grasp Pose Detection](https://github.com/atenpas/gpd) detects 6-DOF grasp poses for a 2-finger grasp (e.g. a parallel jaw gripper) in 3D point clouds from RGBD sensor or PCD file. The grasp detection was enabled with Intel® DLDT toolkit and Intel® OpenVINO™ toolkit.

  <img src="grasp_tutorials/doc/grasp_ros2/img/ros2_grasp_library.png" width = 50% height = 50% alt="ROS2 Grasp Library" align=center />

## ROS2 Interfaces
### Subscribed Topics
  * /camera/depth_registered/points ([sensor_msgs::msg::PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)), PointCloud2 messages from RGBD camera

### Published Topics
ROS2 topic conveys grasp detection results, used by non-MoveIt manipulation applications
  * /grasp_library/clustered_grasps ([grasp_msgs::msg::GraspConfigList](https://github.com/intel/ros2_grasp_library/blob/master/grasp_msgs/msg/GraspConfigList.msg)), detected grasps from GPD

### Delivered Services
ROS2 grasp planning service as MoveIt plugin, used by MoveIt manipulation applications
  * plan_grasps ([moveit_msgs::srv::GraspPlanning](http://docs.ros.org/api/moveit_msgs/html/srv/GraspPlanning.html)), MoveIt grasp planning service

ROS2 interfaces can be bridged back to ROS via [ros1_bridge](https://github.com/ros2/ros1_bridge/blob/master/README.md)

## Known Issues
  * Cloud camera failed at "Invalid sizes when resizing a matrix or array" when dealing with XYZRGBA pointcloud from ROS2 Realsenes, tracked as [#6](https://github.com/atenpas/gpg/issues/6) of gpg, [patch](https://github.com/atenpas/gpg/pull/7) under review.
  * 'colcon test' sometimes failed with test suite "tgrasp_ros2", due to ROS2 service request failure issue (reported ros2 examples issue [#228](https://github.com/ros2/examples/issues/228) and detailed discussed in ros2 demo issue [#304](https://github.com/ros2/demos/issues/304))
  * Rviz2 failed to receive Static TF from camera due to transient_local QoS (expected in the coming ROS2 Eloquent, discussed in geometry2 issue [#183](https://github.com/ros2/geometry2/issues/183)), workaround [patch](https://github.com/intel/ros2_intel_realsense/pull/87) available till the adaption to Eloquent

## Contribute to This Project
  It's welcomed to contribute to this project. Here're some recommended practices:
  * When adding a new feature it's expected to add tests covering the new functionalities.
  * Before submitting a patch, it's recommended to pass all existing tests to avoid regression. See [Grasp Library Test](docs/tutorials_2_grasp_ros2_test.md)


###### *Any security issue should be reported using process at https://01.org/security*
