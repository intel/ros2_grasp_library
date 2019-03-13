# OpenVINO Grasp Library with RGBD Camera

This tutorial introduce the OpenVINO environment setup, how to build and launch Grasp Library with RGBD camera.

## Requirements
### Hardware
* Host running ROS2/ROS
* RGBD sensor
### Software
We verified the software with Ubuntu 18.04 Bionic and ROS2 Crystal Clemmys release. Verification with ROS2 MoveIt is still work in progress. Before this, we have verified the grasp detection with MoveIt Melodic branch (tag 0.10.8) and our visual pick & place application.
* Install ROS2 packages
  [ros-crystal-desktop](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians)

* Install non ROS packages
  ```bash
  sudo apt-get install libpcl-dev libeigen3-dev
  ```

* Install [Intel OpenVINO Toolkit](install_openvino.md)
* Install [GPD](install_gpd.md)

## Build Grasp Library
```bash
# get the source codes
mkdir ~/ros2_ws/src -p
cd ~/ros2_ws/src
git clone https://github.com/intel/ros2_grasp_library.git
# copy GPD models
cp -a <path-to-gpd-repo>/models ros2_grasp_library/gpd
# build
cd ..
source /opt/ros/crystal/setup.bash
colcon build --symlink-install --packages-select grasp_msgs moveit_msgs grasp_library
source ./install/local_setup.bash
```

## Launch Grasp Library
```bash
# Terminal 1, launch RGBD camera
# e.g. launch [ROS2 Realsenes](https://github.com/intel/ros2_intel_realsense)
# or, with a ros-bridge, launch any ROS OpenNI RGBD cameras, like [ROS Realsense](https://github.com/intel-ros/realsense)
ros2 run realsense_ros2_camera realsense_ros2_camera
# Terminal 2, launch Grasp Library
ros2 run grasp_library grasp_library __params:=src/ros2_grasp_library/grasp_library/cfg/grasp_library_params.yaml
# Terminal 3, Optionally, launch Rviz2 to illustrate detection results.
# NOTE You may customize the ".rviz" file for your own camera, like "Global Options -> Fixed Frame" or "Point Cloud 2 -> Topic"
ros2 run rviz2 rviz2 -d src/ros2_grasp_library/grasp_library/rviz2/grasp.rviz
```
