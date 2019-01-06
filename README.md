# ros2_grasp_library
ROS2 Grasp Library enables grasp detection algorithms on ROS2 for visual based industrial robot manipulation. This package provides two levels of interface. Application can decide which level to use, depending on whether MoveIt! framework is adopted.
* ROS2 service compliant with MoveIt interface, used by MoveIt manipulation applications
* ROS2 topic conveys grasp detection results, used by other ROS/ROS2 manipulation applications

The above interfaces can be bridged back to ROS via [ros1_bridge](https://github.com/ros2/ros1_bridge/blob/master/README.md)

ROS2 Grasp Library keep enabling various grasp detection algorithms in the back-end.
- [Grasp Pose Detection](https://github.com/atenpas/gpd) detects 6-DOF grasp poses for a 2-finger grasp (e.g. a parallel jaw gripper) in 3D point clouds. The grasp detection workloads are accelerated with Intel [OpenVINO™](https://software.intel.com/en-us/openvino-toolkit) toolkit
for deployment across various Intel vision devices CPU, GPU, Movidius VPU, and FPGA.

<img src="https://github.intel.com/otc-rse/ros2_grasp_library/blob/master/docs/img/ros2_grasp_library.png" width = "596" height = "436" alt="ROS2 Grasp Library" align=center />

## System Requirements
Ubuntu Linux 16.04 on 64-bit

## Dependencies
### Install ROS2 packages [ros-bouncy-desktop](https://github.com/ros2/ros2/wiki/Linux-Install-Debians)
  * ament_cmake
  * ament_lint_auto, ament_lint_common
  * rclcpp, ros2run
  * rosidl_default_generators, rosidl_default_runtime, rosidl_interface_packages
  * std_msgs, sensor_msgs, geometry_msgs, shape_msgs, trajectory_msgs
  * pcl_conversions

### Install non ROS packages
  * PCL 1.7 or later (debian package "libpcl-dev")
  * Eigen 3.0 or later (debian package "libeigen3-dev")
  An example to install the above packages with command line:
  ```bash
  sudo apt-get install libpcl-dev libeigen3-dev
  ```

### Install [Intel OpenVINO Toolkit](https://github.com/opencv/dldt)
1. Build and install Inference Engine for [Linux](https://github.com/opencv/dldt/blob/2018/inference-engine/README.md#build-on-linux-systems). CMake options for your ref
   ```bash
   cd dldt/inference-engine
   mkdir build && cd build
   cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DGEMM=MKL -DMKLROOT=/usr/local/lib/mklml -DENABLE_MKL_DNN=ON -DENABLE_CLDNN=ON ..
   make -j8
   sudo make install
   ```
2. Share the CMake configures for Inference Engine to be found by other packages
   ```bash
   sudo mkdir /usr/share/InferenceEngine
   sudo cp InferenceEngineConfig*.cmake /usr/share/InferenceEngine
   sudo cp targets.cmake /usr/share/InferenceEngine
   ```
   Then Inference Engine will be found when adding "find_package(InferenceEngine)" into the CMakeLists.txt
3. Configure library path for dynamic loading
   ```bash
   echo `pwd`/bin/intel64/Release/lib > /etc/ld.so.conf.d/openvino.conf
   sudo ldconfig
   ```
4. Optionally, install additional plug-ins for execution at GPU or Movidius NCS,
following [optional steps](https://software.intel.com/en-us/articles/OpenVINO-Install-Linux#inpage-nav-4)

### Install [GPG](https://github.com/atenpas/gpg)
1. Get the code
```bash
git clone https://github.com/atenpas/gpg.git
cd gpg
```
2. Build the library
```bash
mkdir build && cd build
cmake ..
make
sudo make install
# by default, "libgrasp_candidates_generator.so" shall be installed to "/usr/local/lib"
```

### Install [GPD](https://github.com/sharronliu/gpd)
1. Get the code
```bash
git clone https://github.com/sharronliu/gpd.git
git checkout libgpd
cd gpd/src/gpd
```
2. Build the library
```bash
mkdir build && cd build
cmake -DUSE_OPENVINO=ON ..
make
sudo make install
# by default, "libgrasp_pose_detection.so" shall be installed to "/usr/local/lib", and header files installed to "/usr/local/include/gpd"
```

## Get source codes of Grasp Library
```bash
mkdir ~/ros2_ws/src -p
cd ~/ros2_ws/src
git clone https://github.intel.com/otc-rse/ros2_grasp_library.git
# copy GPD models
cp -a <path-to-gpd-repo>/models ros2_grasp_library/gpd
```

## Build Grasp Library
```bash
colcon build --symlink-install --packages-select grasp_msgs moveit_msgs grasp_library
```

## Launch Grasp Library
Launch RGBD camera, e.g. [ROS2 Realsenes](https://github.com/intel/ros2_intel_realsense)
```bash
ros2 run realsense_ros2_camera realsense_ros2_camera
```
Launch Grasp Library
```bash
ros2 run grasp_library grasp_library
```

## Subscribed Topics
  * /camera/depth_registered/points ([sensor_msgs::msg::PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)), PointCloud2 messages from RGBD camera

## Published Topics
  * /grasp_library/clustered_grasps ([grasp_msgs::msg::GraspConfigList](https://github.com/intel/ros2_grasp_library/blob/master/grasp_msgs/msg/GraspConfigList.msg)), detected grasps from GPD

## Delivered Services
  * plan_grasps ([moveit_msgs::srv::GraspPlanning](https://github.com/intel/ros2_grasp_library/blob/master/moveit_msgs_light/srv/GraspPlanning.srv)), MoveIt! grasp planning service

###### *Any security issue should be reported using process at https://01.org/security*
