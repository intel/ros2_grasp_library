# ros2_grasp_library
ROS2 Grasp Library enables grasp detection algorithms on ROS2 for visual based industrial robot manipulation. This package provides two levels of interface. Application can decide which level to use, depending on whether MoveIt! framework is adopted.
* ROS2 grasp planning service compliant with MoveIt interface, used by MoveIt manipulation applications
* ROS2 topic conveys grasp detection results, used by other ROS/ROS2 manipulation applications

The above interfaces can be bridged back to ROS via [ros1_bridge](https://github.com/ros2/ros1_bridge/blob/master/README.md)

ROS2 Grasp Library keep enabling various grasp detection algorithms in the back-end.
- [Grasp Pose Detection](https://github.com/atenpas/gpd) detects 6-DOF grasp poses for a 2-finger grasp (e.g. a parallel jaw gripper) in 3D point clouds from RGBD sensor or PCD file. The grasp detection workloads are accelerated with Intel [OpenVINO™](https://software.intel.com/en-us/openvino-toolkit) toolkit for deployment across various Intel vision devices CPU, GPU, Movidius VPU, and FPGA.

<img src="https://github.com/intel/ros2_grasp_library/blob/master/docs/img/ros2_grasp_library.png" width = "596" height = "436" alt="ROS2 Grasp Library" align=center />

## Dependencies
### Install ROS2 packages [ros-crystal-desktop](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians)

### Install non ROS packages
  * PCL 1.7 or later (debian package "libpcl-dev")
  * Eigen 3.0 or later (debian package "libeigen3-dev")
  An example to install the above packages with command line:
  ```bash
  sudo apt-get install libpcl-dev libeigen3-dev
  ```

### Install [Intel OpenVINO Toolkit](https://github.com/opencv/dldt)
1. Build and install Inference Engine for [Linux](https://github.com/opencv/dldt/blob/2018/inference-engine/README.md#build-on-linux-systems). Detailed steps for your ref
   ```bash
   git clone https://github.com/opencv/dldt.git
   cd dldt/inference-engine
   git submodule init
   git submodule update --recursive
   # install common dependencies
   source ./install_dependencies.sh
   # install mkl for cpu acceleration
   wget https://github.com/intel/mkl-dnn/releases/download/v0.17/mklml_lnx_2019.0.1.20180928.tgz
   tar -zxvf mklml_lnx_2019.0.1.20180928.tgz
   sudo ln -s `pwd`/mklml_lnx_2019.0.1.20180928 /usr/local/lib/mklml
   # install opencl for gpu acceleration
   wget https://github.com/intel/compute-runtime/releases/download/18.28.11080/intel-opencl_18.28.11080_amd64.deb
   sudo dpkg -i intel-opencl_18.28.11080_amd64.deb
   # build
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
   echo `pwd`/../bin/intel64/Release/lib | sudo tee -a /etc/ld.so.conf.d/openvino.conf
   sudo ldconfig
   ```

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
```bash
# Terminal 1, launch RGBD camera, e.g. [ROS2 Realsenes](https://github.com/intel/ros2_intel_realsense)
ros2 run realsense_ros2_camera realsense_ros2_camera
# Terminal 2, launch Grasp Library
ros2 run grasp_library grasp_library
# Terminal 3, Optionally, launch Rviz2 to illustrate detection results
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/ros2_grasp_library/grasp_library/rviz2/grasp.rviz 
```

## Test Grasp Library
Sanity test cases are very basic tests cover ROS2 topic and ROS2 service for Grasp Library. The tests take inputs from a pre-stored PointCloud file (.pcd). Thus it's unnecessary to launch an RGBD camera.
```bash
# Terminal 1, launch Grasp Library
ros2 run grasp_library grasp_library
# Terminal 2, run tests
colcon test --packages-select grasp_msgs grasp_library
```

## Subscribed Topics
  * /camera/depth_registered/points ([sensor_msgs::msg::PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)), PointCloud2 messages from RGBD camera

## Published Topics
  * /grasp_library/clustered_grasps ([grasp_msgs::msg::GraspConfigList](https://github.com/intel/ros2_grasp_library/blob/master/grasp_msgs/msg/GraspConfigList.msg)), detected grasps from GPD

## Delivered Services
  * plan_grasps ([moveit_msgs::srv::GraspPlanning](https://github.com/intel/ros2_grasp_library/blob/master/moveit_msgs_light/srv/GraspPlanning.srv)), MoveIt! grasp planning service

## Known Issues
  * Cloud camera failed at "Invalid sizes when resizing a matrix or array" when dealing with XYZRGBA pointcloud from ROS2 Realsenes, tracked as [#6](https://github.com/atenpas/gpg/issues/6) of gpg, [patch](https://github.com/atenpas/gpg/pull/7) under review.

## Contribute to This Project
  It's welcomed to contribute patches. Here're some recommended practices:
  * When adding a new feature it's expected to add tests covering the new functionalities.
  * Before submitting the patch, it's recommended to pass all existing tests to avoid regression.
    ```bash
    # terminal 1
    ros2 run grasp_library grasp_library
    # terminal 2
    colcon test --packages-select grasp_msgs grasp_library
    ```
    For failed cases check detailed logs at "ros2_ws/log/latest_test/<package_name>/stdout.log".

###### *Any security issue should be reported using process at https://01.org/security*
