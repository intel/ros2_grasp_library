Recognize Pick (OpenVINO Grasp Detection + OpenVINO Object Segmentation)
========================================================================

Overview
--------

A simple application demonstrating how to pick up recognized objects with an industrial robot arm.
The application interact with Grasp Planner and Robot Interface from this Grasp Library.

Requirement
-----------

Before running the code, make sure you have followed the instructions below
to setup the environment.

- Hardware

  - Host running ROS2

  - RGBD sensor

  - `Robot Arm`_

  - `Robot Gripper`_

- Software

  - `ROS2`_

  - Grasp Planner

  - Robot Interface

  - Hand-Eye Calibration

  - `ROS2 OpenVINO <https://github.com/intel/ros2_openvino_toolkit>`_

  - RGBD Sensor

    - `realsense`_

.. _Robot Arm: https://www.universal-robots.com/products/ur5-robot

.. _Robot Gripper: https://www.universal-robots.com/plus/end-effectors/hitbot-electric-gripper

.. _ROS2: https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians 

.. _MoveIt: https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#install-moveit

.. _universal_robot: https://github.com/ros-industrial/universal_robot

.. _ur_modern_driver: https://github.com/ros-industrial/ur_modern_driver

.. _hitbot: https://github.intel.com/otc-rse/hitbot

.. _realsense: https://github.com/intel/ros2_intel_realsense/tree/refactor

.. _Grasp Pose Detection: https://github.com/sharronliu/gpd

.. _OpenVINO: https://github.com/sharronliu/gpd/blob/master/tutorials/tutorial_openvino.md

Download and Build the Application
----------------------------------

Within your catkin workspace, download and compile the example code

::

  cd <path_of_your_ros2_workspace>/src

  git clone https://github.com/intel/ros2_grasp_library.git

  cd ..

  colcon build --symlink-install --ament-cmake-args -DBUILD_RECOGNIZE_PICK=ON

- Build Options

  - BUILD_RECOGNIZE_PICK (ON | **OFF** )
    Switch on/off building of this application


Launch the Application with Real Robot and Camera
-------------------------------------------------

- Publish handeye transform, refer to handeye

::

  ros2 run tf2_ros static_transform_publisher 0.016289810558 0.83987282691 0.483312980714 0.375539744771 0.397068981197 -0.606356068939 0.577614440548 base_link camera_link

- Publish place object

::

  ros2 run recognize_pick place_publisher sports_ball

- Launch object segmentation and RGBD sensor

::

  ros2 launch dynamic_vino_sample pipeline_segmentation.launch.py
  # close the rviz2 window

- Launch UR description

::

  ros2 launch ur_description view_ur5_ros2.launch.py
  #load rviz2 configure file "src/ros2_grasp_library/grasp_apps/recognize_pick/rviz2/recognize_pick.rviz"

- Launch recognize pick app

::

  ros2 run recognize_pick recognize_pick

- Launch grasp planner

::

  ros2 run grasp_ros2 grasp_ros2 __params:=src/ros2_grasp_library/grasp_apps/recognize_pick/cfg/recognize_pick.yaml


ROS2 Parameters (to be updated)
-------------------------------
* **device** [0|1|2|3]: Configure device for grasp pose inference to execute, 0 for CPU, 1 for GPU, 2 for VPU, 3 for FPGA. In case OpenVINO plug-ins are installed, this configure deploy the CNN based deep learning inference on to the target device. Deploying the inference onto **GPU** or **VPU** will save CPU loads for other computation tasks.
* **plane_remove** [false|true]: Configure whether or not remove the planes (like the table plane) from point cloud input. Enabling this helps to avoid generating grasp poses across the table.
* **target_frame_id** ["base"|"string"]: Frame id expected for grasps returned from this service. When this parameter is specified, Grasp Planner try to transform the grasp from the original frame (usually a camera's color frame) to this target frame, given the TF available.
* **place_position** [[-0.45, -0.30, 0.25]|[1*3 double]]: Place position {x, y, z} in the target_frame_id.
* **joint_values_place** [[1*6] double]: Place position in joint values. For each place, this parameter specifies the target joint values for the arm moving to the place position.
* **joint_values_pick** [[1*6] double]: Pick position in joint values. For each pick, this parameter specifies the target joint values for the arm moving to the pre-pick position. This position is usually above the work table.
* **finger_joint_names** [[1*2] string]: Joint names of gripper fingers. Joint names are filled into MoveIt's grasp interface, to control the posture of hand for the position of 'pre_grasp_posture' and 'grasp_posture' (see [moveit_msgs::msg::Grasp](http://docs.ros.org/api/moveit_msgs/html/msg/Grasp.html)). Joint names are usually defined in URDF of the robot hand.
* **finger_positions_open** [[1*2] double]: Positions of all finger joints when the hand is in open status.
* **finger_positions_close** [[1*2] double]: Positions of all finger joints when the hand is in close status.
* **eef_yaw_offset** [PI/4|double]: The end-effector's yaw offset to its parent link.
* **boundry** [[1*6] double]: Workspace boundy, described as a cube {x_min, x_max, y_min, y_max, z_min, z_max} in metres in the target_frame_id.
* **object_height_min** [0.028|double]: Minimum height in metres (altitude above the work table) of object to grasp.
* **kThresholdScore** [20|float]: Minimum score of grasp.
* **grasp_approach** [[0, 0, -1]|[1*3] double]: Expected grasp approach direction.
* **approach_deviation** [PI/9|double]: Maximum deviation angle to the expected approach direction.
* **grasp_position_offset** [[1*2] double]: Grasp position offset introduced by the system (e.g. camera, hand-eye calibration, etc.) {x_offset, y_offset} in metres in the target_frame_id.
