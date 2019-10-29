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

  - `Robot Arm <https://www.universal-robots.com/products/ur5-robot>`_

  - `Robot Gripper`_

- Software

  - `ROS2 <https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians>`_

  - `Grasp Planner <https://github.com/sharronliu/ros2_grasp_library/tree/master/grasp_ros2>`_

  - `Robot Interface <https://github.com/sharronliu/ros2_grasp_library/tree/master/grasp_utils/robot_interface>`_

  - Hand-Eye Calibration

  - `ROS2 OpenVINO <https://github.com/intel/ros2_openvino_toolkit>`_

  - RGBD Sensor

    - `realsense <https://github.com/intel/ros2_intel_realsense/tree/refactor>`_

Download and Build the Application
----------------------------------

Within your catkin workspace, download and compile the example code

::

  cd <path_of_your_ros2_workspace>/src

  git clone https://github.com/intel/ros2_grasp_library.git

  cd ..

  colcon build --symlink-install

- Build Options

  - BUILD_RECOGNIZE_PICK (**ON** | OFF)
    Switch on/off building of this application


Launch the Application with Real Robot and Camera
-------------------------------------------------

- Publish handeye transform, refer to handeye

::

  ros2 run tf2_ros static_transform_publisher 0.029308663357415252, 0.8351437768013816, 0.4887928298268225 -0.002101048177610369, -0.9758220841680959, 0.2178663901368447 0.01735170582767297 base_link camera_color_optical_frame

- Publish place object

::

  ros2 run recognize_pick place_publisher sports_ball

- Launch UR description

::

  ros2 launch ur_description view_ur5_ros2.launch.py

  #load rviz2 configure file "src/ros2_grasp_library/grasp_apps/recognize_pick/rviz2/recognize_pick.rviz"

- Launch realsense

::

  ros2 run realsense_node realsense_node

- Launch object segmentation

::

  ros2 launch dynamic_vino_sample pipeline_segmentation.launch.py

  # close the rviz2 window

- Launch recognize pick app

::

  ros2 run recognize_pick recognize_pick

- Launch grasp planner

::

  ros2 run grasp_ros2 grasp_ros2 __params:=src/ros2_grasp_library/grasp_apps/recognize_pick/cfg/recognize_pick.yaml

