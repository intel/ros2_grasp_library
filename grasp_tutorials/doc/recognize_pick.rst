Recognize Pick (OpenVINO Grasp Detection + OpenVINO Object Segmentation)
========================================================================

Overview
--------

A simple application demonstrating how to pick up recognized objects with an industrial robot arm.
The application interact with Grasp Planner and Robot Interface from this Grasp Library.

Comparing against the `random picking <random_pick.html>`_ application, this recognition picking takes the place commands published from the `place_publisher` which specifying the name the object to pick and the position to place.

The Grasp Detector then takes the object segmentation results from the `OpenVINO Mask-rcnn <https://github.com/intel/ros2_openvino_toolkit>`_ to identify the location of the object in the point cloud image and generates grasp poses for that specific object.

Watch this `demo_video <https://www.youtube.com/embed/trIt0uKRXBs?rel=0>`_ to see the output of this application.

.. raw:: html

  <iframe width="700" height="389" src="https://www.youtube.com/embed/trIt0uKRXBs?list=PLxCmGJeiLgoxq3uqcCVSYnSJ9iQk1L9yP" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

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

  - `Grasp Planner <grasp_planner.html>`_

  - `Robot Interface <robot_interface.html>`_

  - `Hand-Eye Calibration <handeye_calibration.html>`_

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

- Publish handeye transform, refer to `Hand-Eye Calibration`_

- Publish place object

::

  ros2 run recognize_pick place_publisher sports_ball

- Launch UR description

::

  ros2 launch ur_description view_ur5_ros2.launch.py

  #load rviz2 configure file "src/ros2_grasp_library/grasp_apps/recognize_pick/rviz2/recognize_pick.rviz"

- Launch RGBD sensor

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

  ros2 run grasp_ros2 grasp_ros2 __params:=src/ros2_grasp_library/grasp_ros2/cfg/recognize_pick.yaml

