Welcome to ROS2 Grasp Library Tutorial
======================================

This is a ROS2 intelligent visual grasp solution for advanced industrial usages, with OpenVINO™ grasp detection and MoveIt Grasp Planning.

Build and test this tutorial
----------------------------

::

  cd ros2_grasp_library/grasp_tutorials

  sphinx-build . build # check the outputs in the ./build/ folder

  cd ros2_grasp_library/grasp_utils/robot_interface

  doxygen Doxyfile # check the outputs in the ./build/ folder

Contents:
---------
.. toctree::
   :maxdepth: 2

   doc/overview

   doc/getting_start

   doc/grasp_planner

   doc/robot_interface

   doc/bringup_robot

   doc/handeye_calibration

   doc/random_pick

   doc/recognize_pick

   doc/grasp_api

   doc/template
