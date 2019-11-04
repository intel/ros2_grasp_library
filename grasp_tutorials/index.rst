Welcome to ROS2 Grasp Library Tutorial
==========================================

ROS2 Grasp Library enables state-of-the-art CNN based deep learning grasp detection algorithms on ROS2 for visual manipulation of industrial robot. This package provide ROS2 interfaces compliant with the `MoveIt <http://moveit.ros.org>`_ motion planning framework which is supported by most of the `robot models <https://moveit.ros.org/robots>`_ in ROS industrial.

Build This Tutorial
-------------------

::

  git checkout gh-pages

  cd grasp_tutorials

  sphinx-build . build

  cd grasp_utils/robot_interface

  doxygen Doxyfile

Contents:
---------
.. toctree::
   :maxdepth: 2

   doc/overview

   doc/robot_interface

   doc/handeye_calibration

   doc/grasp_planner

   doc/random_pick

   doc/recognize_pick

   doc/template

   doc/grasp_api
