Welcome to ROS2 Grasp Library Tutorial
==========================================

.. _MoveIt: http://moveit.ros.org
.. _robot models: https://moveit.ros.org/robots

ROS2 Grasp Library enables state-of-the-art CNN based deep learning grasp detection algorithms on ROS2 for visual manipulation of industrial robot. This package provide ROS2 interfaces compliant with the `MoveIt`_ motion planning framework which is supported by most of the `robot models`_ in ROS industrial. 

Build This Tutorial
-------------------

::

  cd grasp_tutorials
  sphinx-build . build

Contents:
---------
.. toctree::
   :maxdepth: 2

   doc/overview
   doc/api
   doc/template
   doc/random_pick
   doc/recognize_pick
