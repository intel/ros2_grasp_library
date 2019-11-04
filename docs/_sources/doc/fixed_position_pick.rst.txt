Fixed Position Pick
====================

Overview
--------------

This demo shows how to use the robot interface to pick and place a
object at a predefined location with an UR5 robot arm.

Requirement
------------

Before running the code, make sure you have
followed the instructions below to setup the robot correctly.

- Hardware

  - Host running ROS2

  - `UR5`_

  - `Robot Gripper`_

- Software

  - `ROS2 Dashing`_ Desktop

  - `robot_interface`_

.. _UR5: https://www.universal-robots.com/products/ur5-robot

.. _ROS2 Dashing: https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/

.. _robot_interface: https://github.com/intel/ros2_grasp_library/tree/master/grasp_utils/robot_interface

.. _Robot Gripper: https://www.universal-robots.com/plus/end-effectors/hitbot-electric-gripper

Download and Build the Example Code
------------------------------------

Within your ROS2 workspace, download and compile the example code:

::

  cd <path_of_your_ros2_workspace>/src

  git clone https://github.com/intel/ros2_grasp_library.git

  cd ..

  colcon build --base-paths src/ros2_grasp_library/grasp_apps/fixed_position_pick

Launch the Application
----------------------

- Launch the application

::

  ros2 launch fixed_position_pick fixed_position_pick.launch.py

.. note:: Please make sure the emergency button on the teach pendant is in your hand,
          in case there is any accident.

- Expected Outputs:

  1. The robot moves to the home pose
  2. The robot picks up an object from the predefined location
  3. The robot places the object to another location
  4. The robot moves back to the home pose
