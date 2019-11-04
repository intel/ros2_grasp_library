Draw X
======

Overview
--------------

This demo shows how to use the robot interface to draw letter ``X``
at the fixed positions with an UR5 robot arm.

Requirement
------------

Before running the code, make sure you have
followed the instructions below to setup the robot correctly.

- Hardware

  - Host running ROS2

  - `UR5`_

- Software

  - `ROS2 Dashing`_ Desktop

  - `robot_interface`_

.. _UR5: https://www.universal-robots.com/products/ur5-robot

.. _ROS2 Dashing: https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/

.. _robot_interface: https://github.com/intel/ros2_grasp_library/tree/master/grasp_utils/robot_interface

Download and Build the Example Code
------------------------------------

Within your ROS2 workspace, download and compile the example code:

::

  cd <path_of_your_ros2_workspace>/src

  git clone https://github.com/intel/ros2_grasp_library.git

  cd ..

  colcon build --base-paths src/ros2_grasp_library/grasp_apps/draw_x

Launch the Application
----------------------

- Launch the application

::

  ros2 launch draw_x draw_x.launch.py

.. note:: Please make sure the emergency button on the teach pendant is in your hand,
          in case there is any accident.

- Expected Outputs:

  1. The robot moves its arm to the home pose
  2. The robot moves its arm to the pose above the first corner of X
  3. The robot moves its arm down to the first corner of X
  4. The robot moves its arm to the second corner of X
  5. The robot moves its arm up to the pose above the second corner of X
  6. The robot moves its arm to the pose above the third corner of X
  7. The robot moves its arm down to the third corner of X
  8. The robot moves its arm to the fourth corner of X
  9. The robot moves its arm up to the pose above the fourth corner of X
  10. The robot moves its arm to the home pose again
