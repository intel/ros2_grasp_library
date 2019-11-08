Bring up a New Robot
====================

This tutorial explains what is expected to do when bringing up this ROS2 Grasp Library on a new robot.

.. _GraspPlanning: http://docs.ros.org/api/moveit_msgs/html/srv/GraspPlanning.html
.. _GPD: https://github.com/atenpas/gpd
.. _OpenVINO™: https://software.intel.com/en-us/openvino-toolkit
.. _Grasp: http://docs.ros.org/api/moveit_msgs/html/msg/Grasp.html

Minimum APIs to Implement
-------------------------

- `moveToTcpPose`

  - Move the TCP (tool center point, usually the end effector of the robot arm, not the hand) to a pose specified with position [x, y, z] and orientation [alpha, betta, gamma] (also called [roll, pitch, yaw]). This function returns when the robot `moved` to the specified pose.

- `moveToJointValues`

  - Move each joint of the robot to the specified values (usually angles). This function differs from the `moveToTcpPose` since a same TCP pose may be reached with various solutions of joint values. This function is used when the application expect the joints of the robot in specific state, that is proper to performe any successive picking or place action. This function returns when the robot `moved` to the specified joint values.

- `open`

  - Open the gripper.

- `close`

  - Close the gripper.

- `startLoop`

  - Start a loop to read and publish the robot state. Robot states are subsribed by Rviz for visualization.

Optional implementation and possible extentions
-----------------------------------------------

- Optionally you may implement the `pick` and `place` interface to customize the pick and placed pipeline, or even plug-in the collision avoidance motion planning.

- Python extention is not supported. It's possible to implement the Robot Interface in python and bind to C++.


Refer to `Robot Interface API <../api/html/index.html>`_ for more detailed definition.

Example UR5 Implementation
--------------------------

Refer to the UR5 `example <https://github.com/intel/ros2_grasp_library/tree/master/grasp_utils/robot_interface/src>`_ implementatino for Robot Interface.

Test Your Implementation
------------------------

It's important to test your implementation before integrating this part with other components in ROS2 Grasp Library.

Refer to `UR5 tests <https://github.com/intel/ros2_grasp_library/blob/master/grasp_utils/robot_interface/test/ur_test_move_command.cpp>`_, adapt it to your robot tests.

Bring up Robot Control Applications
-----------------------------------

Once finished the testing, you may start to bring up the `Draw X <draw_x.html>`_ app or the `fixed position pick and place <fixed_position_pick.html>`_ app on your new robot. These application does not require camera, instead they control the robot only.
