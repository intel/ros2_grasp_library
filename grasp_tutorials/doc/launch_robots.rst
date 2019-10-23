Bring up fake controller, real controller and gripper controller
=================================================================

This application runs with either a fake controller **OR** a real controller.

.. note:: It's very important before running the application on a real robot,
          you should verify your platform setup with the "fake controller".

**Option 1**: launch a fake controller

- UR5

::

  roslaunch ur5_hitbot_ilc_platform_moveit_config demo.launch

- Franka

::

  roslaunch franka_hand_table_moveit_config demo.launch

**Option 2**: Launch a real contoller for UR5

::

  roslaunch app_control ur5_bringup.launch robot_ip:='ip address of the robot controller'

  roslaunch hitbot_control hitbot.launch # Optional if a hitbot gripper is used

  roslaunch app_control ur5_moveit_planning_execution.launch
