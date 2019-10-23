Fixed Position Pick and Place
==============================

Overview
--------------
This demo shows how to setup the arm and gripper controller
in simulation and real execution to use the pick and place interface of MoveIt!

Requirement
------------

Before running the code, make sure you have followed the instructions below
to setup the robot to work with MoveIt!.
The setup includes installing necessary robot urdf files,
their gazebo configurations files and ROS driver for the robot control.

- Hardware

  - Host running ROS2/ROS

  - `UR5`_ (optional)

  - `Franka Panda`_ (optional)

  - `Robot Gripper`_ (optional)

- Software

  - `ROS Melodic`_ Desktop-Full

  - `MoveIt`_

  - Universal Robot

    - `Setup UR5 With MoveIt`_

  - Franka Robot

    - `Setup Franka With MoveIt`_

  - Robot Gripper

    - `hitbot`_

.. _UR5: https://www.universal-robots.com/products/ur5-robot

.. _Franka Panda: https://www.franka.de/panda/

.. _Robot Gripper: https://www.universal-robots.com/plus/end-effectors/hitbot-electric-gripper

.. _ROS Melodic: http://wiki.ros.org/melodic/Installation/Ubuntu

.. _MoveIt: https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#install-moveit)

.. _Setup UR5 With MoveIt: https://github.intel.com/pages/otc-rse/moveit_app_zoo/doc/ur5_setup_with_moveit.html

.. _Setup Franka With MoveIt: https://github.intel.com/pages/otc-rse/moveit_app_zoo/doc/franka_setup_with_moveit.html

.. _hitbot: https://github.intel.com/otc-rse/hitbot

Download and Build the Example Code
------------------------------------

Within your catkin workspace, download and compile the example code:

::

  cd <catkin_workspace>/src

  git clone https://github.intel.com/otc-rse/moveit_app_zoo.git

  cd ..

  catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release -BUILD_FIXED_POSITION_PICK=ON

  catkin build

- Build Options

  - BUILD_FIXED_POSITION_PICK (ON | **OFF** )
    Switch on/off building of this application

Launch the Application
----------------------

- `Launch the robots`_

  .. toctree::
    :maxdepth: 1

    ./launch_robots

.. _Launch the robots: https://github.intel.com/pages/otc-rse/moveit_app_zoo/doc/launch_robots.html

- Launch the application

::

  roslaunch pick_place pick_place.launch ur5:=true

.. note:: For other robots other than UR5 and Franka, please refer `Customization Notes`_.

- Launch Options

  Here are some explanations to the launch options
  in the above command line code:

  - Option: ``ur5``, flag to run the demo for UR5 robot:

      * Tpye: Bool
      * Default: false
      * Value: True, enable the demo for UR5 robot.

  - Option: ``franka``, flag to run the demo for franka panda robot:

      * Tpye: Bool
      * Default: false
      * Value: True, enable the demo for franka panda robot.

.. note:: There is a know issue in MoveIt that could cause the pick_place pipeline of MoveIt fail to find
          proper inverse kinematics solutions. Then the motion planners will fail due to this.
          A temporary solution can be:
          https://github.com/ros-planning/moveit/issues/1278#issuecomment-447741229

Expected Outputs
----------------

You should be able to see the robot execute the following motion in Rviz:

1. The robot moves its arm to the home pose
2. The robot picks up a ball on a surface
3. The robot places the ball on another location of the surface

At the end of the demo, you should see Rviz ouput like this:

.. image:: ../_static/images/pick_place.png

Customization Notes
-------------------

- Change robot or gripper

Basicly, you can use any robot and gripper with the demo code.
If you want to change the robot or gripper, you have to create
another moveit config package from your robot's urdf file.
You can follow the `MoveIt Setup Assistant Tutorial`_.

.. _MoveIt Setup Assistant Tutorial: https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html

After you have done this, you can add following lines to ``pick_place.launch``
like the case for the UR5 robot and Hitbot gripper:

::

  <!-- Parameters for ur5 robot -->
  <arg name="frame_id" value="base" if="$(arg ur5)"/>
  <arg name="support_surface" value="operation_surface" if="$(arg ur5)"/>
  <arg name="group_name" value="ur5_arm" if="$(arg ur5)"/>
  <arg name="finger_joints" value="hitbot_base_finger0_joint hitbot_base_finger1_joint" if="$(arg ur5)"/>
  <arg name="finger_open_dists" value="-0.01 0.01" if="$(arg ur5)"/>
  <arg name="object_xyz" value="0.107 -0.545 -0.10" if="$(arg ur5)"/>
  <arg name="grasp_xyz" value="0.107 -0.545 0.108" if="$(arg ur5)"/>
  <arg name="grasp_rpy" value="3.1415 0.0 -0.7853" if="$(arg ur5)"/>
  <arg name="place_xyz" value="-0.107 -0.545 -0.10" if="$(arg ur5)"/>
  <arg name="place_rpy" value="0.0 0.0 0.0" if="$(arg ur5)"/>

Some descriptions to the parameters:

  * frame_id (string | ``"base"``)

      * Specify the root link frame of the robot.

  * support_surface (string | ``"operation_surface"``)

      * Specify the frame on which the object placed.

  * group_name (string | ``"ur5_arm"``)

      * Specify the joint group name
        created with the MoveIt Setup Assistant.

  * finger_joints (string |
    ``"hitbot_base_finger0_joint hitbot_base_finger1_joint"``)

      * Specify the joints name of the gripper.

  * object_xyz (double | ``"0.107 -0.545 -0.10"``)

      * Specify the object position in the "frame_id" frame

  * grasp_xyz (double | ``"0.107 -0.545 0.108"``)

      * Specify the grasp position in the "frame_id" frame

  * grasp_rpy (double | ``"3.1415 0.0 -0.7853"``)

      * Specify the grasp orientation in the "frame_id" frame

  * place_xyz (double | ``"-0.107 -0.545 -0.10"``)

      * Specify the place position in the "frame_id" frame

  * place_rpy (double | ``"0.0 0.0 0.0"``)

      * Specify the place orientation in the "frame_id" frame

- `Geometry Customization`_

.. _Geometry Customization: https://github.intel.com/pages/otc-rse/moveit_app_zoo/doc/table_platform_fence_customization.html
