Draw X Version 1.0
==================

Overview
--------------

A simple application demonstrating how to draw letter ``X``
at fixed positions with an industrial robot arm.
The motion of robot arm is planned by MoveIt!. You can find the way
to customize the corner of ``X`` in `Customization Notes`_.

Requirement
------------

Before running the code, make sure you have
followed the instructions below to setup the robot to work with MoveIt!
The setup includes installing necessary robot urdf files,
their gazebo configurations files and ROS driver for the robot control.

- Hardware

  - Host running ROS2/ROS

  - `UR5`_ (optional)

  - `Franka Panda`_ (optional)

- Software

  - `ROS Melodic`_ Desktop-Full

  - `MoveIt`_

  - `Setup UR5 With MoveIt`_

  - `Setup Franka With MoveIt`_

.. _UR5: https://www.universal-robots.com/products/ur5-robot

.. _Franka Panda: https://www.franka.de/panda/

.. _ROS Melodic: http://wiki.ros.org/melodic/Installation/Ubuntu

.. _MoveIt: https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#install-moveit)

.. _Setup UR5 With MoveIt: https://github.intel.com/pages/otc-rse/moveit_app_zoo/doc/ur5_setup_with_moveit.html

.. _Setup Franka With MoveIt: https://github.intel.com/pages/otc-rse/moveit_app_zoo/doc/franka_setup_with_moveit.html

Download and Build the Example Code
------------------------------------

Within your catkin workspace, download and compile the example code:

::

  cd <catkin_workspace>/src

  git clone https://github.intel.com/otc-rse/moveit_app_zoo.git

  cd ..

  catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release -BUILD_DRAW_X=ON

  catkin build

- Build Options

  - BUILD_DRAW_X (ON | **OFF** ) Switch on/off building of this application


Launch the Application
----------------------

- `Launch the robots`_

  .. toctree::
    :maxdepth: 1

    ./launch_robots

.. _Launch the robots: https://github.intel.com/pages/otc-rse/moveit_app_zoo/doc/launch_robots.html

- Launch the application

::

  roslaunch draw_x draw_x.launch ur5:=true

.. note:: The default launch options are for UR5 robot. For other robots like Franka
          panda, the necessary launch options in the below list should also be enabled.

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

  - Option: ``arm_base``, frame the four corners of letter X with respect to:

      * Tpye: String
      * Default: "base"
      * Value: "base" is used for UR5 robot, "panda_link0"
        is used for Franka robot.

  - Option: ``group_name``, name of joints group for the motion planning task:

      * Tpye: String
      * Default: "ur5_arm"
      * Value: "ur5_arm" is used for UR5 robot, "panda_arm"
        is used for Franka robot.

  - Option: ``eef_parent``, name of the parent link of end-effector:

      * Tpye: String
      * Default: "tool0"
      * Value: "tool0" is used for UR5 robot, "panda_link8" is used
        for Franka robot.

  - Option: ``debug``, control flag about if run the demo continuously
    or in separate steps.

      * Tpye: Bool
      * Default: false
      * Value: True, if you want to see the motion splitted into small steps.
        For each step, click ``Next`` button
        of **RvizVisualToolsGui** to continue.

Expected Outputs
----------------

You should be able to see the robot execute the following motion in Rviz:

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

At the end of the demo, you should see Rviz ouput like this:

.. image:: ../_static/images/draw_x_v_1.png

Customization Notes
-------------------

- How to Change the Corners of X

You can change the coordinate values of
the four corners of letter X in the ``draw_x.launch``:

::

  <!-- The positions of four corners of `X` w.r.t the `arm_base` frame -->
  <arg name="corner1_x" value="0.1" if="$(arg ur5)"/>
  <arg name="corner1_y" value="-0.65" if="$(arg ur5)"/>
  <arg name="corner1_z" value="0.15" if="$(arg ur5)"/>

  <arg name="corner2_x" value="-0.1" if="$(arg ur5)"/>
  <arg name="corner2_y" value="-0.45" if="$(arg ur5)"/>
  <arg name="corner2_z" value="0.15" if="$(arg ur5)"/>

  <arg name="corner3_x" value="-0.1" if="$(arg ur5)"/>
  <arg name="corner3_y" value="-0.65" if="$(arg ur5)"/>
  <arg name="corner3_z" value="0.15" if="$(arg ur5)"/>

  <arg name="corner4_x" value="0.1" if="$(arg ur5)"/>
  <arg name="corner4_y" value="-0.45" if="$(arg ur5)"/>
  <arg name="corner4_z" value="0.15" if="$(arg ur5)"/>

.. note:: They are all with respect to the robot base frame, e.g. ``base_link`` of UR5 and ``panda_link0`` of Franka Panda.
          Chnage the corner position may result in that the robot could not find Inverse Kinematics solutions.

- `Geometry Customization`_

.. _Geometry Customization: https://github.intel.com/pages/otc-rse/moveit_app_zoo/doc/table_platform_fence_customization.html
