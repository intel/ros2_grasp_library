Overview
========

.. _GraspPlanning: http://docs.ros.org/api/moveit_msgs/html/srv/GraspPlanning.html
.. _GPD: https://github.com/atenpas/gpd
.. _OpenVINO™: https://software.intel.com/en-us/openvino-toolkit
.. _Grasp: http://docs.ros.org/api/moveit_msgs/html/msg/Grasp.html

ROS2 Grasp Library consists of

.. image:: ../_static/images/ros2_grasp_library.png
   :width: 523 px
   :height: 311 px
   :align: center

- A ROS2 Grasp Planner providing grasp planning service, as an extensible capability of MoveIt (moveit_msgs::srv::`GraspPlanning`_), translating grasp detection results into MoveIt Interfaces (moveit_msgs::msg::`Grasp`_). A ROS2 Grasp Detctor abstracting interfaces for grasp detection results

- A ROS2 hand-eye calibration module generating transformation from camera frame to robot frame

- Robot interfaces controlling the phsical robot to move, pick, place, as well as to feedback robot states

- ROS2 example applications demonstrating how to use this ROS2 Grasp Library in advanced industrial usages for intelligent visual grasp
