Overview
========

.. _GraspPlanning: http://docs.ros.org/api/moveit_msgs/html/srv/GraspPlanning.html
.. _GPD: https://github.com/atenpas/gpd
.. _OpenVINO™: https://software.intel.com/en-us/openvino-toolkit
.. _Grasp: http://docs.ros.org/api/moveit_msgs/html/msg/Grasp.html

ROS2 Grasp Library consists of

- A ROS2 Grasp Planner providing grasp planning service, as an extensible capability of MoveIt (moveit_msgs::srv::`GraspPlanning`_)

- A ROS2 Grasp Detector interface, enabled with the specific back-end algorithm `GPD`_ with Intel® `OpenVINO™`_ technology

- Hand-eye calibration generating transformation from camera frame to a specified target frame; Grasp translation to the MoveIt Interfaces (moveit_msgs::msg::`Grasp`_)

- Robot interfaces controlling the phsical robot to move, pick, place, as well as to feedback robot states

- Grasp applications demonstrating the grasp planning capabilities, and telling how to put all the above software components together for intelligent visual manipulation tasks

.. image:: ../_static/images/ros2_grasp_library.png
   :width: 523 px
   :height: 311 px
   :align: center
