ROS2 Grasp Library APIs
=======================

.. _GraspPlanning: http://docs.ros.org/api/moveit_msgs/html/srv/GraspPlanning.html
.. _GPD: https://github.com/atenpas/gpd
.. _OpenVINO™: https://software.intel.com/en-us/openvino-toolkit
.. _Grasp: http://docs.ros.org/api/moveit_msgs/html/msg/Grasp.html
.. _PointCloud2: https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg
.. _ObjectsInMasks: https://github.com/intel/ros2_openvino_toolkit/blob/master/people_msgs/msg/ObjectsInMasks.msg
.. _Image: https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg
.. _TransformStamped: https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/TransformStamped.msg

Grasp Planning ROS2 Interfaces
------------------------------

- Subscribed Topics

  - PointCloud2 topic from RGBD sensor (sensor_msgs::msg::`PointCloud2`_)

  - Segmented object topic (people_msgs::msg::`ObjectsInMasks`_)

- Delivered Services

  - plan_grasps (moveit_msgs::srv::`GraspPlanning`_)

Hand-Eye Calibration ROS2 Interfaces
------------------------------------

- Subscribed Topics

  - RGB image from sensor (sensor_msgs::msg::`Image`_)

- Broadcasted Transforms

  - Static transform btw camera and robot (geometry_msgs::msg::`TransformStamped`_)

Robot Interface API
-------------------

- `API <../../../grasp_utils/robot_interface/build/html/index.html>`_
