changelog for ros2_grasp_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2019-03-13)
------------------
* Support "service-driven" grasp detection mechanism (via configure auto_mode) to optimize CPU load for real-time processing.
* Support grasp transformation from camera frame to a specified target frame expected in the visual manipulation.
* Support launch option "grasp_approach" to specify expected approach direction in the target frame specified by 'grasp_frame_id'. Grasp Planner will return grasp poses with approach direction approximate to this parameter.
* Support launch option "device" to configure device for grasp pose inference to execute, 0 for CPU, 1 for GPU, 2 for VPU, 3 for FPGA. In case OpenVINO plug-ins are installed (tutorial), this configure deploy the CNN based deep learning inference on to the target device.
* Add tutorials for introduction to Intel DLDT toolkit and Intel OpenVINO toolkit.
* Add tutorials for launch options and customization notes.

0.3.0 (2018-12-28)
------------------
* Support grasp pose detection from RGBD point cloud.
* Support MoveIt! grasp planning service.
