# robot_interface

ROS2 package to use robot native interface

## Install

Install dependency **ur_modern_driver**:

```shell
git clone -b libur_modern_driver https://github.com/RoboticsYY/ur_modern_driver.git
cd ur_modern_driver/libur_modern_driver
mkdir build && cd build
cmake .. && sudo make install
```

Install dependency **ros2_ur_description**:

```shell
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/RoboticsYY/ros2_ur_description.git
cd .. && colcon build
```

Install **robot_interface**:

The installation should refer to the installation of **ros2_grasp_library**.

> Note: If error "fatal error: Eigen/Geometry: No such file or directory" persists during the compilation, please check the file path of "Eigen/Geometry", if it locates at "/usr/local/include/eigen3/Eigen/Geometry", a work around is creating a soft link to "/usr/local/include/Eigen/Geometry".

## Launch

Launch the UR robot control test executable:

```shell
ros2 launch robot_interface ur_test.launch.py
```

Launch the Rivz2 display:

```shell
ros2 launch ur_description view_ur5_ros2.launch.py
```

## Generate Document

```shell
cd <path to root of ros2_grasp_library>/grasp_utils/robot_interface

doxygen Doxyfile
```
