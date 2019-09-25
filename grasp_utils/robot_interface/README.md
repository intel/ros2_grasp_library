# robot_interface

ROS2 package to use robot native interface

## Install

Install dependency **ur_modern_driver**:

```shell
git clone -b libur_modern_driver https://github.com/RoboticsYY/ur_modern_driver.git
cd libur_modern_driver
mkdir build && cd build
cmake .. && sudo make install
```

Install **robot_interface**:

```shell
mkdir -p ~/ws_ros2/src && cd ~/ws_ros2/src
git clone https://github.com/RoboticsYY/robot_interface.git
cd .. && colcon build
```

## Launch

```shell
ros2 run robot_interface ur_test __params:=`ros2 pkg prefix robot_interface`/share/robot_interface/launch/ur_test.yaml
```
