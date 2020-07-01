#!/bin/bash

WORK_DIR=${DEPS_PATH}/../ros2_overlay_ws

mkdir -p $WORK_DIR/src &&cd $WORK_DIR/src

git clone --depth 1 https://github.com/RoboticsYY/ros2_ur_description.git
git clone --depth 1 https://github.com/RoboticsYY/handeye
git clone --depth 1 https://github.com/RoboticsYY/criutils.git
git clone --depth 1 https://github.com/RoboticsYY/baldor.git

cd $WORK_DIR
source /opt/ros/dashing/setup.sh
colcon build --symlink-install

