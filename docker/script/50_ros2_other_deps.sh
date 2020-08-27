#!/bin/bash

SUDO=$1
if [ "$SUDO" == "sudo" ];then
        SUDO="sudo"
else
        SUDO=""
fi

$SUDO apt-get install -y ros-dashing-object-msgs \
        python3-scipy \
        ros-dashing-eigen3-cmake-module

WORK_DIR=${DEPS_PATH}/../ros2_ws
mkdir -p $WORK_DIR/src &&cd $WORK_DIR/src

git clone --depth 1 https://github.com/RoboticsYY/ros2_ur_description.git
git clone --depth 1 https://github.com/RoboticsYY/handeye
git clone --depth 1 https://github.com/RoboticsYY/criutils.git
git clone --depth 1 https://github.com/RoboticsYY/baldor.git
git clone --depth 1 https://github.com/intel/ros2_openvino_toolkit.git

cd $WORK_DIR
source /opt/ros/dashing/setup.sh
export InferenceEngine_DIR=/opt/openvino_toolkit/openvino/inference-engine/build/
export export CPU_EXTENSION_LIB=/opt/openvino_toolkit/openvino/inference-engine/bin/intel64/Release/lib/libcpu_extension.so
export GFLAGS_LIB=/opt/openvino_toolkit/openvino/inference-engine/bin/intel64/Release/lib/libgflags_nothreads.a
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$InferenceEngine_DIR/../bin/intel64/Release/lib:/usr/local/lib/mklml/lib

colcon build --symlink-install
