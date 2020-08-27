#!/bin/bash

set -e
ROS_PATH=$1

if [  ! -f ${ROS_PATH}/install/setup.bash ];then
        echo -e "Error: ${ROS_PATH}/install/setup.bash don't exist\n"
        echo -e "eg: entrypoint.sh ~/ros2_ws"
        exit
fi

# setup ros2 environment
source /opt/ros/dashing/setup.bash
source ${ROS_PATH}/install/setup.bash
export ROS_DOMAIN_ID=100  # robot_group_id

export InferenceEngine_DIR=/opt/openvino_toolkit/openvino/inference-engine/build/
export export CPU_EXTENSION_LIB=/opt/openvino_toolkit/openvino/inference-engine/bin/intel64/Release/lib/libcpu_extension.so
export GFLAGS_LIB=/opt/openvino_toolkit/openvino/inference-engine/bin/intel64/Release/lib/libgflags_nothreads.a
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$InferenceEngine_DIR/../bin/intel64/Release/lib:/usr/local/lib/mklml/lib


