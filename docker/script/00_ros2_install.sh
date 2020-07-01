#!/bin/bash

DEPS_DIR=${DEPS_PATH}
ros2_version=$1
if [ "$ros2_version" == "" ];then
	ros2_version=dashing
fi

# fix popup caused by libssl
apt-get install -y debconf-utils \
    echo 'libssl1.0.0:amd64 libraries/restart-without-asking boolean true' | debconf-set-selections
    
# Authorize gpg key with apt
apt-get update && apt-get install -y curl gnupg2 lsb-release &&\
    curl http://repo.ros2.org/repos.key | apt-key add -

# Add the repository to sources list
sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install development tools and ROS tools
apt-get update && apt-get install -y \
    python-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions
    
# Install ROS 2 packages
echo "install $ros2_version"
apt-get update && apt-get install -y ros-${ros2_version}-desktop
