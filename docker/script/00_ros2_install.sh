#!/bin/bash

DEPS_DIR=${DEPS_PATH}
ros2_version=dashing
SUDO=$1
if [ "$SUDO" == "sudo" ];then
	SUDO="sudo"
else
	SUDO=""
fi

# fix popup caused by libssl
$SUDO apt-get install -y debconf-utils \
    echo 'libssl1.0.0:amd64 libraries/restart-without-asking boolean true' | $SUDO debconf-set-selections
    
# Authorize gpg key with apt
$SUDO apt-get update && $SUDO apt-get install -y curl gnupg2 lsb-release &&\
    curl http://repo.ros2.org/repos.key | $SUDO apt-key add -

# Add the repository to sources list
$SUDO sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install development tools and ROS tools
$SUDO apt-get update && $SUDO apt-get install -y \
    python-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions
    
# Install ROS 2 packages
echo "install $ros2_version"
$SUDO apt-get update && $SUDO apt-get install -y ros-${ros2_version}-desktop
