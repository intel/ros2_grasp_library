#!/bin/bash

set -e

deps_path=$1
if [ -z "$deps_path" ]; then
  echo -e "warring:\n    install_ros2_grasp_library_deps.sh <your-install-deps-path>"
  exit 0
fi

# mkdir deps-path
echo "DEPS_PATH = $deps_path"
mkdir -p $deps_path
export DEPS_PATH=$deps_path

CURRENT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
echo "CURRENT_DIR = ${CURRENT_DIR}"

# install ros2 dashing
bash ${CURRENT_DIR}/00_ros2_install.sh

# install openvino(DLDT) 2019_R3.1
bash ${CURRENT_DIR}/01_openvino_install.sh

# install opencv 3.4.2
bash ${CURRENT_DIR}/02_opencv_install.sh

# install librealsense 2.34
bash ${CURRENT_DIR}/03_librealsense_install.sh

# install libpcl 1.9.0
bash ${CURRENT_DIR}/04_libpcl_install.sh

# install robot-interface
bash ${CURRENT_DIR}/05_gpg_install.sh
bash ${CURRENT_DIR}/06_gpd_install.sh
bash ${CURRENT_DIR}/07_ur_modern_driver_install.sh

# install ros2 build deps
apt-get install -y ros-dashing-object-msgs \
	python3-scipy \
	ros-dashing-eigen3-cmake-module

# build ros2 other deps
bash ${CURRENT_DIR}/08_ros2_other_deps.sh
