#!/bin/bash

set -e

deps_path=$1
if [ -z "$deps_path" ]; then
  echo -e "warring:\n    install_ros2_grasp_library_deps.sh <your-install-deps-path>"
  echo -e "If you want to use'sudo' : install_ros2_grasp_library_deps.sh <your-install-deps-path> sudo"
  exit 0
fi

shift

# mkdir deps-path
echo "DEPS_PATH = $deps_path"
mkdir -p $deps_path
export DEPS_PATH=$deps_path

CURRENT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
echo "CURRENT_DIR = ${CURRENT_DIR}"

# install ros2 dashing
bash ${CURRENT_DIR}/00_ros2_install.sh $@

# instal eigen 3.2
bash ${CURRENT_DIR}/10_eigen_install.sh $@

# install libpcl 1.8.1
bash ${CURRENT_DIR}/11_libpcl_install.sh $@

# install opencv 4.1.2 
bash ${CURRENT_DIR}/12_opencv_install.sh $@

# install openvino 2019_R3.1
bash ${CURRENT_DIR}/13_openvino_install.sh $@

# install librealsense 2.31
bash ${CURRENT_DIR}/20_librealsense_install.sh $@

# install gpg
bash ${CURRENT_DIR}/30_gpg_install.sh $@

# install gpd
bash ${CURRENT_DIR}/31_gpd_install.sh $@

# install ur_modern_driver
bash ${CURRENT_DIR}/32_ur_modern_driver_install.sh $@

# build ros2 other deps
bash ${CURRENT_DIR}/50_ros2_deps.sh $@
