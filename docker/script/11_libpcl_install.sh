#!/bin/bash 

set -e

DEPS_DIR=${DEPS_PATH}
pcl_version=https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
SUDO=$1
if [ "$SUDO" == "sudo" ];then
        SUDO="sudo"
else
        SUDO=""
fi

cd $DEPS_DIR

$SUDO apt-get install -y libhdf5-dev python3-h5py python3-pip
wget -t 3 -c  $pcl_version
tar -xvf pcl-1.8.1.tar.gz
cd pcl-pcl-1.8.1 &&mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
$SUDO make install
