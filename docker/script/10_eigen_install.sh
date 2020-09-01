#!/bin/bash 

set -e

DEPS_DIR=${DEPS_PATH}
eigen_version=https://gitlab.com/libeigen/eigen/-/archive/3.2/eigen-3.2.tar.gz
SUDO=$1
if [ "$SUDO" == "sudo" ];then
        SUDO="sudo"
else
        SUDO=""
fi

cd $DEPS_DIR

$SUDO apt-get install -y gfortran
wget -t 3 -c $eigen_version
tar -xvf eigen-3.2.tar.gz
cd eigen-3.2 &&mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
$SUDO make install
$SUDO rm -rf /usr/include/eigen3/
$SUDO ln -sf /usr/local/include/eigen3 /usr/include/
$SUDO make install
