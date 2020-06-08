#!/bin/bash

DEPS_DIR=${DEPS_PATH}
LIBPCL_VERSION=$1
if [ "$LIBPCL_VERSION" == "" ];then
	LIBPCL_VERSION="pcl-1.9.1"
fi

cd $DEPS_DIR
git clone --depth 1 https://github.com/PointCloudLibrary/pcl.git -b $LIBPCL_VERSION pcl-trunk
cd pcl-trunk && mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
make -j4 install
