#!/bin/bash

DEPS_DIR=${DEPS_PATH}
opencv_version=$1
if [ "$opencv_version" == "" ];then
	opencv_version=3.4.2
fi

#install opencv
cd $DEPS_DIR
apt-get update && apt-get install -y build-essential \
        libgtk2.0-dev \
        pkg-config \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        python-dev \
        python-numpy \
        libtbb2 \
        libtbb-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libdc1394-22-dev
git clone --depth 1 https://github.com/opencv/opencv.git -b $opencv_version
git clone --depth 1 https://github.com/opencv/opencv_contrib.git -b $opencv_version
cd $DEPS_DIR/opencv
git clone --depth 1 https://github.com/opencv/opencv_3rdparty -b ippicv/master_20180518 .cache/ippicv_download
mkdir -p .cache/ippicv &&cp -rvf .cache/ippicv_download/ippicv/ippicv_2017u3_lnx_intel64_general_20180518.tgz .cache/ippicv/b7cc351267db2d34b9efa1cd22ff0572-ippicv_2017u3_lnx_intel64_general_20180518.tgz
mkdir -p build && cd build
cd $DEPS_DIR/opencv/build
cmake -DOPENCV_EXTRA_MODULES_PATH=$DEPS_DIR/opencv_contrib/modules \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_opencv_cnn_3dobj=OFF \
    -DBUILD_LIST="core,imgproc,plot,tracking,highgui,videoio,aruco" \
    ..
make -j12
make install
ldconfig
