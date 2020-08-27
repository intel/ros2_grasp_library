#!/bin/bash

DEPS_DIR=${DEPS_PATH}
opencv_version=4.1.2
SUDO=$1
if [ "$SUDO" == "sudo" ];then
        SUDO="sudo"
else
        SUDO=""
fi

#install opencv
cd $DEPS_DIR
$SUDO apt-get update && $SUDO apt-get install -y build-essential \
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
mkdir -p build && cd build
cd $DEPS_DIR/opencv/build
cmake -D OPENCV_EXTRA_MODULES_PATH=$DEPS_DIR/opencv_contrib/modules \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D BUILD_EXAMPLES=ON \
    -D BUILD_opencv_xfeatures2d=OFF \
    ..
make -j4
$SUDO make install
echo "/usr/local/lib" | $SUDO tee /etc/ld.so.conf.d/opencv.conf
$SUDO ldconfig
