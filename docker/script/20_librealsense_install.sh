#!/bin/bash

DEPS_DIR=${DEPS_PATH}
librealsense_version=2.31.0-0~realsense0.1791
SUDO=$1
if [ "$SUDO" == "sudo" ];then
        SUDO="sudo"
else
        SUDO=""
fi

# install librealsense v2.34.0-0~realsense0.2251
echo "install librealsense 2.34.0-0~realsense0.2251"
cd $DEPS_DIR
if [ $http_proxy == "" ];then
	$SUDO apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
else
	$SUDO apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --keyserver-options http-proxy=$http_proxy --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
fi
$SUDO sh -c 'echo "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo `lsb_release -cs` main" > /etc/apt/sources.list.d/librealsense.list'
$SUDO apt-get update && $SUDO apt-get install -y librealsense2=${librealsense_version} \
       	librealsense2-dev=${librealsense_version} \
       	librealsense2-udev-rules=${librealsense_version} \
	librealsense2-gl=${librealsense_version} \
        librealsense2-utils=${librealsense_version} \
	librealsense2-dbg=${librealsense_version} \
	librealsense2-dkms
