#!/bin/bash

DEPS_DIR=${DEPS_PATH}
librealsense_version=$1
if [ "$librealsense_version" == "" ];then
	librealsense_version=2.34.0-0~realsense0.2251
fi

# install librealsense v2.34.0-0~realsense0.2251
echo "install librealsense 2.34.0-0~realsense0.2251"
cd $DEPS_DIR
if [ $http_proxy == "" ];then
	apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
else
	apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --keyserver-options http-proxy=$http_proxy --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
fi
sh -c 'echo "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo `lsb_release -cs` main" > /etc/apt/sources.list.d/librealsense.list'
apt-get update && apt-get install -y librealsense2=${librealsense_version} \
       	librealsense2-dev=${librealsense_version} \
       	librealsense2-udev-rules=${librealsense_version}
