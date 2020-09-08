#!/bin/bash

DEPS_DIR=${DEPS_PATH}
SUDO=$1
if [ "$SUDO" == "sudo" ];then
        SUDO="sudo"
else
        SUDO=""
fi

# install gpg
cd $DEPS_DIR
wget -t 3 -c https://github.com/atenpas/gpg/archive/3dcd656d70f095ad1bda3d2fb597a994198466ab.zip
unzip 3dcd656d70f095ad1bda3d2fb597a994198466ab.zip 
cd gpg-3dcd656d70f095ad1bda3d2fb597a994198466ab
mkdir -p build && cd build
cmake .. && make
$SUDO make install
ls /usr/local/lib/libgrasp_candidates_generator.so
