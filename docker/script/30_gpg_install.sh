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
git clone --depth 1  https://github.com/atenpas/gpg.git
cd gpg
mkdir -p build && cd build
cmake .. && make
$SUDO make install
ls /usr/local/lib/libgrasp_candidates_generator.so

