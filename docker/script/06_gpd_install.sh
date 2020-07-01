#!/bin/bash

# install gpd
DEPS_DIR=${DEPS_PATH}
cd $DEPS_DIR
git clone --depth 1 https://github.com/sharronliu/gpd.git -b libgpd
cd gpd/src/gpd
mkdir -p build && cd build
cmake -DUSE_OPENVINO=ON .. && make
make install
