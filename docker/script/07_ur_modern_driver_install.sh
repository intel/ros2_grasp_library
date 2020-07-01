#!/bin/bash

DEPS_DIR=${DEPS_PATH}
# install ur_modern_driver
cd $DEPS_DIR
git clone --depth 1 https://github.com/RoboticsYY/ur_modern_driver.git -b libur_modern_driver
cd ur_modern_driver/libur_modern_driver
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release .. && make
make install
