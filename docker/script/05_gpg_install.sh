#!/bin/bash

DEPS_DIR=${DEPS_PATH}
# install gpg
cd $DEPS_DIR
git clone --depth 1  https://github.com/atenpas/gpg.git
cd gpg
mkdir -p build && cd build
cmake .. && make
make install
ls /usr/local/lib/libgrasp_candidates_generator.so

