#!/bin/bash

DEPS_DIR=${DEPS_PATH}
MKL_URL=https://github.com/intel/mkl-dnn/releases/download/v0.19/mklml_lnx_2019.0.5.20190502.tgz
MKL_VERSION=mklml_lnx_2019.0.5.20190502
OPENVINO_VERSION=2019_R3.1

# install mkl 2019.0.5.20190502
apt-get update && apt-get install -y wget
cd $DEPS_DIR
wget ${MKL_URL} &&\
  tar -xvf ${MKL_VERSION}.tgz &&\
  cd ${MKL_VERSION} &&\
  mkdir -p /usr/local/lib/mklml &&\
  cp -rf ./lib /usr/local/lib/mklml &&\
  cp -rf ./include /usr/local/lib/mklml &&\
  touch /usr/local/lib/mklml/version.info

#install opencl 19.41.14441
cd $DEPS_DIR
mkdir -p opencl && cd opencl &&\
  wget https://github.com/intel/compute-runtime/releases/download/19.41.14441/intel-gmmlib_19.3.2_amd64.deb &&\
  wget https://github.com/intel/compute-runtime/releases/download/19.41.14441/intel-igc-core_1.0.2597_amd64.deb &&\
  wget https://github.com/intel/compute-runtime/releases/download/19.41.14441/intel-igc-opencl_1.0.2597_amd64.deb &&\
  wget https://github.com/intel/compute-runtime/releases/download/19.41.14441/intel-opencl_19.41.14441_amd64.deb &&\
  wget https://github.com/intel/compute-runtime/releases/download/19.41.14441/intel-ocloc_19.41.14441_amd64.deb &&\
  dpkg -i *.deb

#install cmake 3.11
cd $DEPS_DIR
wget https://www.cmake.org/files/v3.14/cmake-3.14.3.tar.gz && \
  tar xf cmake-3.14.3.tar.gz && \
  (cd cmake-3.14.3 && ./bootstrap --parallel=$(nproc --all) && make --jobs=$(nproc --all) && make install) && \
  rm -rf cmake-3.14.3 cmake-3.14.3.tar.gz

#install openvino 2019_R3.1
cd $DEPS_DIR
apt-get update && apt-get install -y git
git clone --depth 1 https://github.com/openvinotoolkit/openvino -b ${OPENVINO_VERSION}
cd $DEPS_DIR/openvino/inference-engine
git submodule update --init --recursive &&\
  chmod +x install_dependencies.sh &&\
  ./install_dependencies.sh
mkdir -p build && cd build &&\
  cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DGEMM=MKL -DMKLROOT=/usr/local/lib/mklml -DENABLE_MKL_DNN=ON -DENABLE_CLDNN=ON ..
cd $DEPS_DIR/openvino/inference-engine/build
make -j8

cd $DEPS_DIR/openvino/inference-engine/build
mkdir -p /usr/share/InferenceEngine &&\
  cp InferenceEngineConfig*.cmake /usr/share/InferenceEngine &&\
  cp targets.cmake /usr/share/InferenceEngine &&\
  echo `pwd`/../bin/intel64/Release/lib | sudo tee -a /etc/ld.so.conf.d/openvino.conf &&\
  ldconfig

