#!/bin/bash

DEPS_DIR=${DEPS_PATH}
MKL_URL=https://github.com/intel/mkl-dnn/releases/download/v0.19/mklml_lnx_2019.0.5.20190502.tgz
MKL_VERSION=mklml_lnx_2019.0.5.20190502
OPENVINO_VERSION=2019_R3.1

SUDO=$1
if [ "$SUDO" == "" ];then
        SUDO="sudo"
fi

# install mkl 2019.0.5.20190502
$SUDO apt-get update && $SUDO apt-get install -y wget
cd $DEPS_DIR
wget -t 3 -c ${MKL_URL} &&\
  tar -xvf ${MKL_VERSION}.tgz &&\
  cd ${MKL_VERSION} &&\
  $SUDO mkdir -p /usr/local/lib/mklml &&\
  $SUDO cp -rf ./lib /usr/local/lib/mklml &&\
  $SUDO cp -rf ./include /usr/local/lib/mklml &&\
  $SUDO touch /usr/local/lib/mklml/version.info

#install opencl 19.41.14441
cd $DEPS_DIR
mkdir -p opencl && cd opencl &&\
  wget -t 3 -c https://github.com/intel/compute-runtime/releases/download/19.41.14441/intel-gmmlib_19.3.2_amd64.deb &&\
  wget -t 3 -c https://github.com/intel/compute-runtime/releases/download/19.41.14441/intel-igc-core_1.0.2597_amd64.deb &&\
  wget -t 3 -c https://github.com/intel/compute-runtime/releases/download/19.41.14441/intel-igc-opencl_1.0.2597_amd64.deb &&\
  wget -t 3 -c https://github.com/intel/compute-runtime/releases/download/19.41.14441/intel-opencl_19.41.14441_amd64.deb &&\
  wget -t 3 -c https://github.com/intel/compute-runtime/releases/download/19.41.14441/intel-ocloc_19.41.14441_amd64.deb &&\
  $SUDO dpkg -i *.deb

#install cmake 3.11
if [ $(cmake --version|grep "version"|awk '{print $3}') != "3.14.3"  ];then
  cd $DEPS_DIR
  wget -t 3 -c https://www.cmake.org/files/v3.14/cmake-3.14.3.tar.gz && \
    tar xf cmake-3.14.3.tar.gz && \
    (cd cmake-3.14.3 && ./bootstrap --parallel=$(nproc --all) && make --jobs=$(nproc --all) && $SUDO make install) && \
    rm -rf cmake-3.14.3 cmake-3.14.3.tar.gz
fi
#install openvino 2019_R3.1
cd $DEPS_DIR
$SUDO apt-get update && $SUDO apt-get install -y git
git clone --depth 1 https://github.com/openvinotoolkit/openvino -b ${OPENVINO_VERSION}
cd $DEPS_DIR/openvino/inference-engine
git submodule update --init --recursive &&\
  chmod +x install_dependencies.sh &&\
  $SUDO ./install_dependencies.sh
mkdir -p build && cd build &&\
  cmake -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DGEMM=MKL -DMKLROOT=/usr/local/lib/mklml \
  -DTHREADING=OMP \
  -DENABLE_MKL_DNN=ON \
  -DENABLE_CLDNN=ON \
  -DENABLE_OPENCV=OFF \
  ..
cd $DEPS_DIR/openvino/inference-engine/build
make -j8

cd $DEPS_DIR/openvino/inference-engine/build
$SUDO mkdir -p /usr/share/InferenceEngine &&\
  $SUDO cp InferenceEngineConfig*.cmake /usr/share/InferenceEngine &&\
  $SUDO cp targets.cmake /usr/share/InferenceEngine &&\
  echo `pwd`/../bin/intel64/Release/lib | $SUDO tee -a /etc/ld.so.conf.d/openvino.conf &&\
  $SUDO ldconfig
