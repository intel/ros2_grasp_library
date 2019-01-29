Intel® Distribution of OpenVINO™ (Open Visual Inference & Neural Network Optimization) toolkit, based on convolutional neural networks (CNN), extends workloads across Intel® hardware (including accelerators) and maximizes performance. The toolkit enables CNN-based deep learning inference at the edge computation, and supports heterogeneous execution across various compution vision devices -- CPU, GPU, Intel® Movidius™ NCS, and FPGA -- using a **common** API.

The toolkit is available from open source project [Intel OpenVINO Toolkit](https://github.com/opencv/dldt). Refer to the installation guides to build and install Inference Engine for [Linux](https://github.com/opencv/dldt/blob/2018/inference-engine/README.md#build-on-linux-systems). Below is detailed steps for your ref.
1. Build and install Inference Engine
   ```bash
   git clone https://github.com/opencv/dldt.git
   cd dldt/inference-engine
   git submodule init
   git submodule update --recursive
   # install common dependencies
   source ./install_dependencies.sh
   # install mkl for cpu acceleration
   wget https://github.com/intel/mkl-dnn/releases/download/v0.17/mklml_lnx_2019.0.1.20180928.tgz
   tar -zxvf mklml_lnx_2019.0.1.20180928.tgz
   sudo ln -s `pwd`/mklml_lnx_2019.0.1.20180928 /usr/local/lib/mklml
   # install opencl for gpu acceleration
   wget https://github.com/intel/compute-runtime/releases/download/18.28.11080/intel-opencl_18.28.11080_amd64.deb
   sudo dpkg -i intel-opencl_18.28.11080_amd64.deb
   # build
   mkdir build && cd build
   cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DGEMM=MKL -DMKLROOT=/usr/local/lib/mklml -DENABLE_MKL_DNN=ON -DENABLE_CLDNN=ON ..
   make -j8
   sudo make install
   ```
2. Share the CMake configures for Inference Engine to be found by other packages
   ```bash
   sudo mkdir /usr/share/InferenceEngine
   sudo cp InferenceEngineConfig*.cmake /usr/share/InferenceEngine
   sudo cp targets.cmake /usr/share/InferenceEngine
   ```
   Then Inference Engine will be found when adding "find_package(InferenceEngine)" into the CMakeLists.txt
3. Configure library path for dynamic loading
   ```bash
   echo `pwd`/../bin/intel64/Release/lib | sudo tee -a /etc/ld.so.conf.d/openvino.conf
   sudo ldconfig
   ```
