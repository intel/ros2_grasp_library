# Intel® DLDT toolkit and Intel® OpenVINO™ toolkit

This tutorial introduces the DLDT toolkit and OpenVINO toolkit.

Intel® [DLDT](https://github.com/opencv/dldt) is a Deep Learning Deployment Toolkit common to all architectures. The toolkit allows developers to convert pre-trained deep learning models into optimized Intermediate Representation (IR) models, then deploy the IR models through a high-level C++ Inference Engine API integrated with application logic. Additionally, [Open Model Zoo](https://github.com/opencv/open_model_zoo) provides more than 100 pre-trained optimized deep learning models and a set of demos to expedite development of high-performance deep learning inference applications. Online tutorials are availble for
* [Inference Engine Build Instructions](https://github.com/opencv/dldt/blob/2019/inference-engine/README.md)

Intel® [OpenVINO™](https://software.intel.com/en-us/openvino-toolkit) (Open Visual Inference & Neural Network Optimization) toolkit enables CNN-based deep learning inference at the edge computation, extends workloads across Intel® hardware (including accelerators) and maximizes performance. The toolkit supports heterogeneous execution across various compution vision devices -- CPU, GPU, Intel® Movidius™ NCS, and FPGA -- using a common API. Online tutorials are available for
* [Model Optimize Developer Guide](https://software.intel.com/en-us/articles/OpenVINO-ModelOptimizer)
* [Inference Engine Developer Guide](https://software.intel.com/en-us/articles/OpenVINO-InferEngine)
* [Intel® Neural Compute Stick 2](https://software.intel.com/en-us/neural-compute-stick/get-started)


## Install DLDT and OpenVINO
It's recommended to refer to the online documents of the toolkits for the latest installation instruction. Below is detailed steps we verified with Ubuntu 18.04 on Intel NUC6i7KYK for your ref.
1. Build and install Inference Engine
   ```bash
   git clone https://github.com/opencv/dldt.git
   git checkout 2019_R3
   # follow the instructions below to install all dependents, including mklml, opencl, etc.
   # https://github.com/opencv/dldt/blob/2019_R3/inference-engine/README.md#build-on-linux-systems
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
4. Optionally install plug-ins for InferenceEngine deployment on heterogeneous devices
   * Install [plug-in](https://github.com/opencv/dldt/blob/2019_R3/inference-engine/README.md#optional-additional-installation-steps-for-the-intel-movidius-neural-compute-stick-and-neural-compute-stick-2) for deployment on Intel Movidius Neural Computation Sticks Myriad X.
