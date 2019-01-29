Installation guide for Grasp Pose Detection

### Install [GPG](https://github.com/atenpas/gpg)
1. Get the code
```bash
git clone https://github.com/atenpas/gpg.git
cd gpg
```
2. Build the library
```bash
mkdir build && cd build
cmake ..
make
sudo make install
# by default, "libgrasp_candidates_generator.so" shall be installed to "/usr/local/lib"
```

### Install [GPD](https://github.com/sharronliu/gpd)
1. Get the code
```bash
git clone https://github.com/sharronliu/gpd.git
git checkout libgpd
cd gpd/src/gpd
```
2. Build the library
```bash
mkdir build && cd build
cmake -DUSE_OPENVINO=ON ..
make
sudo make install
# by default, "libgrasp_pose_detection.so" shall be installed to "/usr/local/lib"
# and header files installed to "/usr/local/include/gpd"
```
