# Precondition
## add docker group
```
sudo groupadd docker
sudo usermod -aG docker $USER
```
## Build docker image
```
cd ros2_grasp_library/docker
docker build -t intel/ros2:ros2_grasp_library_deps .

```
If your use proxy
```
docker build -t intel/ros2:ros2_grasp_library_deps --build-arg http_proxy=<proxy>:<port> --build-arg https_proxy=<proxy>:<port> .
```
## OPTION:Please refer below command to verify image creating success
```
docker images

REPOSITORY            TAG                           IMAGE ID            CREATED             SIZE
intel/ros2            ros2_grasp_library_deps       b6d619a01f33        1 hours ago         6.92GB
```
# Run OpenVINO Grasp Library with RGBD Camera
## Terminal 1: Build ros2_grasp_library and launch Rviz2 to illustrate detection results.
After the project runs, there will be a pop-up x window, you need to set the operating environment first.
```
./setup_docker_display.sh

docker run -it --rm --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw -v /dev/bus/usb:/dev/bus/usb  \
    -v /dev:/dev:rw -e XAUTHORITY=/tmp/.docker.xauth -e DISPLAY --name ros2_grasp_library intel/ros2:ros2_grasp_library_deps bash

# cd /root/
# mkdir -p ros2_ws/src
# cd ros2_ws/src
# git clone https://github.com/intel/ros2_grasp_library.git
# git clone https://github.com/intel/ros2_intel_realsense.git -b refactor
# git clone https://github.com/intel/ros2_openvino_toolkit.git
# cd ..
# colcon build --symlink-install --packages-select grasp_msgs moveit_msgs people_msgs grasp_ros2 realsense_msgs realsense_ros realsense_node
# source ./install/local_setup.bash
# ros2 run rviz2 rviz2 -d src/ros2_grasp_library/grasp_ros2/rviz2/grasp.rviz
```
## Terminal 2: launch RGBD camera
```
docker exec -t -i ros2_grasp_library bash

# source /root/ros2_ws/install/setup.bash
# ros2 run realsense_node realsense_node
```
## Terminal 3: launch Grasp Library
```
docker exec -t -i ros2_grasp_library bash

# source /root/ros2_ws/install/setup.bash
# ros2 run grasp_ros2 grasp_ros2 __params:=src/ros2_grasp_library/grasp_ros2/cfg/grasp_ros2_params.yaml
```
Note: If you haven't already installed or want more information on how to use docker, please see the article here for more information:
https://docs.docker.com/install/

