# Test Grasp Library

This tutorial explains how to test Grasp Library. A few Test Suites enabled:
* ROS2 built-in tests for static code scanning like, copyright tests, cppcheck tests, cpplint tests, lint_cmake tests, uncrustify tests, xmllint tests.
* Grasp Library basic functional test: tgrasp_library

Before test, make sure you have setup the environment to build the Grasp Library, following tutorials [Grasp Library with RGBD Camera](tutorials_1_grasp_library_with_camera.md).

## Test Grasp Library
Sanity test cases are very basic tests cover ROS2 topic and ROS2 service for Grasp Library. The tests take inputs from a pre-stored PointCloud file (.pcd). Thus it's unnecessary to launch an RGBD camera.
```bash
# Terminal 1, launch Grasp Library
ros2 run grasp_library grasp_library __params:=src/ros2_grasp_library/grasp_library/cfg/test_grasp_library.yaml
# Terminal 2, run tests
colcon test --packages-select grasp_msgs grasp_library
```
For failed cases check detailed logs at "log/latest_test/grasp_library/stdout.log".
