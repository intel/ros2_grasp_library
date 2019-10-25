# handeye_target_detection

## 1. Introduction

This package is used to estimate the pose of
calibration patterns. Currently four kinds of OpenCV calibration patterns are supported: CHESSBOARD, ASYMMETRIC_CIRCLES_GRID,
CHARUCO, ARUCO.

## 2. Prerequisite

* Download and print the calibration pattern on an A4 paper without the border shrink:
  * [Chessboard](./data/pattern/chessboard_9X6.png)
    * Width * Height: 9X6
    * Square size: 0.026
  * [Asymmetric circles grid 1](./data/pattern/asymmetric_circles_grid_4X11.png)
    * Width * Height: 4X11
    * Cricles seperation: 0.035
  * [Asymmetric circles grid 2](./data/pattern/asymmetric_circles_grid_3X5.png)
    * Width * Height: 3X5
    * Cricles seperation: 0.035
  * [Aruco board 1](./data/pattern/aruco_5X7_DICT_6X6_250.png)
    * Width * Height: 5X7
    * Dictionary: 6X6_250
    * Marker size: 0.035
    * Marker seperation: 0.007
  * [Aruco board 2](./data/pattern/aruco_3X4_DICT_4X4_50.png)
    * Width * Height: 3X4
    * Dictionary: 4X4_50
    * Marker size: 0.0256
    * Marker seperation: 0.0066
  * [Charuco board](./data/pattern/charuco_5X7_DICT_6X6_250.jpg)
    * Width * Height: 5X7
    * Dictionary: 6X6_250
    * Square size: 0.035
    * Marker size: 0.022
* RGB camera:
  * Video/Image file
  * Intel<sup>®</sup>RealSense<sup>TM</sup> (Tested with D435)
  * Standard USB camera
* ROS Dashing (Ubuntu 18.04, 64 bits)

## 3. Environment Setup

* Install [ROS Dashing](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)
* Install [Intel<sup>®</sup>RealSense<sup>TM</sup> SDK 2.0](https://github.com/IntelRealSense/librealsense)
* Install [Intel<sup>®</sup>RealSense<sup>TM</sup> ROS2 Wrapper](https://github.com/intel/ros2_intel_realsense)

## 4. Build and install

* Install dependencies

```shell
sudo apt install ros-dashing-cv-bridge \
                 ros-dashing-image-transport
```

* Build with ros2_grasp_library

## 5. Run

Before running the code, a camera should be launched and
guarantee that the topics of the RGB image and camera info are being published.

If a RealSense D435 camera is used, run the following command to bring up the camera:

```shell
ros2 run realsense_node realsense_node __params:=`ros2 pkg prefix realsense_examples`/share/realsense_examples/config/d435.yaml
```

> Note: other cameras can be used, only if it can publish RGB image topic and camera_info topic.

Run the following command to bring up the pose estimation of a calibration pattern:

```shell
ros2 launch handeye_target_detection pose_estimation.launch.py
```

The launch file loads parameters from `./launch/pose_estimation.yaml` file. For the meaning of these parameters, refer to the table below:

```shell
Launch options:

  --pattern (string, default: ARUCO)
    The pattern of the calibration plate, it should be one of {CHESSBOARD, ASYMMETRIC_CIRCLES_GRID, CHARUCO, ARUCO}

  --image_topic (string, default: /camera/color/image_raw)
    The RGB image topic, to which the pose estimation node subscribes

  --camera_info_topic (string, default: /camera/color/camera_info)
    The camera info topic, to which the pose estimation node subscribes

  --publish_image_topic (string, default: /image/detected)
    The image topic published by the pose estimation ndoe
  
  --width (int, default: 3)
    Usualy the number of squres or markers along the X direction

  --Height (int, default: 4)
    Usually the number of squares or markers alogn the Y direction

  --dictionary (string, default: DICT_6X6_250)
    If ARUCO or CHARUCO pattern is used, this parameter indicates which marker dictionary the Board markers belong to. For more infomation, refer to https://docs.opencv.org/3.4.0/d5/dae/tutorial_aruco_detection.html

  --chessboard_square_size (double, default: 0.026)
    If a CHESSBOARD is used, this indicates the square length

  --circle_grid_seperation (double, default: 0.035)
    If an ASYMMETRIC_CIRCLES_GRID is used, this indicates the seperation distance between circles

  --aruco_board_marker_size (double, default: 0.035)
    The size of aruco marker

  --aruco_board_marker_seperation (double, default: 0.007)
    The seperation distance of aruco marker

  --charuco_board_marker_size (double, default: 0.022)
    The length of charuco marker

  --charuco_board_square_size (double, default: 0.037)
    The length of charuco square
```

For running properly, user has to customize these parameters.

For example, to make the pose estimation of the four calibration patterns listed in the `Prerequisite` section, the parameters should be:

* Chessboard

```yml
pose_estimation:
    ros__parameters:
        pattern: "CHESSBOARD"
        image_topic: "/camera/color/image_raw"
        camera_info_topic: "/camera/color/camera_info"
        publish_image_topic: "/image/detected"
        width: 9
        height: 6
        chessboard_square_size: 0.026
```

* Asymmetric circles grid

```yml
# 4X11 0.035
pose_estimation:
    ros__parameters:
        pattern: "ASYMMETRIC_CIRCLES_GRID"
        image_topic: "/camera/color/image_raw"
        camera_info_topic: "/camera/color/camera_info"
        publish_image_topic: "/image/detected"
        width: 4
        height: 11
        circle_grid_seperation: 0.035
```

```yml
# 3X5 0.035
pose_estimation:
    ros__parameters:
        pattern: "ASYMMETRIC_CIRCLES_GRID"
        image_topic: "/camera/color/image_raw"
        camera_info_topic: "/camera/color/camera_info"
        publish_image_topic: "/image/detected"
        width: 3
        height: 5
        circle_grid_seperation: 0.035
```

* Aruco board

```yml
# 5x7 DICT_6X6_250 0.035 0.007
pose_estimation:
    ros__parameters:
        pattern: "ARUCO"
        image_topic: "/camera/color/image_raw"
        camera_info_topic: "/camera/color/camera_info"
        publish_image_topic: "/image/detected"
        width: 5
        height: 7
        dictionary: "DICT_6X6_250"
        aruco_board_marker_size: 0.035
        aruco_board_marker_seperation: 0.007
```

```yml
# 3x4 DICT_4X4_50 0.0256 0.0066
pose_estimation:
    ros__parameters:
        pattern: "ARUCO"
        image_topic: "/camera/color/image_raw"
        camera_info_topic: "/camera/color/camera_info"
        publish_image_topic: "/image/detected"
        width: 3
        height: 4
        dictionary: "DICT_4X4_50"
        aruco_board_marker_size: 0.0256
        aruco_board_marker_seperation: 0.0066
```

* Charuco board

```yml
pose_estimation:
    ros__parameters:
        pattern: "CHARUCO"
        image_topic: "/camera/color/image_raw"
        camera_info_topic: "/camera/color/camera_info"
        publish_image_topic: "/image/detected"
        width: 5
        height: 7
        dictionary: "DICT_6X6_250"
        charuco_board_marker_size: 0.022
        charuco_board_square_size: 0.037
```

## 6. Results

If the detection works well, the (red, green, blue) arrows indicating the coordiante system origin at the corner of the calibration board should show up on the picture. For the patterns listed above, it should looks like:

CHESSBOARD|ASYMMETRIC CIRCLES GRID 4X11|ASYMMETRIC CIRCLES GRID 3X5|CHARUCO|ARUCO 5X7|ARUCO 3X4
----------|----------------------------|----|-------|-----|----
![CHESSBOARD][image1]|![ASYMMETRIC CIRCLES GRID][image2_1]|![ASYMMETRIC CIRCLES GRID][image2_2]|![CHARUCO][image3]|![ARUCO][image4_1]|![ARUCO][image4_2]

[image1]:data/detected/chessboard/chessboard.png
[image2_1]:data/detected/circlegrid/4X11_circles_grid.png
[image2_2]:data/detected/circlegrid/3X5_circles_grid.png
[image3]:data/detected/charuco/charuco.png
[image4_1]:data/detected/aruco/5X7_aruco.png
[image4_2]:data/detected/aruco/3X4_aruco.png

###### *Any security issue should be reported using process at https://01.org/security*
