# Grasp Library Tests and Exampels

This tutorial documents Grasp Library tests which also serve as example codes for the usage of Grasp Library.

## Grasp Library Tests
Test Suites enabled:
* ROS2 built-in tests for static code scanning like, copyright tests, cppcheck tests, cpplint tests, lint_cmake tests, uncrustify tests, xmllint tests.
* Grasp Library basic functional tests: tgrasp_library, basic tests cover ROS2 topic and ROS2 service of Grasp Library.

Before test, make sure you have setup the environment to build the Grasp Library, following tutorials [Grasp Library with RGBD Camera](tutorials_1_grasp_library_with_camera.md). The tests take inputs from a pre-stored PointCloud file (.pcd). Thus it's unnecessary to launch an RGBD camera.
```bash
# Terminal 1, launch Grasp Library
ros2 run grasp_library grasp_library __params:=src/ros2_grasp_library/grasp_library/cfg/test_grasp_library.yaml
# Terminal 2, run tests
colcon test --packages-select grasp_msgs grasp_library
```
For failed cases check detailed logs at "log/latest_test/grasp_library/stdout.log".

## Grasp Library Examples
The [grasp test codes](../grasp_library/tests/tgrasp_library.cpp) also demonstrate how to use this Grasp Library for grasp detection and grasp planning.

### Grasp Detection Example (Non-MoveIt App)
This example creats ROS2 subscription to the "Detected Grasps" topic and get the detection results from callback. Grasp Library is expected to work in 'auto_mode=true', sensor-driven grasp detection, see example launch options [here](../grasp_library/cfg/grasp_library_params.yaml).

```bash
#include <rclcpp/rclcpp.hpp>
#include <grasp_msgs/msg/grasp_config_list.hpp>
#include "grasp_library/consts.hpp"

static rclcpp::Node::SharedPtr node = nullptr;

static void topic_cb(const grasp_msgs::msg::GraspConfigList::SharedPtr msg)
{
  RCLCPP_INFO(node->get_logger(), "Grasp Callback Received");
}

int main(int argc, char * argv[])
{
  // init ROS2
  rclcpp::init(argc, argv);
  // create ROS2 node
  node = rclcpp::Node::make_shared("GraspDetectionExample");
  // subscribe to the "Detected Grasps" topic
  auto sub = node->create_subscription<grasp_msgs::msg::GraspConfigList>(Consts::kTopicDetectedGrasps, topic_cb);
  // create ROS2 executor to process any pending in/out messages
  rclcpp::spin(node);

  node = nullptr;
  rclcpp::shutdown();
  return ret;
}
```

### Grasp Planning Example (MoveIt App)
This example creates ROS2 client for the "plan_grasps" service and get the palnning results from async service response. Grasp Library is expected to work in 'auto_mode=false', service-driven grasp detection, see launch option example [here](../grasp_library/cfg/test_grasp_library.yaml).

```bash
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/grasp_planning.hpp>
#include "grasp_library/consts.hpp"

static rclcpp::Node::SharedPtr node = nullptr;
static std::shared_ptr<GraspPlanning::Response> result = nullptr;

int main(int argc, char * argv[])
{
  // init ROS2
  rclcpp::init(argc, argv);
  // create ROS2 node
  node = rclcpp::Node::make_shared("GraspPlanningExample");
  // create ROS2 client for MoevIt "plan_grasps" service
  auto client = node->create_client<GraspPlanning>("plan_grasps");
  // wait for ROS2 service ready
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Client interrupted");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Wait for service");
  }
  // fill in a request
  auto request = std::make_shared<GraspPlanning::Request>();
  // send async request
  auto result_future = client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Request sent");
  // wait for response
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Request failed");
    return 1;
  }
  // get grasp planning results from response
  result = result_future.get();
  RCLCPP_INFO(node->get_logger(), "Response received %d", result->error_code.val);

  node = nullptr;
  rclcpp::shutdown();
  return ret;
}
```
