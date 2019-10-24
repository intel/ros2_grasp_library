/** Copyright (c) 2019 Intel Corporation. All Rights Reserved
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *     http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  */

#include <chrono>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <handeye_tf_service/srv/handeye_tf.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

using HandeyeTF = handeye_tf_service::srv::HandeyeTF;
using namespace std::chrono_literals;

class ServerNode : public rclcpp::Node
{
public:
  explicit ServerNode(const rclcpp::NodeOptions & options)
  : Node("handeye_tf_server", options), broadcaster_(this)
  {
    // Init tf message
    tf_msg_.header.frame_id = "base"; // Used to void TF_NO_FRAME_ID error, updated by user later
    tf_msg_.child_frame_id = "camera_link";
    // Initialize rotation to avoid TF_DENORMALIZED_QUATERNION error
    tf_msg_.transform.rotation.x = 0.0;
    tf_msg_.transform.rotation.y = 0.0;
    tf_msg_.transform.rotation.z = 0.0;
    tf_msg_.transform.rotation.w = 1.0;
    
    // Init timer
    timer_ = this->create_wall_timer(
      100ms, std::bind(&ServerNode::timer_callback, this));
    
    // Init tf listener
    clock_ = this->get_clock();
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Service handler
    auto handle_service =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<HandeyeTF::Request> request,
        std::shared_ptr<HandeyeTF::Response> response) -> void
      {
        if (request->publish.data) // Publish the camera-robot transform
        {
          RCLCPP_INFO(this->get_logger(), "Incoming publish request\nframe_id: %s child_frame_id: %s",
            request->transform.header.frame_id.data(), request->transform.child_frame_id.data());
          tf_msg_ = request->transform;
        }
        else // Lookup the requested transform
        {
          (void)request_header;
          RCLCPP_INFO(this->get_logger(), "Incoming lookup request\nframe_id: %s child_frame_id: %s",
            request->transform.header.frame_id.data(), request->transform.child_frame_id.data());

          try
          {
            response->tf_lookup_result = tf_buffer_->lookupTransform(request->transform.header.frame_id, 
                                    request->transform.child_frame_id, tf2::TimePoint());
          }
          catch (tf2::TransformException &ex)
          {
            std::string temp = ex.what();
            RCLCPP_WARN(this->get_logger(), "%s", temp.c_str());
          }
        }
      };

    // Create a service that will use the callback function to handle requests.
    srv_ = create_service<HandeyeTF>("handeye_tf_service", handle_service);
    RCLCPP_INFO(this->get_logger(), "Handeye TF service created.");
  }

private:
  void timer_callback()
  {
    broadcaster_.sendTransform(tf_msg_);
  }

  // Handeye service
  rclcpp::Service<HandeyeTF>::SharedPtr srv_;

  // Variables used for looking up tf transforms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timer used for static transform publish 
  rclcpp::TimerBase::SharedPtr timer_;
  // TF message for camera w.r.t robot transform
  geometry_msgs::msg::TransformStamped tf_msg_;
  // TF broadcaster
  tf2_ros::StaticTransformBroadcaster broadcaster_;
  rclcpp::Clock::SharedPtr clock_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServerNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}