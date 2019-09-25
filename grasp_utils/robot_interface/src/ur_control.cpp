// Copyright (c) 2019 Intel Corporation. All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <robot_interface/ur_control.hpp>

bool URControl::moveToTcpPose(double x, double y, double z, 
                              double alpha, double beta, double gamma, 
                              double vel, double acc)
{
  std::string command_script = "movej(p[" + 
                               std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "," +
                               std::to_string(alpha) + "," + std::to_string(beta) + "," + std::to_string(gamma) + "]," + 
                               std::to_string(vel) + "," + std::to_string(acc) + ")\n";
  urscriptInterface(command_script);
  return true;
}

bool URControl::open(const double distance)
{
  rt_commander_->setToolVoltage(static_cast<uint8_t>(24));
  if (!gripper_powered_up_)
  {
    rt_commander_->setToolVoltage(static_cast<uint8_t>(24));
    gripper_powered_up_ = true;
    std::cout << "Gripper powered up." << std::endl;    
  }

  rt_commander_->setDigitalOut(16, true);
  rt_commander_->setDigitalOut(17, false);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  return true;
}

bool URControl::close(const double distance)
{
  if (!gripper_powered_up_)
  {
    rt_commander_->setToolVoltage(static_cast<uint8_t>(24));
    gripper_powered_up_ = true;
    std::cout << "Gripper powered up." << std::endl;
  }

  rt_commander_->setDigitalOut(16, false);
  rt_commander_->setDigitalOut(17, true);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  return true;
}

bool URControl::urscriptInterface(const std::string command_script)
{
  bool res = rt_commander_->uploadProg(command_script);
  if (!res)
  {
    LOG_ERROR("Program upload failed!");
  }

  return res;
}

bool URControl::start()
{
  // Initialize parameter client
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  // Get parameters
  args_.host = parameters_client->get_parameter("host", HOST);
  args_.joint_names = parameters_client->get_parameter("joint_names", JOINTS);
  args_.shutdown_on_disconnect = parameters_client->get_parameter("shutdown_on_disconnect", SHUTDOWN_ON_DISCONNECT);

  // Print parameters
  RCLCPP_INFO(this->get_logger(), args_.host);
  std::stringstream ss;
  for (auto & name : args_.joint_names)
  {
    ss << name << " ";
  }
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  RCLCPP_INFO(this->get_logger(), std::to_string(args_.shutdown_on_disconnect));

  // Initialize socket communication
  factory_.reset(new URFactory(args_.host));

  notifier_ = nullptr;

  if (args_.shutdown_on_disconnect)
  {
    LOG_INFO("Notifier: Pipeline disconnect will shutdown the node");
    notifier_ = new ShutdownOnPipelineStoppedNotifier();
  }
  else
  {
    LOG_INFO("Notifier: Pipeline disconnect will be ignored.");
    notifier_ = new IgnorePipelineStoppedNotifier();
  }

  // RT packets
  rt_parser_ = factory_->getRTParser();
  rt_stream_.reset(new URStream(args_.host, UR_RT_PORT));
  rt_prod_.reset(new URProducer<RTPacket>(*rt_stream_, *rt_parser_));
  rt_commander_ = factory_->getCommander(*rt_stream_);
  rt_vec_.push_back(this);
  rt_cons_.reset(new MultiConsumer<RTPacket>(rt_vec_));
  rt_pl_.reset(new Pipeline<RTPacket>(*rt_prod_, *rt_cons_, "RTPacket", *notifier_));

  // Message packets
  state_parser_ = factory_->getStateParser();
  state_stream_.reset(new URStream(args_.host, UR_SECONDARY_PORT));
  state_prod_.reset(new URProducer<StatePacket>(*state_stream_, *state_parser_));
  state_cons_.reset(new MultiConsumer<StatePacket>(state_vec_));
  state_pl_.reset(new Pipeline<StatePacket>(*state_prod_, *state_cons_, "StatePacket", *notifier_));

  LOG_INFO("Starting main loop");

  rt_pl_->run();
  state_pl_->run();

  return true;
}

bool URControl::getTcpPose(RTShared& packet)
{
  auto tv = packet.tool_vector_actual;
  
  tcp_pose_.x = tv.position.x;
  tcp_pose_.y = tv.position.y;
  tcp_pose_.z = tv.position.z;
  tcp_pose_.alpha = tv.rotation.x;
  tcp_pose_.beta = tv.rotation.y;
  tcp_pose_.gamma = tv.rotation.z;

  return true;
}

bool URControl::consume(RTState_V1_6__7& state)
{
  return publish(state) && getTcpPose(state);
}
bool URControl::consume(RTState_V1_8& state)
{
  return publish(state) && getTcpPose(state);
}
bool URControl::consume(RTState_V3_0__1& state)
{
  return publish(state) && getTcpPose(state);
}
bool URControl::consume(RTState_V3_2__3& state)
{
  return publish(state) && getTcpPose(state);
}

bool URControl::publish(RTShared& packet)
{
  return publishJoints(packet, rclcpp::Node::now());
}

bool URControl::publishJoints(RTShared& packet, rclcpp::Time t)
{
  sensor_msgs::msg::JointState joint_msg;
  joint_msg.header.stamp = t;

  joint_msg.name.assign(joint_names_.begin(), joint_names_.end());
  joint_msg.position.assign(packet.q_actual.begin(), packet.q_actual.end());
  joint_msg.velocity.assign(packet.qd_actual.begin(), packet.qd_actual.end());
  joint_msg.effort.assign(packet.i_actual.begin(), packet.i_actual.end());

  joint_pub_->publish(joint_msg);

  return true;
}