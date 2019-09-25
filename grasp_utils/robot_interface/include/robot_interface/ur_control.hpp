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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <robot_interface/control_base.hpp>

#include "ur_modern_driver/log.h"
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/factory.h"
#include "ur_modern_driver/ur/messages.h"
#include "ur_modern_driver/ur/parser.h"
#include "ur_modern_driver/ur/producer.h"
#include "ur_modern_driver/ur/rt_state.h"
#include "ur_modern_driver/ur/state.h"

static const std::vector<std::string> JOINTS = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                                 "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };
static const std::string HOST = "192.168.0.5";
static const bool SHUTDOWN_ON_DISCONNECT = true;
static const int UR_SECONDARY_PORT = 30002;
static const int UR_RT_PORT = 30003;

struct ProgArgs
{
public:
  std::string host;
  std::vector<std::string> joint_names;
  bool shutdown_on_disconnect;
};

class IgnorePipelineStoppedNotifier : public INotifier
{
public:
  void started(std::string name)
  {
    LOG_INFO("Starting pipeline %s", name.c_str());
  }
  void stopped(std::string name)
  {
    LOG_INFO("Stopping pipeline %s", name.c_str());
  }
};

class ShutdownOnPipelineStoppedNotifier : public INotifier
{
public:
  void started(std::string name)
  {
    LOG_INFO("Starting pipeline %s", name.c_str());
  }
  void stopped(std::string name)
  {
    LOG_INFO("Shutting down on stopped pipeline %s", name.c_str());
    rclcpp::shutdown();
    exit(1);
  }
};

class URControl: public ArmControlBase, public URRTPacketConsumer
{
public:
  URControl(const std::string node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ArmControlBase(node_name, options), gripper_powered_up_(false)
  {
    for (auto const& joint : JOINTS)
    {
      joint_names_.push_back(joint);
    }
  }

  ~URControl()
  {
    rt_pl_->stop();
    state_pl_->stop();
    factory_.reset(nullptr);
    notifier_ = nullptr;
    LOG_INFO("UR control interface shut down.");
  }

  // Overload ArmControlBase functions
  virtual bool moveToTcpPose(double x, double y, double z, 
                             double alpha, double beta, double gamma, 
                             double vel, double acc);

  virtual bool open(const double distance = 0);

  virtual bool close(const double distance = 0);

  // Send URScript to ur robot controller
  bool urscriptInterface(const std::string command_script);

  // Start socket communication loop
  bool start();

  // Overload URRTPacketConsumer functions
  virtual bool consume(RTState_V1_6__7& state);
  virtual bool consume(RTState_V1_8& state);
  virtual bool consume(RTState_V3_0__1& state);
  virtual bool consume(RTState_V3_2__3& state);

  virtual void setupConsumer()
  {
  }
  virtual void teardownConsumer()
  {
  }
  virtual void stopConsumer()
  {
  }

  // Functions to publish joint states
  bool publishJoints(RTShared& packet, rclcpp::Time t);
  bool publish(RTShared& packet);

  // Function to get tool pose
  bool getTcpPose(RTShared& packet);

private:

  ProgArgs args_;
  std::string local_ip_;
  std::unique_ptr<URFactory> factory_;

  // Robot rt message
  std::unique_ptr<URParser<RTPacket>> rt_parser_;
  std::unique_ptr<URStream> rt_stream_;
  std::unique_ptr<URProducer<RTPacket>> rt_prod_;
  std::unique_ptr<URCommander> rt_commander_;
  vector<IConsumer<RTPacket> *> rt_vec_;
  std::unique_ptr<MultiConsumer<RTPacket>> rt_cons_;
  std::unique_ptr<Pipeline<RTPacket>> rt_pl_;

  INotifier *notifier_;

  // Robot state message
  std::unique_ptr<URParser<StatePacket>> state_parser_;
  std::unique_ptr<URStream> state_stream_;
  std::unique_ptr<URProducer<StatePacket>> state_prod_;
  vector<IConsumer<StatePacket> *> state_vec_;
  std::unique_ptr<MultiConsumer<StatePacket>> state_cons_;
  std::unique_ptr<Pipeline<StatePacket>> state_pl_;

  bool gripper_powered_up_;
};