// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
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

#include "sdk_command/sdk_command_node.hpp"

SDKCmdNode::SDKCmdNode(const rclcpp::NodeOptions & options) : Node("user_command_node", options)
{
  this->get_parameter("pub_freq", pub_freq_);
  RCLCPP_INFO(this->get_logger(), "User SDK!");

  user_sdk_publisher_ = this->create_publisher<tita_locomotion_interfaces::msg::LocomotionCmd>(
    tita_topic::user_command, 10);

  auto period = std::chrono::duration<double>(1.0 / pub_freq_);
  timer_ = this->create_wall_timer(period, std::bind(&SDKCmdNode::timerCallback, this));
}

void SDKCmdNode::timerCallback()
{
  auto unique_joy_msg = std::make_unique<tita_locomotion_interfaces::msg::LocomotionCmd>();
  unique_joy_msg->header.stamp = this->now();
  unique_joy_msg->header.frame_id = "cmd";

  auto pitch_cmd = tita_utils::pitchToQuaternion(1.0);
  auto roll_cmd = tita_utils::rollToQuaternion(1.0);
  auto body_pose = tita_utils::multiplyQuaternions(pitch_cmd, roll_cmd);

  unique_joy_msg->twist.angular.z = 0.5;
  unique_joy_msg->pose.position.z = 0.2;
  unique_joy_msg->fsm_mode = "transform_up"; // "idle" "stop" "charge" "jump" "balance" "transform_down" "transform_up"
  unique_joy_msg->pose.orientation.x = body_pose.x;
  unique_joy_msg->pose.orientation.y = body_pose.y;
  unique_joy_msg->pose.orientation.z = body_pose.z;
  unique_joy_msg->pose.orientation.w = body_pose.w;

  user_sdk_publisher_->publish(std::move(unique_joy_msg));
}
