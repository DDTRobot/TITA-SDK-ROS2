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

#ifndef SDK_COMMAND__SDK_COMMAND_NODE_HPP_
#define SDK_COMMAND__SDK_COMMAND_NODE_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tita_locomotion_interfaces/msg/locomotion_cmd.hpp"
#include "tita_utils/topic_names.hpp"

class SDKCmdNode : public rclcpp::Node
{
public:
  explicit SDKCmdNode(const rclcpp::NodeOptions & options);

private:
  /// \brief  Numble A receive callback.
  void timerCallback();

  rclcpp::Publisher<tita_locomotion_interfaces::msg::LocomotionCmd>::SharedPtr user_sdk_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double pub_freq_ = 170.0;
};

#endif  // SDK_COMMAND__SDK_COMMAND_NODE_HPP_
