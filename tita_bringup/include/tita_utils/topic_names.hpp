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

#ifndef TITA_UTILS__TOPIC_NAMES_HPP_
#define TITA_UTILS__TOPIC_NAMES_HPP_

#include <string>

namespace tita_topic
{

// Base
inline const std::string odom = "/odom";
inline const std::string map = "map";

// command
inline const std::string active_command = "command/active/command";
inline const std::string teleop_command = "command/teleop/command";
inline const std::string passive_command = "command/passive/command";
inline const std::string user_command = "command/user/command";
inline const std::string joy = "/joy";

inline const std::string manager_twist_command = "command/manager/cmd_twist";
inline const std::string manager_pose_command = "command/manager/cmd_pose";
inline const std::string manager_key_command = "command/manager/cmd_key";
inline const std::string manager_locomotion_command = "command/manager/locomotion_cmd";

// perception
inline const std::string image_left = "perception/camera/image/left";
inline const std::string image_right = "perception/camera/image/right";
inline const std::string cam_info_left = "perception/camera/info/left";
inline const std::string cam_info_right = "perception/camera/info/right";
inline const std::string cam_imu = "perception/cam/imu";
inline const std::string cam_point = "perception/camera/point_cloud";
inline const std::string tof_face_point = "perception/devices/face_point";
inline const std::string tof_neck_point = "perception/devices/neck_point";
inline const std::string ultrasonic_data = "perception/devices/ultrawave";
inline const std::string angle_data = "perception/detector/angle_data";
inline const std::string obstacle_data = "perception/obstacle/distance_data";
inline const std::string obstacle_point = "perception/obstacle/point";

// system
inline const std::string system_status = "system/status";
inline const std::string kleft_battery_topic = "system/battery/left";
inline const std::string kright_battery_topic = "system/battery/right";
inline const std::string kleft_battery_diagnostic_topic = "system/battery_diagnostic/left";
inline const std::string kright_battery_diagnostic_topic = "system/battery_diagnostic/right";
inline const std::string kpower_state_set_service = "system/power_supply/power_state_set_srv";
inline const std::string kpower_heart_beat_service = "system/power_supply/power_heart_beat_srv";
inline const std::string kpower_self_test_service = "system/power_supply/power_self_test_srv";
inline const std::string ktemperature_controller_topic =
  "system/temperature/temperature_controller";
inline const std::string kfan_mode_set_service = "system/temperature/fan_mode_set";
inline const std::string krobot_power_off_trigger_service =
  "system/power_supply/robot_power_off_trigger_srv";
inline const std::string khead_light_control_service =
  "system/light_control/head_light_control_srv";
inline const std::string ktail_light_control_service =
  "system/light_control/tail_light_control_srv";
inline const std::string kleg_light_control_service = "system/light_control/leg_light_control_srv";

// locomotion
inline const std::string body_imu = "imu_sensor_broadcaster/imu";
inline const std::string joint_states = "joint_states";
inline const std::string fsm_mode = "locomotion/body/fsm_mode";
inline const std::string motors_status = "locomotion/motors_status";

// tita_tower
inline const std::string tower_odom = "tower/odometry";
inline const std::string tower_map = "tower/map_point";
inline const std::string tower_left_img = "tower/left_img";
inline const std::string tower_right_img = "tower/right_img";

// localization
inline const std::string chassis_odom = "chassis/odometry";
inline const std::string chassis_reset = "chassis/srv/reset";

}  // namespace tita_topic

#endif  // TITA_UTILS__TOPIC_NAMES_HPP_
