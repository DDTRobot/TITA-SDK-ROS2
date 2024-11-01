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

#ifndef TITA_UTILS__TITA_UTILS_HPP_
#define TITA_UTILS__TITA_UTILS_HPP_

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

namespace tita_utils
{

struct Quaternion
{
  double w, x, y, z;
};

/// \brief  Convert roll angle to quaternion.
/// \param  double roll value in [-1.0,1.0].
/// \returns return a Quaternion struct.
Quaternion rollToQuaternion(double roll);

/// \brief  Convert pitch angle to quaternion.
/// \param  double pitch value in [-1.0,1.0].
/// \returns return a Quaternion struct.
Quaternion pitchToQuaternion(double pitch);

/// \brief  Convert yaw angle to quaternion.
/// \param  double yaw value in [-1.0,1.0].
/// \returns return a Quaternion struct.
Quaternion yawToQuaternion(double yaw);

/// \brief  Merge q1 and q2 into one.
/// \param  Quaternion q1.
/// \param  Quaternion q2.
/// \returns return a double numble.
Quaternion multiplyQuaternions(const Quaternion & q1, const Quaternion & q2);

}  // namespace tita_utils

#endif  // TITA_UTILS__TITA_UTILS_HPP_
