/*
 * Copyright (C) 2019  Rhys Mainwaring
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
 *
*/

// 实现深水重力波色散关系 ω²=|g|k，供波谱参数在周期与波长之间转换。
#include <cmath>

#include "wave_gazebo_plugins/Physics.hh"
#include "wave_gazebo_plugins/PhysicalConstants.hh"

namespace asv
{
  /////////////////////////////////////////////////
  // 输入波数 k，单位 rad/m，输出角频率 rad/s；平方根要求 k 非负。
  double Physics::DeepWaterDispersionToOmega(double _wavenumber)
  {
    const double g = std::fabs(PhysicalConstants::Gravity());
    return std::sqrt(g * _wavenumber);
  }

  /////////////////////////////////////////////////
  // 输入角频率 rad/s，返回波数 rad/m；使用重力绝对值以消除 Z 轴符号约定。
  double Physics::DeepWaterDispersionToWavenumber(double _omega)
  {
    const double g = std::fabs(PhysicalConstants::Gravity());
    return _omega * _omega / g;
  }
}
