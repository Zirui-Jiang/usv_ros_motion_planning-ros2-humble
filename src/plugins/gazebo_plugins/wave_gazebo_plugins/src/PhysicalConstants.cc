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

// 集中提供固定物理常数；此处常数不随 Gazebo 世界参数或 SDF 自动变化。
#include "wave_gazebo_plugins/PhysicalConstants.hh"

namespace asv
{
  /////////////////////////////////////////////////////////////////////////////
  // 采用 Z 轴向上的约定，重力加速度为 -9.8 m/s²。
  double PhysicalConstants::Gravity()
  {
    return -9.8;
  }

  /////////////////////////////////////////////////////////////////////////////
  // 万有引力常数单位为 m³/(kg·s²)，与地表重力加速度不同。
  double PhysicalConstants::G()
  {
    return 6.67408E-11;
  }

  /////////////////////////////////////////////////////////////////////////////
  // 返回水密度 kg/m³。
  double PhysicalConstants::WaterDensity()
  {
    return 998.6;
  }

  /////////////////////////////////////////////////////////////////////////////
  // 返回运动黏度 m²/s，原始参数对应 18 摄氏度水。
  double PhysicalConstants::WaterKinematicViscosity()
  {
    return 1.0533E-6;
  }
}
