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

// Gazebo Base 子实体保存波场参数，提供物理插件间共享访问；本版本不在此生成动态网格。
#include <iostream>
#include <string>

#include <gazebo/common/Assert.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Vector2.hh>

#include "wave_gazebo_plugins/WavefieldEntity.hh"
#include "wave_gazebo_plugins/Wavefield.hh"
#include "wave_gazebo_plugins/Utilities.hh"

using namespace gazebo;
using namespace common;

namespace asv
{
/////////////////////////////////////////////////////////////////////////////
// WavefieldEntity

  /// \internal
  /// \brief Private data for the WavefieldEntity
  class WavefieldEntityPrivate
  {
    /// \brief The size of the wavefield. Default value is [1000 1000].
    public: ignition::math::Vector2d size;

    /// \brief The number of grid cells in the wavefield.
    /// Default value is [50 50].
    public: ignition::math::Vector2d cellCount;

    /// \brief The wave parameters.
    public: std::shared_ptr<asv::WaveParameters> waveParams;
  };

  /////////////////////////////////////////////////
  WavefieldEntity::~WavefieldEntity()
  {
  }

  /////////////////////////////////////////////////
  WavefieldEntity::WavefieldEntity(physics::BasePtr _parent) :
    Base(_parent),
    data(new WavefieldEntityPrivate())
  {
  }

  /////////////////////////////////////////////////
  // 读取 size、cell_count 元数据，再从 wave 子元素生成分量参数；这两个尺寸字段当前仅保存。
  void WavefieldEntity::Load(sdf::ElementPtr _sdf)
  {
    GZ_ASSERT(_sdf != nullptr, "Invalid parameter _sdf");

    Base::Load(_sdf);

    // Wavefield Parameters
    this->data->size = Utilities::SdfParamVector2(*_sdf, "size", \
                                  ignition::math::Vector2d(1000, 1000));
    this->data->cellCount = Utilities::SdfParamVector2(*_sdf, "cell_count", \
                                   ignition::math::Vector2d(50, 50));

    // Wave Parameters
    gzmsg << "WavefieldEntity: Loading WaveParameters from SDF" <<  std::endl;
    this->data->waveParams.reset(new WaveParameters());
    if (_sdf->HasElement("wave"))
    {
      gzmsg << "Found <wave> tag" << std::endl;
      sdf::ElementPtr sdfWave = _sdf->GetElement("wave");
      this->data->waveParams->SetFromSDF(*sdfWave);
    }
    else
    {
      gzmsg << "Missing <wave> tag" << std::endl;
    }
    // @DEBUG_INFO
    this->data->waveParams->DebugPrint();
  }

  /////////////////////////////////////////////////
  void WavefieldEntity::Fini()
  {
    Base::Fini();
  }

  /////////////////////////////////////////////////
  void WavefieldEntity::Init()
  {
  }

  /////////////////////////////////////////////////
  void WavefieldEntity::Reset()
  {
  }

  /////////////////////////////////////////////////
  // 当前更新函数为空，采样器直接使用共享参数和调用时刻求值。
  void WavefieldEntity::Update()
  {
  }

  /////////////////////////////////////////////////
  // const 共享指针允许调用者读取参数而不通过该接口修改它。
  std::shared_ptr<const WaveParameters> WavefieldEntity::GetWaveParams() const
  {
    return this->data->waveParams;
  }

  /////////////////////////////////////////////////
  // 固定使用父模型名加 ::wavefield_entity 后缀，与 GetWaveParams 的查找规则配套。
  std::string WavefieldEntity::MakeName(const std::string& _parentName)
  {
    return std::string(_parentName + "::wavefield_entity");
  }
}
