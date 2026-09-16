/*
 * Copyright (C) 20109 Brian Bingham
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

// 波高标尺插件直接将模型世界 Z 设置为计算水面高度，便于比对视觉海面与物理波高。
#include <functional>
#include <string>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>

#include "wave_gazebo_plugins/wavegauge_plugin.hh"
#include "wave_gazebo_plugins/Wavefield.hh"
#include "wave_gazebo_plugins/WavefieldEntity.hh"
#include "wave_gazebo_plugins/WavefieldModelPlugin.hh"

using namespace asv;
using namespace gazebo;

/////////////////////////////////////////////////
WaveguagePlugin::WaveguagePlugin()
  : fluidLevel(0.0)
{
}

/////////////////////////////////////////////////
// 读取 wave_model 名称与 fluid_level 平均水位 m，保留传入的模型指针。
void WaveguagePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Received NULL model pointer");
  GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");

  // Capture the model pointer.
  this->model = _model;

  if (_sdf->HasElement("wave_model"))
  {
    this->waveModelName = _sdf->Get<std::string>("wave_model");
  }
  if (_sdf->HasElement("fluid_level"))
  {
    this->fluidLevel = _sdf->Get<double>("fluid_level");
  }
}

/////////////////////////////////////////////////
// 注册每个物理步的回调，周期由 Gazebo 世界更新驱动。
void WaveguagePlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WaveguagePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
// 先查共享波场参数；找不到时保持原位，找到后在模型 XY 处取波高并加平均水位。
void WaveguagePlugin::OnUpdate()
{
  // Retrieve the wave model...

  std::shared_ptr<const WaveParameters> waveParams \
    = WavefieldModelPlugin::GetWaveParams(
      this->model->GetWorld(), this->waveModelName);

  // No ocean waves...
  if (waveParams == nullptr)
  {
    return;
  }
  #if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d modelPose = this->model->WorldPose();
  #else
    ignition::math::Pose3d modelPose = this->model->GetWorldPose().Ign();
  #endif

  // Compute the wave displacement at the model location
  #if GAZEBO_MAJOR_VERSION >= 8
    double waveHeightS = WavefieldSampler::ComputeDepthSimply(
      *waveParams, modelPose.Pos(),
      this->model->GetWorld()->SimTime().Double());
  #else
    double waveHeightS = WavefieldSampler::ComputeDepthSimply(
      *waveParams, modelPose.Pos(),
      this->model->GetWorld()->GetSimTime().Double());
  #endif

  // Add the mean water level
  waveHeightS += this->fluidLevel;

  // Set vertical location to match the wave height
  // 只替换竖直位置，保留水平位置与姿态；这里不通过浮力动力学积分更新位置。
  modelPose.Pos().Z(waveHeightS);
  this->model->SetWorldPose(modelPose);
}

GZ_REGISTER_MODEL_PLUGIN(WaveguagePlugin)
