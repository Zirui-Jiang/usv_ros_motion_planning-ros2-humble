/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

// 根据各浮力几何体的浸没体积计算浮力与阻尼，并将合力施加到对应 Gazebo link。
#include <functional>
#include <string>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include "usv_gazebo_plugins/buoyancy_gazebo_plugin.hh"

using namespace asv;
using namespace gazebo;
using namespace gazebo::buoyancy;
using namespace ::buoyancy;

//////////////////////////////////////////////////
BuoyancyObject::BuoyancyObject()
  : linkId(-1),
    linkName(""),
    pose(0, 0, 0, 0, 0, 0),
    mass(0.0),
    shape(nullptr)
{
}

///////////////////////////////////////////////////
BuoyancyObject::BuoyancyObject(BuoyancyObject &&obj) noexcept // NOLINT
  : linkId(obj.linkId),
    linkName(obj.linkName),
    pose(obj.pose),
    mass(obj.mass),
    shape(std::move(obj.shape))
{
}

///////////////////////////////////////////////////
// 从单个 buoyancy 配置读取 link、相对位姿和形状；缺少必要项时抛异常，由插件逐项捕获。
void BuoyancyObject::Load(const physics::ModelPtr model,
    const sdf::ElementPtr elem)
{
  // parse link
  if (elem->HasElement("link_name"))
  {
    this->linkName = elem->GetElement("link_name")->Get<std::string>();
    physics::LinkPtr link = model->GetLink(linkName);
    if (!link)
    {
      throw ParseException("link_name", "invalid link name");
    }
    this->linkId = link->GetId();
  }
  else
  {
    throw ParseException("link_name", "missing element");
  }

  // parse pose (optional)
  if (elem->HasElement("pose"))
  {
    this->pose = elem->GetElement("pose")->Get<ignition::math::Pose3d>();
  }

  // parse geometry
  if (elem->HasElement("geometry"))
  {
    sdf::ElementPtr geometry = elem->GetElement("geometry");
    try
    {
      this->shape = std::move(ShapeVolume::makeShape(geometry));
    }
    catch(...)
    {
      throw;
    }
  }
  else
  {
    throw ParseException("geometry", "missing element");
  }
}

//////////////////////////////////////////////////
std::string BuoyancyObject::Disp()
{
  std::stringstream ss;
  ss << "Buoyancy object\n"
      << "\tlink: " << linkName << "[" << linkId << "]\n"
      << "\tpose: " << pose << '\n'
      << "\tgeometry " << shape->Display() << '\n'
      << "\tmass " << mass;
  return ss.str();
}

/////////////////////////////////////////////////
BuoyancyPlugin::BuoyancyPlugin()
  : fluidDensity(997),
    fluidLevel(0.0),
    linearDrag(0.0),
    angularDrag(0.0),
    waveModelName(""),
    lastSimTime(0.0)
{
}

/////////////////////////////////////////////////
// 读取流体密度 kg/m³、平均水面高度 m 和阻尼系数；同一 link 可关联多个浮力几何体。
void BuoyancyPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != nullptr, "Received NULL model pointer");
  GZ_ASSERT(_sdf != nullptr, "Received NULL SDF pointer");

  // Capture the model and world pointers.
  this->model = _model;
  this->world = this->model->GetWorld();

  // Capture the wave model
  if (_sdf->HasElement("wave_model"))
  {
    this->waveModelName = _sdf->Get<std::string>("wave_model");
  }
  this->waveParams = nullptr;

  if (_sdf->HasElement("fluid_density"))
  {
    this->fluidDensity = _sdf->Get<double>("fluid_density");
  }
  if (_sdf->HasElement("fluid_level"))
  {
    this->fluidLevel = _sdf->Get<double>("fluid_level");
  }
  if (_sdf->HasElement("linear_drag"))
  {
    this->linearDrag = _sdf->Get<double>("linear_drag");
  }
  if (_sdf->HasElement("angular_drag"))
  {
    this->angularDrag = _sdf->Get<double>("angular_drag");
  }

  if (_sdf->HasElement("buoyancy"))
  {
    gzmsg << "Found buoyancy element(s), looking at each element..."
          << std::endl;
    for (sdf::ElementPtr buoyancyElem = _sdf->GetElement("buoyancy");
        buoyancyElem;
        buoyancyElem = buoyancyElem->GetNextElement("buoyancy"))
    {
      try
      {
        BuoyancyObject buoyObj = BuoyancyObject();
        buoyObj.Load(_model, buoyancyElem);

        // add link to linkMap if it is not in the map
        if (this->linkMap.find(buoyObj.linkId) == this->linkMap.end())
        {
          this->linkMap[buoyObj.linkId] = _model->GetLink(buoyObj.linkName);
          // initialize link height
          if (!this->waveModelName.empty())
          {
            this->linkHeights[linkMap[buoyObj.linkId]] = this->fluidLevel;
          }
          else
          {
            this->linkHeights[linkMap[buoyObj.linkId]] = 0;
          }
          // initialize link height velocity
          this->linkHeightDots[linkMap[buoyObj.linkId]] = 0;
        }

        // get mass
        #if GAZEBO_MAJOR_VERSION >= 8
          buoyObj.mass = this->linkMap[buoyObj.linkId]->GetInertial()->Mass();
        #else
          buoyObj.mass =
              this->linkMap[buoyObj.linkId]->GetInertial()->GetMass();
        #endif

        // add buoyancy object to list and display stats
        gzmsg << buoyObj.Disp() << std::endl;
        buoyancyObjects.push_back(std::move(buoyObj));
      }
      catch(const std::exception& e)
      {
        gzwarn << e.what() << std::endl;
      }
    }
  }

  // Initialize sim time memory
  #if GAZEBO_MAJOR_VERSION >= 8
    this->lastSimTime =  this->world->SimTime().Double();
  #else
    this->lastSimTime =  this->world->GetSimTime().Double();
  #endif
}

/////////////////////////////////////////////////
// 保存物理更新事件连接，使 OnUpdate 在每个仿真步被调用。
void BuoyancyPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&BuoyancyPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
// 先更新各 link 所在位置的波高，再逐形状求浸没体积；这里用局部水平水面近似波面。
void BuoyancyPlugin::OnUpdate()
{
  // update wave height if wave model specified
  if (!this->waveModelName.empty())
  {
    // If we haven't yet, retrieve the wave parameters from ocean model plugin.
    if (!this->waveParams)
    {
      gzmsg << "usv_gazebo_dynamics_plugin: waveParams is null. "
            << "Trying to get wave parameters from ocean model" << std::endl;
      this->waveParams = WavefieldModelPlugin::GetWaveParams(
          this->world, this->waveModelName);
    }

    #if GAZEBO_MAJOR_VERSION >= 8
      double simTime = this->world->SimTime().Double();
    #else
      double simTime = this->world->GetSimTime().Double();
    #endif

    // 时间差单位为秒；后面的波高差分要求 dt 非零，当前实现未在此作零步长检查。
    double dt = simTime - this->lastSimTime;
    this->lastSimTime = simTime;

    // get wave height for each link
    for (auto& link : this->linkMap)
    {
      auto linkPtr = link.second;
      #if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d linkFrame = linkPtr->WorldPose();
      #else
        ignition::math::Pose3d linkFrame = linkPtr->GetWorldPose().Ign();
      #endif

      // Compute the wave displacement at the centre of the link frame.
      // Wave field height at the link, relative to the mean water level.
      // 返回相对平均水面的波高 m；配置了波场时，这里要求 waveParams 已成功获取。
      double waveHeight = WavefieldSampler::ComputeDepthSimply(
          *waveParams, linkFrame.Pos(), simTime);

      this->linkHeightDots[linkPtr] =
          (waveHeight - this->linkHeights[linkPtr]) / dt;
      this->linkHeights[linkPtr] = waveHeight;
    }
  }

  for (auto& buoyancyObj : this->buoyancyObjects)
  {
    auto link = this->linkMap[buoyancyObj.linkId];
    #if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d linkFrame = link->WorldPose();
    #else
        ignition::math::Pose3d linkFrame = link->GetWorldPose().Ign();
    #endif
    // 将 link 的世界位姿与浮力形状相对位姿相乘，得到形状在世界坐标系中的位姿。
    linkFrame = linkFrame * buoyancyObj.pose;

    auto submergedVolume = buoyancyObj.shape->CalculateVolume(linkFrame,
        this->linkHeights[link] + this->fluidLevel);

    GZ_ASSERT(submergedVolume.volume >= 0,
        "Non-positive volume found in volume properties!");

    // calculate buoyancy and drag forces
    // 忽略小于阈值的浸没体积，避免很薄水下切片引起不稳定的力与质心计算。
    if (submergedVolume.volume > 1e-6)
    {
      // By Archimedes' principle,
      // buoyancy = -(mass*gravity)*fluid_density/object_density
      // object_density = mass/volume, so the mass term cancels.
      // 阿基米德浮力为 -ρVg，体积 m³、密度 kg/m³，输出世界坐标系力 N。
      ignition::math::Vector3d buoyancy = -this->fluidDensity
          * submergedVolume.volume * model->GetWorld()->Gravity();

      #if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Vector3d linVel = link->WorldLinearVel();
        ignition::math::Vector3d angVel = link->RelativeAngularVel();
      #else
        ignition::math::Vector3d linVel= link->GetWorldLinearVel().Ign();
        ignition::math::Vector3d angVel = link->GetRelativeAngularVel().Ign();
      #endif

      // partial mass = total_mass * submerged_vol / total_vol
      // 以浸没体积分数估计参与阻尼的质量；平动阻尼比较水面垂向速度和 link 世界速度。
      float partialMass = buoyancyObj.mass * submergedVolume.volume
          / buoyancyObj.shape->volume;

      // drag (based on Exact Buoyancy for Polyhedra by Eric Catto)
      // linear drag
      ignition::math::Vector3d relVel =
          ignition::math::Vector3d(0, 0, this->linkHeightDots[link]) - linVel;
      ignition::math::Vector3d dragForce = linearDrag * partialMass * relVel;
      buoyancy += dragForce;
      // 浮力叠加平动阻尼后只截断向下的 Z 分量，水平阻尼分量仍保留。
      if (buoyancy.Z() < 0.0)
      {
        buoyancy.Z() = 0.0;
      }
      // apply force
      link->AddForceAtWorldPosition(buoyancy, submergedVolume.centroid);

      // drag torque
      // 角阻尼使用 link 本体系角速度和形状平均长度平方，最终以本体系力矩施加。
      double averageLength2 = ::pow(buoyancyObj.shape->averageLength, 2);
      ignition::math::Vector3d dragTorque = (-partialMass * angularDrag
          * averageLength2) * angVel;
      link->AddRelativeTorque(dragTorque);
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(BuoyancyPlugin)
