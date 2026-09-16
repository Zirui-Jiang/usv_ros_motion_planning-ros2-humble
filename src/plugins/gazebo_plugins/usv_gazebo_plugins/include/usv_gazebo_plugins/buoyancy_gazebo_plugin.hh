/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

// 浮力插件接口与状态：用多个几何体描述排水体积，各几何体绑定到模型的具体 link。
#ifndef USV_GAZEBO_PLUGINS_BUOYANCY_GAZEBO_PLUGIN_HH_
#define USV_GAZEBO_PLUGINS_BUOYANCY_GAZEBO_PLUGIN_HH_

#include <map>
#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "usv_gazebo_plugins/shape_volume.hh"

#include "wave_gazebo_plugins/Wavefield.hh"
#include "wave_gazebo_plugins/WavefieldEntity.hh"
#include "wave_gazebo_plugins/WavefieldModelPlugin.hh"

namespace gazebo
{
  namespace buoyancy
  {
    /// \brief A class for storing buoyancy object properties
    // 单个浮力对象保存 link 标识、相对位姿、质量和形状所有权；含 unique_ptr，因此禁用拷贝。
    class BuoyancyObject
    {
      /// \brief Default constructor
      public: BuoyancyObject();

      /// \brief Default move constructor
      public: BuoyancyObject(BuoyancyObject&& obj) noexcept; // NOLINT

      /// \brief No copy constructor
      public: BuoyancyObject(BuoyancyObject& obj) = delete;

      /// \brief Load buoyancy object from SDF
      /// @param model model associated with buoyancy object
      /// @param elem sdf for buoyancy element
      public: void Load(const physics::ModelPtr model,
                        const sdf::ElementPtr elem);

      /// \brief Display string for buoyancy object
      public: std::string Disp();

      /// \brief Associated link ID
      public: int linkId;

      /// \brief Associated link name
      public: std::string linkName;

      /// \brief Pose of buoyancy relative to link
      public: ignition::math::Pose3d pose;

      /// \brief Object mass (from inertial elem)
      public:double mass;

      /// \brief Buoyancy object's shape properties
      public: ::buoyancy::ShapeVolumePtr shape;
    };
  }  // end of buoyancy namespace

  /// \brief This plugin simulates buoyancy of an object in fluid.
  ///   <wave_model>:    Name of the wave model object (optional)
  ///
  ///   <fluid_density>: Sets the density of the fluid that surrounds the
  ///                    buoyant object [kg/m^3].
  ///                    This parameter is optional (default value 997 kg/m^3).
  ///
  ///   <fluid_level>:   The height of the fluid/air interface [m].
  ///                    This parameter is optional (default value 0 m).
  ///
  ///   <linear_drag>:   Linear drag coefficent [N/(m/s)].
  ///                    Translational drag implement as linear function
  ///                    of velocity.
  ///                    This parameter is optional.
  ///
  ///   <angular_drag>:  Angular drag coefficent [(Nm)/(rad/s)].
  ///                    Rotational drag implemented as linear function
  ///                    of velocity.
  ///                    This parameter is optional.
  ///
  ///   <buoyancy>:      Describes the volume properties
  ///                    For example:
  ///
  ///                    <buoyancy name="buoyancy1">
  ///                      <link_name>link</link_name>
  ///                      <geometry>
  ///                        ...
  ///                      </geometry>
  ///                    </buoyancy>
  ///
  ///     <link>:        Name of associated link element
  ///
  ///     <geometry>:    Geometry element specifying buoyancy object's
  ///                    volume properties.
  ///                    Supported shapes: box, sphere, cylinder
  // 加载时解析 SDF，初始化时连接事件，每步求波高、浸没体积和作用于浮心的合力。
  class BuoyancyPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: BuoyancyPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief The density of the fluid in which the object is submerged in
    /// kg/m^3. Defaults to 1000, the fluid density of water at 15 Celsius.
    protected: double fluidDensity;

    /// \brief The height of the fluid/air interface [m]. Defaults to 0.
    protected: double fluidLevel;

    /// \brief Linear drag coefficient. Defaults to 0.
    protected: double linearDrag;

    /// \brief Angular drag coefficient. Defaults to 0.
    protected: double angularDrag;

    /// \brief List of buoyancy objects for model
    protected: std::vector<buoyancy::BuoyancyObject> buoyancyObjects;

    /// \brief Map of <link ID, link pointer>
    // 按 link ID 复用 link 指针，避免同一刚体有多个浮力形状时重复查询模型。
    protected: std::map<int, gazebo::physics::LinkPtr> linkMap;

    /// \brief Pointer to base model
    protected: physics::ModelPtr model;

    /// \brief Pointer to the Gazebo world
    /// Retrieved when the model is loaded.
    protected: physics::WorldPtr world;

    /// \brief The name of the wave model
    protected: std::string waveModelName;

    /// \brief Map of water height at each link from previous timestep
    // 分别缓存每个 link 处的波高 m 与差分波面速度 m/s，供水相对速度阻尼计算。
    protected: std::map<gazebo::physics::LinkPtr, double> linkHeights;

    /// \brief Map of water velocity at each link
    protected: std::map<gazebo::physics::LinkPtr, double> linkHeightDots;

    /// \brief Previous update time
    protected: double lastSimTime;

    /// \brief The wave parameters.
    protected: std::shared_ptr<const asv::WaveParameters> waveParams;
  };
}

#endif
