/*

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the usv_gazebo_plugins package,
known as this Package.

This Package free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This Package s distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this package.  If not, see <http://www.gnu.org/licenses/>.

*/

// 接收各推进器 Float32 推力与桨角指令，按选定曲线映射为推力 N 并沿发动机当前朝向施加；侧推即横置的第三台推进器。
#include <chrono>
#include <functional>

#include "usv_gazebo_plugins/wamv_usv_gazebo_thrust_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
WamvThruster::WamvThruster(WamvUsvThrust* _parent)
{
  // Set some defaults
  this->cmdTopic = "thruster_default_cmdTopic";
  this->angleTopic = "thruster_default_angleTopic";
  this->maxCmd = 1.0;
  this->maxForceFwd = 250.0;
  this->maxForceRev = -250.0;
  this->maxAngle = M_PI / 2;
  this->mappingType = 0;
  this->enableAngle = true;
  this->plugin = _parent;

  // Initialize some things
  this->currCmd = 0.0;
  this->currAngle = 0.0;
  this->lastCmdTime = this->plugin->world->SimTime();
  this->lastAngleTime = this->plugin->world->SimTime();
}

//////////////////////////////////////////////////
void WamvThruster::OnThrustCmd(const std_msgs::msg::Float32::SharedPtr _msg)
{
  // When we get a new thrust command!
  RCLCPP_DEBUG_STREAM(this->plugin->rosnode->get_logger(),
                      "New thrust command! " << _msg->data);
  std::lock_guard<std::mutex> lock(this->plugin->mutex);
  this->lastCmdTime = this->plugin->world->SimTime();
  this->currCmd = _msg->data;
}

//////////////////////////////////////////////////
void WamvThruster::OnAngleCmd(const std_msgs::msg::Float32::SharedPtr _msg)
{
  // When we get a new angle command!
  RCLCPP_DEBUG_STREAM(this->plugin->rosnode->get_logger(),
                      "New angle command! " << _msg->data);
  if (!this->enableAngle)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(this->plugin->mutex);
  this->lastAngleTime = this->plugin->world->SimTime();
  this->currAngle = _msg->data;
}

//////////////////////////////////////////////////
double WamvUsvThrust::SdfParamDouble(sdf::ElementPtr _sdfPtr, const std::string& _paramName,
                                      const double _defaultVal) const
{
  if (!_sdfPtr->HasElement(_paramName))
  {
    RCLCPP_INFO_STREAM(this->rosnode->get_logger(),
                       "Parameter <" << _paramName
                                     << "> not found: "
                                        "Using default value of <"
                                     << _defaultVal << ">.");
    return _defaultVal;
  }

  double val = _sdfPtr->Get<double>(_paramName);
  RCLCPP_DEBUG_STREAM(this->rosnode->get_logger(),
                      "Parameter found - setting <" << _paramName << "> to <" << val << ">.");
  return val;
}

//////////////////////////////////////////////////
void WamvUsvThrust::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  RCLCPP_DEBUG(this->rosnode ? this->rosnode->get_logger() : rclcpp::get_logger("wamv_usv_gazebo_thrust_plugin"),
               "Loading wamv_usv_gazebo_thrust_plugin");
  this->model = _parent;
  this->world = this->model->GetWorld();

  // Create the ROS 2 node first so that logging is available.
  std::string nodeNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    nodeNamespace = _sdf->Get<std::string>("robotNamespace");
  }
  this->rosnode = std::make_shared<rclcpp::Node>(
      "wamv_usv_gazebo_thrust_plugin", nodeNamespace);

  auto logger = this->rosnode->get_logger();

  // Get parameters from SDF
  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", 1.0);

  RCLCPP_DEBUG_STREAM(logger, "Loading thrusters from SDF");
  // For each thruster
  int thrusterCounter = 0;
  if (_sdf->HasElement("wamv_thruster"))
  {
    sdf::ElementPtr thrusterSDF = _sdf->GetElement("wamv_thruster");
    while (thrusterSDF)
    {
      // Instatiate
      WamvThruster thruster(this);

      RCLCPP_DEBUG_STREAM(logger, "WamvThruster #" << thrusterCounter);

      // Find link by name in SDF
      if (thrusterSDF->HasElement("linkName"))
      {
        std::string linkName = thrusterSDF->Get<std::string>("linkName");
        thruster.link = this->model->GetLink(linkName);
        if (thruster.link == nullptr)
        {
          RCLCPP_ERROR_STREAM(logger,
                              "Could not find a link by the name <" << linkName << "> in the model!");
        }
        else
        {
          RCLCPP_DEBUG_STREAM(logger, "WamvThruster added to link <" << linkName << ">");
        }
      }
      else
      {
        RCLCPP_ERROR_STREAM(logger, "Please specify a link name for each thruster!");
      }

      // Parse out propellor joint name
      if (thrusterSDF->HasElement("propJointName"))
      {
        std::string propName = thrusterSDF->GetElement("propJointName")->Get<std::string>();
        thruster.propJoint = this->model->GetJoint(propName);
        if (thruster.propJoint == nullptr)
        {
          RCLCPP_WARN_STREAM(logger,
                             "Could not find a propellor joint by the name of <" << propName << "> in the model!");
        }
        else
        {
          RCLCPP_DEBUG_STREAM(logger, "Propellor joint <" << propName << "> added to thruster");
        }
      }
      else
      {
        RCLCPP_WARN_STREAM(logger, "No propJointName SDF parameter for thruster #" << thrusterCounter);
      }

      // Parse out engine joint name (steerable thruster)
      if (thrusterSDF->HasElement("engineJointName"))
      {
        std::string engineName = thrusterSDF->GetElement("engineJointName")->Get<std::string>();
        thruster.engineJoint = this->model->GetJoint(engineName);
        if (thruster.engineJoint == nullptr)
        {
          RCLCPP_WARN_STREAM(logger,
                             "Could not find an engine joint by the name of <" << engineName
                             << "> in the model!");
        }
        else
        {
          RCLCPP_DEBUG_STREAM(logger, "Engine joint <" << engineName << "> added to thruster");
        }
      }
      else
      {
        RCLCPP_WARN_STREAM(logger, "No engineJointName SDF parameter for thruster #" << thrusterCounter);
      }

      // Parse individual thruster SDF parameters
      thruster.maxCmd = this->SdfParamDouble(thrusterSDF, "maxCmd", 1.0);
      thruster.maxForceFwd = this->SdfParamDouble(thrusterSDF, "maxForceFwd", 250.0);
      thruster.maxForceRev = this->SdfParamDouble(thrusterSDF, "maxForceRev", -250.0);
      thruster.maxAngle = this->SdfParamDouble(thrusterSDF, "maxAngle", M_PI / 2);
      if (thrusterSDF->HasElement("enableAngle"))
      {
        thruster.enableAngle = thrusterSDF->Get<bool>("enableAngle");
      }
      if (thrusterSDF->HasElement("mappingType"))
      {
        thruster.mappingType = thrusterSDF->Get<int>("mappingType");
        RCLCPP_DEBUG_STREAM(logger,
                            "Parameter found - setting <mappingType> to <" << thruster.mappingType << ">.");
      }
      else
      {
        thruster.mappingType = 0;
        RCLCPP_INFO_STREAM(logger,
                           "Parameter <mappingType> not found: "
                           "Using default value of <"
                           << thruster.mappingType << ">.");
      }

      // Parse for subscription topic
      if (thrusterSDF->HasElement("cmdTopic"))
      {
        thruster.cmdTopic = thrusterSDF->Get<std::string>("cmdTopic");
      }
      else
      {
        RCLCPP_ERROR_STREAM(logger,
                            "Please specify a cmdTopic (for ROS subscription) "
                            "for each thruster!");
      }

      // Parse for angle subscription topic
      if (thrusterSDF->HasElement("angleTopic"))
      {
        thruster.angleTopic = thrusterSDF->Get<std::string>("angleTopic");
      }
      else
      {
        RCLCPP_WARN_STREAM(logger,
                           "Please specify an angleTopic (for ROS subscription) "
                           "for each thruster!");
      }

      // Push to vector and increment
      this->thrusters.push_back(thruster);
      thrusterSDF = thrusterSDF->GetNextElement("wamv_thruster");
      thrusterCounter++;
    }
  }
  else
  {
    RCLCPP_WARN_STREAM(logger, "No 'wamv_thruster' tags in description - how will you move?");
  }
  RCLCPP_DEBUG_STREAM(logger, "Found " << thrusterCounter << " thrusters");

  // Arm the engine joints as ODE velocity motors so the steerable thrusters
  // can be held at their commanded angle (classic-gazebo equivalent of the
  // gz::sim JointPositionController with velocity commands used by VRX).
  // Teleporting the joint with SetPosition() every tick instead freezes the
  // whole model (velocities get reset each physics update).
  for (auto& thruster : this->thrusters)
  {
    if (thruster.engineJoint != nullptr && thruster.enableAngle)
    {
      thruster.engineJoint->SetParam("fmax", 0, static_cast<double>(50.0));
    }
  }

  // Advertise joint state publisher to view propellers in rviz. Each
  // thruster contributes its engine joint (if any) and its propeller joint.
  this->jointStatePub =
      this->rosnode->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  for (size_t i = 0; i < this->thrusters.size(); ++i)
  {
    if (this->thrusters[i].engineJoint != nullptr)
    {
      this->jointStateMsg.name.push_back(this->thrusters[i].engineJoint->GetName());
      this->jointStateMsg.position.push_back(0.0);
      this->jointStateMsg.velocity.push_back(0.0);
      this->jointStateMsg.effort.push_back(0.0);
    }
    if (this->thrusters[i].propJoint != nullptr)
    {
      this->jointStateMsg.name.push_back(this->thrusters[i].propJoint->GetName());
      this->jointStateMsg.position.push_back(0.0);
      this->jointStateMsg.velocity.push_back(0.0);
      this->jointStateMsg.effort.push_back(0.0);
    }
  }

  for (size_t i = 0; i < this->thrusters.size(); ++i)
  {
    // Subscribe to commands for each thruster.
    this->thrusters[i].cmdSub =
        this->rosnode->create_subscription<std_msgs::msg::Float32>(
            this->thrusters[i].cmdTopic, 1,
            [this, i](const std_msgs::msg::Float32::SharedPtr msg) {
              this->thrusters[i].OnThrustCmd(msg);
            });

    // Subscribe to angle commands for each thruster (if enabled).
    if (this->thrusters[i].enableAngle)
    {
      this->thrusters[i].angleSub =
          this->rosnode->create_subscription<std_msgs::msg::Float32>(
              this->thrusters[i].angleTopic, 1,
              [this, i](const std_msgs::msg::Float32::SharedPtr msg) {
                this->thrusters[i].OnAngleCmd(msg);
              });
    }
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&WamvUsvThrust::Update, this));
}

//////////////////////////////////////////////////
double WamvUsvThrust::ScaleThrustCmd(const double _cmd, const double _maxCmd, const double _maxPos,
                                      const double _maxNeg) const
{
  double val = 0.0;
  if (_cmd >= 0.0)
  {
    val = _cmd / _maxCmd * _maxPos;
    val = std::min(val, _maxPos);
  }
  else
  {
    double absMaxNeg = std::abs(_maxNeg);
    val = _cmd / _maxCmd * absMaxNeg;
    val = std::max(val, -1.0 * absMaxNeg);
  }
  return val;
}

//////////////////////////////////////////////////
double WamvUsvThrust::Glf(const double _x, const float _A, const float _K, const float _B, const float _v,
                           const float _C, const float _M) const
{
  return _A + (_K - _A) / (pow(_C + exp(-_B * (_x - _M)), 1.0 / _v));
}

//////////////////////////////////////////////////
double WamvUsvThrust::GlfThrustCmd(const double _cmd, const double _maxPos, const double _maxNeg) const
{
  double val = 0.0;
  if (_cmd > 0.01)
  {
    val = this->Glf(_cmd, 0.01f, 59.82f, 5.0f, 0.38f, 0.56f, 0.28f);
    val = std::min(val, _maxPos);
  }
  else if (_cmd < 0.01)
  {
    val = this->Glf(_cmd, -199.13f, -0.09f, 8.84f, 5.34f, 0.99f, -0.57f);
    val = std::max(val, _maxNeg);
  }
  return val;
}

//////////////////////////////////////////////////
// Thruster mapping.
double WamvUsvThrust::Square(const double _x, const double _k) const
{
  return _k * abs(_x) * _x;
}

//////////////////////////////////////////////////
double WamvUsvThrust::SquareThrustCmd(const double _cmd, const double _maxPos, const double _maxNeg) const
{
  double val = 0.0;
  if (_cmd > 0.01)
  {
    val = this->Square(_cmd, 17.86f);
    val = std::min(val, _maxPos);
  }
  else if (_cmd < 0.01)
  {
    val = this->Square(_cmd, 10.86f);
    val = std::max(val, _maxNeg);
  }
  return val;
}

//////////////////////////////////////////////////
void WamvUsvThrust::UpdateEngineJoint(WamvThruster& _thruster)
{
  if (!_thruster.engineJoint || !_thruster.enableAngle)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(this->mutex);
  // Steer the engine towards the commanded angle with a P velocity servo
  // (ODE motor). Zero corresponds to the rest orientation declared in the
  // URDF joint origin (e.g. the lateral thruster of the "T" layout points
  // sideways at zero), so with no angle commands the engines hold their
  // rest orientation.
  const double target = std::min(std::max(_thruster.currAngle, -_thruster.maxAngle), _thruster.maxAngle);
  const double error = target - _thruster.engineJoint->Position(0);
  const double cmdVel = std::min(std::max(2.0 * error, -1.0), 1.0);
  _thruster.engineJoint->SetParam("vel", 0, cmdVel);
}

//////////////////////////////////////////////////
void WamvUsvThrust::Update()
{
  rclcpp::spin_some(this->rosnode);

  common::Time now = this->world->SimTime();

  for (size_t i = 0; i < this->thrusters.size(); ++i)
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      // Enforce command timeout
      double dtc = (now - this->thrusters[i].lastCmdTime).Double();
      if (dtc > this->cmdTimeout && this->cmdTimeout > 0.0)
      {
        this->thrusters[i].currCmd = 0.0;
        RCLCPP_DEBUG_STREAM(this->rosnode->get_logger(), "[" << i << "] Cmd Timeout");
      }

      // Enforce angle command timeout
      double dta = (now - this->thrusters[i].lastAngleTime).Double();
      if (dta > this->cmdTimeout && this->cmdTimeout > 0.0)
      {
        this->thrusters[i].currAngle = 0.0;
      }

      // Apply the thrust mapping
      ignition::math::Vector3d tforcev(0, 0, 0);
      switch (this->thrusters[i].mappingType)
      {
        case 0:
          tforcev.X() = this->ScaleThrustCmd(this->thrusters[i].currCmd, this->thrusters[i].maxCmd,
                                             this->thrusters[i].maxForceFwd, this->thrusters[i].maxForceRev);
          break;
        case 1:
          tforcev.X() = this->GlfThrustCmd(this->thrusters[i].currCmd, this->thrusters[i].maxForceFwd,
                                           this->thrusters[i].maxForceRev);
          break;
        case 2:
          tforcev.X() = this->SquareThrustCmd(this->thrusters[i].currCmd, this->thrusters[i].maxForceFwd,
                                              this->thrusters[i].maxForceRev);
          break;
        default:
          RCLCPP_FATAL_STREAM(this->rosnode->get_logger(),
                              "Cannot use mappingType=" << this->thrusters[i].mappingType);
          break;
      }

      // Apply force for each thruster
      this->thrusters[i].link->AddLinkForce(tforcev);

      // Spin the propellers
      this->SpinPropeller(this->thrusters[i].propJoint, this->thrusters[i].currCmd);
    }

    // Update the steerable engine joint
    this->UpdateEngineJoint(this->thrusters[i]);
  }

  // Publish the propeller/engine joint state
  this->jointStateMsg.header.stamp = this->rosnode->now();
  this->jointStatePub->publish(this->jointStateMsg);
}

//////////////////////////////////////////////////
void WamvUsvThrust::SpinPropeller(physics::JointPtr& _propeller, const double _input)
{
  if (!_propeller)
    return;

  const double kMinInput = 0.1;
  const double kMaxInput = 1.0;
  const double kMaxEffort = 2.0;
  double effort = 0.0;

  if (std::abs(_input) > kMinInput)
    effort = (_input / kMaxInput) * kMaxEffort;

  _propeller->SetForce(0, effort);

  // Get index in joint state message for this propeller
  int index = -1;
  for (size_t i = 0; i < this->jointStateMsg.name.size(); ++i)
  {
    if (_propeller->GetName() == this->jointStateMsg.name[i])
    {
      index = static_cast<int>(i);
      break;
    }
  }
  if (index < 0)
  {
    RCLCPP_WARN(this->rosnode->get_logger(),
                "Propeller %s cannot be associated with a joint, "
                "skipping message.",
                _propeller->GetName().c_str());
    return;
  }

  // Set position, velocity, and effort of joint from gazebo
  ignition::math::Angle position = _propeller->Position(0);
  position.Normalize();
  this->jointStateMsg.position[index] = position.Radian();
  this->jointStateMsg.velocity[index] = _propeller->GetVelocity(0);
  this->jointStateMsg.effort[index] = effort;
}

GZ_REGISTER_MODEL_PLUGIN(WamvUsvThrust);
