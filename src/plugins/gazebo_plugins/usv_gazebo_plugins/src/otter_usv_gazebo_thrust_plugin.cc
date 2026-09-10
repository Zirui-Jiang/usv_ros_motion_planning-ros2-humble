/*

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the usv_gazebo_dynamics_plugin package,
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

#include <chrono>
#include <functional>

#include "usv_gazebo_plugins/otter_usv_gazebo_thrust_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
OtterThruster::OtterThruster(OtterUsvThrust* _parent)
{
  // Set some defaults
  this->cmdTopic = "thruster_default_cmdTopic";
  this->maxCmd = 1.0;
  this->maxForceFwd = 100.0;
  this->maxForceRev = -100.0;
  this->mappingType = 0;
  this->plugin = _parent;

  // Initialize some things
  this->currCmd = 0.0;
  this->lastCmdTime = this->plugin->world->SimTime();
}

//////////////////////////////////////////////////
void OtterThruster::OnThrustCmd(const std_msgs::msg::Float32::SharedPtr _msg)
{
  // When we get a new thrust command!
  RCLCPP_DEBUG_STREAM(this->plugin->rosnode->get_logger(),
                      "New thrust command! " << _msg->data);
  std::lock_guard<std::mutex> lock(this->plugin->mutex);
  this->lastCmdTime = this->plugin->world->SimTime();
  this->currCmd = _msg->data;
}

//////////////////////////////////////////////////
double OtterUsvThrust::SdfParamDouble(sdf::ElementPtr _sdfPtr, const std::string& _paramName,
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
void OtterUsvThrust::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  RCLCPP_DEBUG(this->rosnode ? this->rosnode->get_logger() : rclcpp::get_logger("otter_usv_gazebo_thrust_plugin"),
               "Loading usv_gazebo_thrust_plugin");
  this->model = _parent;
  this->world = this->model->GetWorld();

  // Create the ROS 2 node first so that logging is available.
  std::string nodeNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    nodeNamespace = _sdf->Get<std::string>("robotNamespace");
  }
  this->rosnode = std::make_shared<rclcpp::Node>(
      "otter_usv_gazebo_thrust_plugin", nodeNamespace);

  auto logger = this->rosnode->get_logger();

  // Get parameters from SDF
  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", 1.0);

  RCLCPP_DEBUG_STREAM(logger, "Loading thrusters from SDF");
  // For each thruster
  int thrusterCounter = 0;
  if (_sdf->HasElement("otter_thruster"))
  {
    sdf::ElementPtr thrusterSDF = _sdf->GetElement("otter_thruster");
    while (thrusterSDF)
    {
      // Instatiate
      OtterThruster thruster(this);

      RCLCPP_DEBUG_STREAM(logger, "OtterThruster #" << thrusterCounter);

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
          RCLCPP_DEBUG_STREAM(logger, "OtterThruster added to link <" << linkName << ">");
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

      // Parse individual thruster SDF parameters
      thruster.maxCmd = this->SdfParamDouble(thrusterSDF, "maxCmd", 1.0);
      thruster.maxForceFwd = this->SdfParamDouble(thrusterSDF, "maxForceFwd", 250.0);
      thruster.maxForceRev = this->SdfParamDouble(thrusterSDF, "maxForceRev", -100.0);
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

      // Push to vector and increment
      this->thrusters.push_back(thruster);
      thrusterSDF = thrusterSDF->GetNextElement("otter_thruster");
      thrusterCounter++;
    }
  }
  else
  {
    RCLCPP_WARN_STREAM(logger, "No 'thruster' tags in description - how will you move?");
  }
  RCLCPP_DEBUG_STREAM(logger, "Found " << thrusterCounter << " thrusters");

  // Advertise joint state publisher to view propellers in rviz
  // TODO: consider throttling joint_state pub for performance
  // (every OnUpdate may be too frequent).
  this->jointStatePub =
      this->rosnode->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  this->jointStateMsg.name.resize(thrusters.size());
  this->jointStateMsg.position.resize(thrusters.size());
  this->jointStateMsg.velocity.resize(thrusters.size());
  this->jointStateMsg.effort.resize(thrusters.size());

  for (size_t i = 0; i < this->thrusters.size(); ++i)
  {
    // Prefill the joint state message with the propeller joint.
    this->jointStateMsg.name[i] = this->thrusters[i].propJoint->GetName();

    // Subscribe to commands for each thruster.
    this->thrusters[i].cmdSub =
        this->rosnode->create_subscription<std_msgs::msg::Float32>(
            this->thrusters[i].cmdTopic, 1,
            [this, i](const std_msgs::msg::Float32::SharedPtr msg) {
              this->thrusters[i].OnThrustCmd(msg);
            });
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&OtterUsvThrust::Update, this));
}

//////////////////////////////////////////////////
double OtterUsvThrust::ScaleThrustCmd(const double _cmd, const double _maxCmd, const double _maxPos,
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
double OtterUsvThrust::Glf(const double _x, const float _A, const float _K, const float _B, const float _v,
                           const float _C, const float _M) const
{
  return _A + (_K - _A) / (pow(_C + exp(-_B * (_x - _M)), 1.0 / _v));
}

//////////////////////////////////////////////////
double OtterUsvThrust::GlfThrustCmd(const double _cmd, const double _maxPos, const double _maxNeg) const
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
// Otter thruster mapping.
double OtterUsvThrust::Square(const double _x, const double _k) const
{
  return _k * abs(_x) * _x;
}

//////////////////////////////////////////////////
double OtterUsvThrust::SquareThrustCmd(const double _cmd, const double _maxPos, const double _maxNeg) const
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
void OtterUsvThrust::Update()
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
  }

  // Publish the propeller joint state
  this->jointStateMsg.header.stamp = this->rosnode->now();
  this->jointStatePub->publish(this->jointStateMsg);
}

//////////////////////////////////////////////////
void OtterUsvThrust::SpinPropeller(physics::JointPtr& _propeller, const double _input)
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
  for (size_t i = 0; i < this->thrusters.size(); ++i)
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

GZ_REGISTER_MODEL_PLUGIN(OtterUsvThrust);
