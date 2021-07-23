// BSD 2-Clause License

// Copyright (c) 2021, CNRS

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SOT_UNIVERSAL_ROBOT_DEVICE_HH
#define SOT_UNIVERSAL_ROBOT_DEVICE_HH

#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/device.hh>

using std::string;
using std::map;

using dynamicgraph::sot::Device;
using dynamicgraph::sot::ControlValues;
using dynamicgraph::sot::SensorValues;

class SoTUniversalRobotDevice : public dynamicgraph::sot::Device
{
public:
  static const std::string CLASS_NAME;
  static const double TIMESTEP_DEFAULT;

  virtual const std::string& getClassName () const
  {
    return CLASS_NAME;
  }

  SoTUniversalRobotDevice(std::string RobotName);
  virtual ~SoTUniversalRobotDevice();

  void setSensors(std::map<std::string,SensorValues> &sensorsIn);

  void setupSetSensors(std::map<std::string,SensorValues> &sensorsIn);

  void nominalSetSensors(std::map<std::string,SensorValues> &sensorsIn);

  void cleanupSetSensors(std::map<std::string, SensorValues> &sensorsIn);

  void getControl(std::map<std::string, ControlValues> &anglesOut);

  /// \todo this should go into the parent class, in sot-core package
  void setTimeStep (double dt)
  {
    timestep_ = dt;
  }

protected:

  void setClosedLoop (const bool& closedLoop)
  {
    closedLoop_ = closedLoop;
  };

  /// Integrate control signal to update internal state.
  virtual void integrate(const double &dt);

  /// \brief Whether the control of the base should be expressed in odometry
  ///        frame of base frame.
  bool closedLoop_;

  /// \brief Previous robot configuration.
  dynamicgraph::Vector previousState_;

  /// proportional and derivative position-control gains
  dynamicgraph::Signal <dynamicgraph::Vector, int> p_gainsSOUT_;

  dynamicgraph::Signal <dynamicgraph::Vector, int> d_gainsSOUT_;

  /// Intermediate variables to avoid allocation during control
  dynamicgraph::Vector dgforces_;
  dynamicgraph::Vector dgRobotState_;
  dynamicgraph::Vector torques_;
  dynamicgraph::Vector p_gains_;
  dynamicgraph::Vector d_gains_;
}; // class SoTUniversalRobotDevice

#endif // SOT_UNIVERSAL_ROBOT_DEVICE_HH
