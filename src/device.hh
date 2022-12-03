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

class SoTRobotArmDevice : public dynamicgraph::sot::Device
{
public:
  static const std::string CLASS_NAME;
  static const double TIMESTEP_DEFAULT;

  virtual const std::string& getClassName () const
  {
    return CLASS_NAME;
  }

  SoTRobotArmDevice(std::string RobotName);
  virtual ~SoTRobotArmDevice();

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
}; // class SoTRobotArmDevice

/// Entity that converts a vector of dimension 7 to a vector of dimension 8
///
/// Useful for Franka robot where device.state is of dimension 7 while
/// dynamic.position expects a vector of dimension 8.
/// The last component is set to 0
class DeviceToDynamic : public dynamicgraph::Entity
{
public:
  static const std::string CLASS_NAME;
  DeviceToDynamic(const std::string& name) :
    dynamicgraph::Entity(name), sinSIN(0x0, "DeviceToDynamic("+name+")::input(vector)::sin"),
    soutSOUT(boost::bind(&DeviceToDynamic::compute, this, _1, _2),
             sinSIN, "DeviceToDynamic("+name+")::ouput(vector)::sout")
  {
    signalRegistration(sinSIN << soutSOUT);
  }
private:
  dynamicgraph::Vector& compute(dynamicgraph::Vector& res, int time)
  {
    res.head<7>() = sinSIN(time);
    res[7] = 0;
    return res;
  }
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> sinSIN;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> soutSOUT;
};

#endif // SOT_UNIVERSAL_ROBOT_DEVICE_HH
