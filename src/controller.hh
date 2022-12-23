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

#ifndef SOT_UNIVERSAL_ROBOT_CONTROLLER_HH
#define SOT_UNIVERSAL_ROBOT_CONTROLLER_HH

#include <sot/core/fwd.hh>
#include <sot/core/abstract-sot-external-interface.hh>

class SoTRobotArmDevice;

using std::string;
using std::map;
using dynamicgraph::shared_ptr;
using dynamicgraph::sot::AbstractSotExternalInterface;
using dynamicgraph::sot::Device;
using dynamicgraph::sot::ControlValues;
using dynamicgraph::sot::SensorValues;


class SoTRobotArmController: public AbstractSotExternalInterface
{
 public:

  static const string LOG_PYTHON;

  SoTRobotArmController();
  SoTRobotArmController(const char robotName[]);
  SoTRobotArmController(string robotName);
  virtual ~SoTRobotArmController();

  void setupSetSensors(map<string,SensorValues> &sensorsIn);

  void nominalSetSensors(map<string,SensorValues> &sensorsIn);

  void cleanupSetSensors(map<string, SensorValues> &sensorsIn);

  void getControl(map<string, ControlValues> &anglesOut);

  void setNoIntegration(void);
  void setSecondOrderIntegration(void);

  /// Embedded python interpreter accessible via Corba/ros
  shared_ptr<dynamicgraph::Interpreter> interpreter_;

 protected:
  // Update output port with the control computed from the
  // dynamic graph.
  void updateRobotState(std::vector<double> &anglesIn);

  void runPython(std::ostream& file,
		 const string& command,
		 dynamicgraph::Interpreter& interpreter);

  void startupPython(const std::string& robotType);

  void init();

  SoTRobotArmDevice* device_;
}; // class SoTRobotArmController

#endif // SOT_UNIVERSAL_ROBOT_CONTROLLER_HH
