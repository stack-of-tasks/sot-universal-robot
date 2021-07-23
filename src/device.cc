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

#define ENABLE_RT_LOG
#include "device.hh"
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/real-time-logger.h>

#include <sot/core/debug.hh>

// Return true if it saturates.
inline bool saturateBounds(double &val, const double &lower,
                           const double &upper) {
  assert(lower <= upper);
  if (val < lower) {
    val = lower;
    return true;
  }
  if (upper < val) {
    val = upper;
    return true;
  }
  return false;
}

#define CHECK_BOUNDS(val, lower, upper, what)                                  \
  for (int i = 0; i < val.size(); ++i) {                                       \
    double old = val(i);                                                       \
    if (saturateBounds(val(i), lower(i), upper(i))) {                          \
      dgRTLOG() << "Robot " what " bound violation at DoF " << i <<            \
	": requested " << old << " but set " << val(i) << '\n';                \
    }                                                                          \
  }

#ifdef VP_DEBUG
class initLog {
public:
  initLog(void) { dynamicgraph::sot::DebugTrace::openFile(); }
};
initLog log_initiator;
#endif //#ifdef VP_DEBUG

const double SoTUniversalRobotDevice::TIMESTEP_DEFAULT = 0.001;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SoTUniversalRobotDevice,
				   "DeviceUniversalRobot");

SoTUniversalRobotDevice::SoTUniversalRobotDevice(std::string RobotName):
  Device(RobotName),
  closedLoop_ (false),
  previousState_ (),
  p_gainsSOUT_ ("StackOfTasks(" + RobotName + ")::output(vector)::p_gains"),
  d_gainsSOUT_ ("StackOfTasks(" + RobotName + ")::output(vector)::d_gains"),
  dgforces_ (6),
  torques_()
{
  timestep_ = TIMESTEP_DEFAULT;
  sotDEBUGIN(25) ;
  signalRegistration (p_gainsSOUT_ << d_gainsSOUT_);
  dynamicgraph::Vector data (3); data.setZero ();
  using namespace dynamicgraph::command;
  std::string docstring;
  /* Command increment. */
  docstring =
      "\n"
      "    Integrate dynamics for time step provided as input\n"
      "\n"
      "      take one floating point number as input\n"
      "\n";
  addCommand("increment",
             makeCommandVoid1((Device&)*this,
                              &Device::increment, docstring));

  docstring =
      "    Set the integration in closed loop\n"
      "\n"
      "      - Input: boolean\n"
      "\n";
  addCommand("setClosedLoop",
             makeCommandVoid1(*this,
                              &SoTUniversalRobotDevice::setClosedLoop,
			      docstring));
  sotDEBUGOUT(25);
}

SoTUniversalRobotDevice::~SoTUniversalRobotDevice()
{ }

void SoTUniversalRobotDevice::setSensors(map<string,SensorValues> &SensorsIn)
{
  sotDEBUGIN(25) ;
  map<string,SensorValues>::iterator it;
  int t = stateSOUT.getTime () + 1;
  bool setRobotState = false;

  it = SensorsIn.find("forces");
  if (it!=SensorsIn.end())
  {

    // Implements force recollection.
    const std::vector<double>& forcesIn = it->second.getValues();
    assert (std::div((int)forcesIn.size(), 6).rem == 0);
    int K = (int)forcesIn.size() / 6;
    for(int i=0;i<K;++i)
    {
      for(int j=0;j<6;++j)
        dgforces_(j) = forcesIn[i*6+j];
      forcesSOUT[i]->setConstant(dgforces_);
      forcesSOUT[i]->setTime (t);
    }
  }

  it = SensorsIn.find("motor-angles");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& anglesIn = it->second.getValues();
    dgRobotState_.resize (anglesIn.size ());
    for (unsigned i = 0; i < anglesIn.size(); ++i)
      dgRobotState_ (i) = anglesIn[i];
    setRobotState = true;
  }

  it = SensorsIn.find("torques");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& torques = SensorsIn["torques"].getValues();
    torques_.resize(torques.size());
    for(std::size_t i = 0; i < torques.size(); ++i)
      torques_ (i) = torques [i];
    pseudoTorqueSOUT.setConstant(torques_);
    pseudoTorqueSOUT.setTime(t);
  }

  it = SensorsIn.find("p_gains");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& p_gains = SensorsIn["p_gains"].getValues();
    p_gains_.resize(p_gains.size());
    for(std::size_t i = 0; i < p_gains.size(); ++i)
      p_gains_ (i) = p_gains[i];
    p_gainsSOUT_.setConstant(p_gains_);
    p_gainsSOUT_.setTime(t);
  }

  it = SensorsIn.find("d_gains");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& d_gains = SensorsIn["d_gains"].getValues();
    d_gains_.resize(d_gains.size());
    for(std::size_t i = 0; i < d_gains.size(); ++i)
      d_gains_ (i) = d_gains[i];
    d_gainsSOUT_.setConstant(d_gains_);
    d_gainsSOUT_.setTime(t);
  }

  if (setRobotState) {
    robotState_.setConstant(dgRobotState_);
    robotState_.setTime(t);
  }

  sotDEBUGOUT(25);
}

void SoTUniversalRobotDevice::setupSetSensors(map<string,SensorValues>&
					      SensorsIn)
{
  setSensors (SensorsIn);
  setState (robotState_);
}

void SoTUniversalRobotDevice::nominalSetSensors(map<string,SensorValues>&
						SensorsIn)
{
  setSensors (SensorsIn);
}


void SoTUniversalRobotDevice::cleanupSetSensors(map<string, SensorValues>&
						SensorsIn)
{
  setSensors (SensorsIn);
}

void SoTUniversalRobotDevice::getControl(map<string,ControlValues> &controlOut)
{
  sotDEBUGIN(25) ;
  std::vector<double> anglesOut;

  // Integrate control
  increment(timestep_);
  sotDEBUG (25) << "state = " << state_.transpose() << std::endl;
  previousState_ = state_;

  // Specify the joint values for the controller.
  anglesOut.resize(state_.size());

  for(unsigned int i=0; i < state_.size();++i)
    anglesOut[i] = state_(i);
  controlOut["control"].setValues(anglesOut);
  sotDEBUGOUT(25) ;
}

void SoTUniversalRobotDevice::integrate(const double &dt)
{
  using dynamicgraph::Vector;
  const Vector &controlIN = controlSIN.accessCopy();

  if (sanityCheck_ && controlIN.hasNaN()) {
    dgRTLOG() << "Device::integrate: Control has NaN values: " << '\n'
              << controlIN.transpose() << '\n';
    return;
  }

  if (controlInputType_ == dynamicgraph::sot::CONTROL_INPUT_NO_INTEGRATION) {
    state_ = controlIN;
    return;
  }

  if (vel_control_.size() == 0)
    vel_control_ = Vector::Zero(controlIN.size());

  if (controlInputType_ == dynamicgraph::sot::CONTROL_INPUT_TWO_INTEGRATION) {
    // TODO check acceleration
    // Position increment
    vel_control_ = velocity_ + (0.5 * dt) * controlIN;
    // Velocity integration.
    velocity_ += controlIN * dt;
  } else {
    vel_control_ = controlIN;
  }

  // Velocity bounds check
  if (sanityCheck_) {
    CHECK_BOUNDS(velocity_, lowerVelocity_, upperVelocity_, "velocity");
  }

  // Position integration
  state_ += vel_control_ * dt;

  // Position bounds check
  if (sanityCheck_) {
    CHECK_BOUNDS(state_, lowerPosition_, upperPosition_, "position");
  }
}
