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
#include <dynamic-graph/entity.h>
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

const double SoTRobotArmDevice::TIMESTEP_DEFAULT = 0.001;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SoTRobotArmDevice,
				   "DeviceRobotArm");

SoTRobotArmDevice::SoTRobotArmDevice(std::string RobotName):
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
}

SoTRobotArmDevice::~SoTRobotArmDevice()
{ }

void SoTRobotArmDevice::setSensors(map<string,SensorValues> &SensorsIn)
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

void SoTRobotArmDevice::setupSetSensors(map<string,SensorValues>&
					      SensorsIn)
{
  setSensors (SensorsIn);
  // Robot state might be smaller than state in case there is a gripper
  // with mimic joints.
  state_.head(robotState_.accessCopy().size()) = robotState_.accessCopy();
  stateSOUT.setConstant(state_);
}

void SoTRobotArmDevice::nominalSetSensors(map<string,SensorValues>&
						SensorsIn)
{
  setSensors (SensorsIn);
}


void SoTRobotArmDevice::cleanupSetSensors(map<string, SensorValues>&
						SensorsIn)
{
  setSensors (SensorsIn);
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DeviceToDynamic, "DeviceToDynamic");
