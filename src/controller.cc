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

#include "controller.hh"

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

#include <dynamic_graph_bridge/ros_init.hh>
#include <dynamic_graph_bridge/ros_interpreter.hh>

#include <sot/core/debug.hh>
#include <sot/core/exception-abstract.hh>

#include "device.hh"

const std::string SoTRobotArmController::LOG_PYTHON
("/tmp/RobotArmController_python.out");

using std::cout;
using std::endl;
using dynamicgraph::sot::AbstractSotExternalInterface;

boost::condition_variable cond;
boost::mutex mut;
bool data_ready;

void workThread(SoTRobotArmController *controller)
{

  dynamicgraph::Interpreter aLocalInterpreter(dynamicgraph::rosInit(false,true));

  controller->interpreter_ =
    dynamicgraph::make_shared<dynamicgraph::Interpreter>(aLocalInterpreter);
  cout << "Going through the thread." << endl;
  {
    boost::lock_guard<boost::mutex> lock(mut);
    data_ready=true;
  }
  cond.notify_all();
  ros::waitForShutdown();
}

SoTRobotArmController::SoTRobotArmController():
  device_(new SoTRobotArmDevice("RobotArm"))
{
  // Create thread and python interpreter
  init();
  std::string robotType("ur");
  ros::NodeHandle nh;
  /// Read /sot_controller/dt to know what is the control period
  if (nh.hasParam("/sot_controller/dt")) {
    double dt;
    nh.getParam("/sot_controller/dt", dt);
    device_->setTimeStep(dt);
    ROS_INFO_STREAM("Set control period to: " << dt);
    if (nh.hasParam("/sot/robot")){
      nh.getParam("/sot/robot", robotType);
    }
  }
  startupPython(robotType);
  interpreter_->startRosService ();
}

SoTRobotArmController::SoTRobotArmController(std::string RobotName):
  device_(new SoTRobotArmDevice (RobotName))
{
  // Create thread and python interpreter
  init();
}

SoTRobotArmController::SoTRobotArmController(const char robotName[]):
  device_(new SoTRobotArmDevice (robotName))
{
  // Create thread and python interpreter
  init();
}

void SoTRobotArmController::init()
{
  cout << "Going through SoTRobotArmController." << endl;
  boost::thread thr(workThread,this);
  boost::unique_lock<boost::mutex> lock(mut);
  cond.wait(lock);
}

SoTRobotArmController::~SoTRobotArmController()
{
}

void SoTRobotArmController::setupSetSensors
(map<string,SensorValues> &SensorsIn)
{
  device_->setupSetSensors(SensorsIn);
}


void SoTRobotArmController::nominalSetSensors
(map<string,SensorValues> &SensorsIn)
{
  device_->nominalSetSensors(SensorsIn);
}

void SoTRobotArmController::cleanupSetSensors
(map<string, SensorValues> &SensorsIn)
{
  device_->cleanupSetSensors(SensorsIn);
}


void SoTRobotArmController::getControl
(map<string,ControlValues> &controlOut)
{
  device_->getControl(controlOut);
}

void SoTRobotArmController::
setNoIntegration(void)
{
  device_->setNoIntegration();
}

void SoTRobotArmController::
setSecondOrderIntegration(void)
{
  device_->setSecondOrderIntegration();
}

void SoTRobotArmController::
runPython(std::ostream& file,
	  const std::string& command,
	  dynamicgraph::Interpreter& interpreter)
{
  file << ">>> " << command << endl;
  std::string lerr(""),lout(""),lres("");
  interpreter.runCommand(command,lres,lout,lerr);
  if (lres != "None")
  {
    if (lres=="<NULL>")
    {
      file << lout << endl;
      file << "------" << endl;
      file << lerr << endl;
    }
    else
	file << lres << endl;
  }
}

void SoTRobotArmController::startupPython(const std::string& robotType)
{
  std::ofstream aof(LOG_PYTHON.c_str());
  runPython (aof, "import sys, os", *interpreter_);
  runPython (aof, "pythonpath = os.environ['PYTHONPATH']", *interpreter_);
  runPython (aof, "path = []", *interpreter_);
  runPython (aof,
	     "for p in pythonpath.split(':'):\n"
	     "  if p not in sys.path:\n"
	     "    path.append(p)", *interpreter_);
  runPython (aof, "path.extend(sys.path)", *interpreter_);
  runPython (aof, "sys.path = path", *interpreter_);
  runPython (aof, "import dynamic_graph.sot.robot_arm.prologue",
	     *interpreter_);
  std::ostringstream os;
  os << "robot = dynamic_graph.sot.robot_arm.prologue.makeRobot"
    " (robotType='" << robotType << "')";
  runPython (aof, os.str(), *interpreter_);

  // Calling again rosInit here to start the spinner. It will
  // deal with topics and services callbacks in a separate, non
  // real-time thread. See roscpp documentation for more
  // information.
  dynamicgraph::rosInit (true);
  aof.close();
}

extern "C"
{
  AbstractSotExternalInterface * createSotExternalInterface()
  {
    return new SoTRobotArmController ();
  }
}

extern "C"
{
  void destroySotExternalInterface(AbstractSotExternalInterface *p)
  {
    delete p;
  }
}
