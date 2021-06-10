# Copyright (c) 2021, CNRS

# Author: Florent Lamiraux

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Add the library to wrap the device of Tiago.

SET(DEVICE_NAME sot-universal-robot-device)
ADD_LIBRARY(${DEVICE_NAME} SHARED device.cc)

# Link the dynamic library containing the SoT with its dependencies.
TARGET_LINK_LIBRARIES(${DEVICE_NAME} sot-core::sot-core)

INSTALL(TARGETS ${DEVICE_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

# build python submodule
STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${DEVICE_NAME})
DYNAMIC_GRAPH_PYTHON_MODULE("sot/universal_robot/${PYTHON_LIBRARY_NAME}"
  ${DEVICE_NAME} sot_universal_robot-${PYTHON_LIBRARY_NAME}-wrap
  SOURCE_PYTHON_MODULE ${CMAKE_CURRENT_SOURCE_DIR}/device-python.cc)

# Install Python files.
SET(PYTHON_MODULE dynamic_graph/sot/universal_robot)
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py")
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "universal_robot.py")
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "prologue.py")

# Add the library to wrap the controller of Universal-Robot.
SET(CONTROLLER_NAME sot-universal-robot-controller)
ADD_LIBRARY(${CONTROLLER_NAME} SHARED
  controller.cc
  controller.hh
  )

# Link the dynamic library containing the SoT with its dependencies.
TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} sot-core::sot-core dynamic-graph-python::dynamic-graph-python
    dynamic_graph_bridge::ros_bridge dynamic_graph_bridge::ros_interpreter sot-universal-robot-device)

INSTALL(TARGETS ${CONTROLLER_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
