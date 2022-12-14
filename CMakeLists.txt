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

cmake_minimum_required(VERSION 3.1)

# Project properties
SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME sot_universal_robot)
SET(PROJECT_DESCRIPTION "dynamic-graph package for Universal Robot")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
OPTION(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(DOXYGEN_USE_TEMPLATE_CSS TRUE)
SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")
SET(CXX_DISABLE_WERROR TRUE)

# JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/ros.cmake)
INCLUDE(cmake/python.cmake)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
FINDPYTHON()
SEARCH_FOR_BOOST_PYTHON()

ADD_PROJECT_DEPENDENCY(sot-dynamic-pinocchio REQUIRED PKG_CONFIG_REQUIRES sot-dynamic-pinocchio)
ADD_PROJECT_DEPENDENCY(dynamic-graph-python 4.0.0 REQUIRED PKG_CONFIG_REQUIRES dynamic-graph-python)
ADD_PROJECT_DEPENDENCY(dynamic_graph_bridge REQUIRED PKG_CONFIG_REQUIRES dynamic_graph_bridge)
ADD_PROJECT_DEPENDENCY(ur_description)
ADD_REQUIRED_DEPENDENCY(dynamic_graph_bridge_msgs)

# Main Library
ADD_LIBRARY(${PROJECT_NAME} INTERFACE)
INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

# Add subdirectories.
ADD_SUBDIRECTORY(src)

INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Install launch directory
INSTALL(FILES
  launch/controller.launch
  launch/sot_params.launch
  DESTINATION share/${PROJECT_NAME}/launch)

# Install configuration files
INSTALL(FILES
  config/sot_params_common.yaml
  config/sot_params_ur.yaml
  DESTINATION share/${PROJECT_NAME}/config)

# Install default URDF file
INSTALL(FILES
  urdf/ur10.urdf
  DESTINATION share/${PROJECT_NAME}/urdf)
