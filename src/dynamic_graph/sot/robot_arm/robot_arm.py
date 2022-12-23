# -*- coding: utf-8 -*-
# Copyright 2021, 2022 CNRS - Airbus SAS
# Author: Florent Lamiraux, Olivier Stasse
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import pinocchio as se3
from dynamic_graph import plug
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Vector
from dynamic_graph.sot.dynamic_pinocchio.dynamic import DynamicPinocchio
from dynamic_graph.sot.dynamic_pinocchio.humanoid_robot import AbstractRobot
import pinocchio

class RobotArm(AbstractRobot):
    """
    This class defines an industrial robot arm
    """

    rosParamName = "/robot_description"
    def __init__(self, name, device=None, tracer=None, loadFromFile=False):
        self.OperationalPointsMap = None

        if loadFromFile:
            print("Loading from file " + self.defaultFilename)
            self.loadModelFromUrdf(self.defaultFilename, rootJointType=None)
        else:
            print("Using ROS parameter \"/robot_description\"")
            import rospy
            if self.rosParamName not in rospy.get_param_names():
                raise RuntimeError('"' + self.rosParamName +
                                   '" is not a ROS parameter.')
            s = rospy.get_param(self.rosParamName)
            self.loadModelFromString(s, rootJointType=None)
        AbstractRobot.__init__(self, name, tracer)

        # Create rigid body dynamics model and data (pinocchio)
        self.dynamic = DynamicPinocchio(self.name + "_dynamic")
        self.dynamic.setModel(self.pinocchioModel)
        self.dynamic.setData(self.pinocchioData)
        self.dynamic.displayModel()
        self.dimension = self.dynamic.getDimension()

        self.device = device
        self.initializeRobot()

        # Create operational points based on operational points map
        # (if provided)
        if self.OperationalPointsMap is not None:
            self.initializeOpPoints()

    def defineHalfSitting(self, q):
        pass

    def _initialize(self):
        AbstractRobot._initialize(self)
        self.OperationalPoints.extend(['wrist', 'gaze'])

__all__ = [RobotArm]
