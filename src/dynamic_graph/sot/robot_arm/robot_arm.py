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

import numpy
import pinocchio as se3
from dynamic_graph import plug
from dynamic_graph.sot.dynamic_pinocchio.dynamic import DynamicPinocchio
from dynamic_graph.sot.dynamic_pinocchio.robot import AbstractRobot
import pinocchio

class RobotArm(AbstractRobot):
    """
    This class defines an industrial robot arm
    """
    # Index of the first actuated joint in the configuration vector
    firstJointIndex = 0
    def getActuatedJoints(self):
        return range(0, self.device.getControlSize())

    def initializeEntities(self):
        # initialize size of some signals. The size is not known at construction
        controlSize = self.device.getControlSize()
        self.selector.selec(self.firstJointIndex, self.firstJointIndex +
                             controlSize)
        q = numpy.zeros(self.dynamic.getDimension())
        q[0:controlSize] = self.device.signal("robotState").value
        self.integrator.setInitialConfig(q)
        self.integrator.signal("velocity").value = \
            numpy.zeros(self.pinocchioModel.nv)
        self.integrator.signal('configuration').recompute(0)
