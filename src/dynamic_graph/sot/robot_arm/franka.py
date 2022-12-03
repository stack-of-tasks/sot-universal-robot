# Copyright, 2022 CNRS
# Author: Florent Lamiraux
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

from .robot_arm import RobotArm
from .sot_robot_arm_device import DeviceToDynamic

class Franka(RobotArm):
    """
    This class defines a robot of type Franka Panda with gripper
    """
    defaultFilename = "package://sot_universal_robot/urdf/franka.urdf"

    def __init__(self, name, device=None, tracer=None, loadFromFile=False):
        RobotArm.__init__(self, name, device, tracer, loadFromFile)

    # Resize the signal from dimension 7 to 8
    #
    # self.dynamic is initialized using the URDF model containing
    # 9 joints: 7 for the arm, 2 for the fingers. The second finger
    # joint mimics the first finger joint. The dimension is therefore 8.
    #
    #  self.device is initialized via ros params and contains only 7 joints.
    def setClosedLoop(self, closedLoop):
        if closedLoop:
            raise RuntimeError('Closed loop not possible for Franka robot.')
        else:
            d = DeviceToDynamic('d2d')
            plug(self.device.state, d.signal('sin'))
            plug(d.signal('sout'), self.dynamic.position)
            self.device.setClosedLoop(False)

    ## Only the 7 arm joints are actuated
    #
    #  The fingers are not controlled via roscontrol.
    def getActuatedJoints(self):
        return range(0,7)
