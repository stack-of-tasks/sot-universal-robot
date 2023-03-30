# BSD 2-Clause License

# Copyright (c) 2021, CNRS

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

from math import pi
import numpy
import matplotlib.pyplot as plt
from dynamic_graph import plug
from dynamic_graph.sot.core.operator import Substract_of_vector, \
    Multiply_double_vector
from dynamic_graph import writeGraph
from dynamic_graph.sot.robot_arm import RobotArm
import example_robot_data

RobotArm.defaultFilename = example_robot_data.getModelPath(".") + \
    "/ur_description/urdf/ur10_robot.urdf"

robot = RobotArm('ur10', loadFromFile = True)

robot.integrator.signal("velocity").value = numpy.zeros(6)
robot.integrator.signal("configuration").recompute(10000)

error = Substract_of_vector('error')
plug(robot.integrator.signal("configuration"), error.sin1)
error.sin2.value = numpy.array([pi/6, -pi/2, pi/2, 0, 0, 0,])
control = Multiply_double_vector('control')
control.sin1.value = -1.
plug(error.sout, control.sin2)
plug(control.sout, robot.integrator.signal("velocity"))

size = 1000
traj = numpy.zeros((6, size))

for i in range(size):
    t = 20000 + i*10000
    robot.integrator.signal("configuration").recompute(t)
    traj[:,i] = robot.integrator.signal("configuration").value

fig = plt.figure()
ax = fig.add_subplot(111)
times = 1e-3*numpy.arange(size)
for i in range(3):
    ax.plot(times, traj[i,:])
plt.show()
