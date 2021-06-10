# -*- coding: utf-8 -*-
# Copyright 2021, Joseph Mirabel, Florent Lamiraux,

# sys.argv is not defined when running the remove interpreter, but it is
# required by rospy
import sys

from dynamic_graph.sot.universal_robot.sot_universal_robot_device \
    import DeviceUniversalRobot
from dynamic_graph.sot.universal_robot import UniversalRobot as Robot

if not hasattr(sys, 'argv'):
    sys.argv = [
        "dynamic_graph",
    ]

print("Prologue UniversalRobot")


# Create the device.
# This entity behaves exactly like robotsimu except:
# 1. it does not provide the increment command
# 2. it forwards the robot control to the sot-abstract
#    controller.
def makeRobot():
    # Create the robot using the device.
    robot = Robot(name='ur', device=DeviceUniversalRobot('UniversalRobot'))
    return robot

