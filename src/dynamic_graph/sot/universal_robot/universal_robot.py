# -*- coding: utf-8 -*-
# Copyright 2021, Olivier Stasse, Florent Lamiraux, LAAS-CNRS

from __future__ import print_function

import pinocchio as se3
from dynamic_graph import plug
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Vector
from dynamic_graph.sot.dynamic_pinocchio.dynamic import DynamicPinocchio
from dynamic_graph.sot.dynamic_pinocchio.humanoid_robot import AbstractRobot
import pinocchio

class UniversalRobot(AbstractRobot):
    """
    This class defines a UniversalRobot UR3, UR5, or UR10
    """

    defaultFilename = "package://sot_universal_robot/urdf/ur10.urdf"

    def __init__(self, name, device=None, tracer=None, loadFromFile=False):
        self.OperationalPointsMap = {
            'wrist': 'wrist_3_joint',
            'gaze': 'wrist_3_joint',
        }

        rosParamName = "/robot_description"
        if loadFromFile:
            print("Loading from file " + self.defaultFilename)
            self.loadModelFromUrdf(self.defaultFilename, rootJointType=None)
        else:
            print("Using ROS parameter \"/robot_description\"")
            import rospy
            if rosParamName not in rospy.get_param_names():
                raise RuntimeError('"' + rosParamName +
                                   '" is not a ROS parameter.')
            s = rospy.get_param(rosParamName)
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
        return 6 * [0.]

    def setClosedLoop(self, closedLoop):
        if closedLoop:
            plug(self.device.robotState, self.dynamic.position)
            self.device.setClosedLoop(True)
        else:
            plug(self.device.state, self.dynamic.position)
            self.device.setClosedLoop(False)

    def _initialize(self):
        AbstractRobot._initialize(self)
        self.OperationalPoints.extend(['wrist', 'gaze'])

__all__ = [UniversalRobot]
