import math
import wpilib
from commands2 import SequentialCommandGroup
import config
import constants
import utils
from oi.keymap import Controllers
from subsystem import Climber
from toolkit.command import SubsystemCommand
from units.SI import radians
from enum import Enum


# cmds: set climber (angle), zero climber

class SetClimber(SubsystemCommand[Climber]):

    """
    Sets the climber to a given angle (radians).
    param: radians in radians
    """

    def __init__(self, subsystem: Climber, angle: radians):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.angle = angle

    def initialize(self):
        # change to actual name
        self.subsystem.set_angle(self.angle)
        self.subsystem.arm_moving = True

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.check_angle(self.angle)

    def end(self, interrupted: bool):
        if interrupted:
            arm_radians = self.subsystem.get_angle()
            print(f"Stuck at {arm_radians} radians")

        self.subsystem.arm_moving = False

class ZeroClimber(SubsystemCommand[Climber]):
    """
    Zeros the climber.
    """

    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        return self.subsystem.zeroed == True

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.zeroed
    
    def end(self, interrupted: bool):
        if not interrupted:
            self.subsystem.zero()