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
from utils import LocalLogger

from wpimath.controller import (
    PIDController
)

logger = LocalLogger("Climber Command: ")

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

        self.pid = PIDController(0.01, 0, 0) #TODO: Tune

    def initialize(self):
        # change to actual name
        self.pid.setSetpoint(self.angle)
        self.subsystem.moving = True

    def execute(self):
        self.subsystem.set_raw_output(self.pid.calculate(self.subsystem.get_angle()))

    def isFinished(self):
        return abs(self.angle - self.subsystem.get_angle(self.angle)) < config.climber_angle_threshold

    def end(self, interrupted: bool):
        if interrupted:
            arm_radians = self.subsystem.get_angle()
            logger.warn(f"Stuck at {arm_radians} radians")

        self.subsystem.set_raw_output(0)
        self.subsystem.moving = False