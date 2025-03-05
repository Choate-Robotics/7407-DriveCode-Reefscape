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

# cmds: deploy climber
class DeployClimb(SubsystemCommand[Climber]):
    # to do: fix comments
    """
    Sets the climber to a given angle (radians).
    param: radians in radians
    """

    def __init__(self, subsystem: Climber, speed: float = config.deploy_climber_speed, upper_bound: float = config.deploy_position):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.speed = speed
        self.upper_bound = upper_bound

    def initialize(self):
        self.subsystem.set_raw_output(self.speed)
        self.subsystem.moving = True

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.get_motor_revolutions() >= self.upper_bound

    def end(self, interrupted: bool):
        self.subsystem.set_raw_output(0)
        self.subsystem.moving = False

# cmds: lift climb
class Climb(SubsystemCommand[Climber]):
    # to do: fix comments
    """
    Sets the climber to a given angle (radians).
    param: radians in radians
    """

    def __init__(self, subsystem: Climber, speed: float = config.deploy_climber_speed, lower_bound: float = 0):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.speed = speed
        self.lower_bound = lower_bound

    def initialize(self):
        self.subsystem.set_raw_output(-self.speed)
        self.subsystem.moving = True

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.get_motor_revolutions() <= self.lower_bound

    def end(self, interrupted: bool):
        self.subsystem.set_raw_output(0)
        self.subsystem.moving = False