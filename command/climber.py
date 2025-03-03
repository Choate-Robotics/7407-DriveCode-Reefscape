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

# cmds: zero climber
class ZeroClimber(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.zero_encoder()

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.zeroed

# cmds: deploy climber
class DeployClimb(SubsystemCommand[Climber]):

    """
    Sets the climber to a given angle (radians).
    param: radians in radians
    """

    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.revolutions = config.climber_revolutions

    def initialize(self):
        # change to actual name
        self.subsystem.set_raw_output(config.deploy_climber_speed)

    def execute(self):
        pass

    def isFinished(self):
        if self.subsystem.get_motor_revolutions() >= self.revolutions:
            return True
        return False

    def end(self, interrupted: bool):
        if interrupted:
            logger.warn(f"Climber stuck")

        self.subsystem.set_raw_output(0)
        self.subsystem.moving = False

# cmds: lift climb
class LiftClimb(SubsystemCommand[Climber]):

    """
    Sets the climber to a given angle (radians).
    param: radians in radians
    """

    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.revolutions = config.climber_revolutions

    def initialize(self):
        # change to actual name
        self.subsystem.set_raw_output(config.lift_climber_speed)

    def execute(self):
        pass

    def isFinished(self):
        if self.subsystem.get_motor_revolutions() >= self.revolutions:
            return True
        return False

    def end(self, interrupted: bool):
        if interrupted:
            logger.warn(f"Climber stuck")

        self.subsystem.set_raw_output(0)
        self.subsystem.moving = False