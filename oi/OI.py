from utils import LocalLogger
from oi.keymap import Keymap
import command
from robot_systems import Robot
import math

from commands2 import InstantCommand

log = LocalLogger("OI")


class OI:
    @staticmethod
    def init() -> None:
        log.info("Initializing OI...")

    @staticmethod
    def map_controls():
        log.info("Mapping controls...")
        pass

        # Keymap.Drivetrain.RESET_GYRO.onTrue(
        #     command.DrivetrainZero(Robot.drivetrain)) \
        #     .onFalse(command.DriveSwerveCustom(Robot.drivetrain))

        Keymap.Wrist.A.onTrue(
            command.SetWrist(Robot.wrist, math.radians(-117))
        )
        Keymap.Wrist.B.onTrue(
            command.SetWrist(Robot.wrist, math.radians(0))
        )
        Keymap.Wrist.Y.onTrue(
            command.SetWrist(Robot.wrist, math.radians(45))
        )