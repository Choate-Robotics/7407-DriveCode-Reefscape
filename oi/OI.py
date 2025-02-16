from utils import LocalLogger
from oi.keymap import Keymap
import command
from robot_systems import Robot
from commands2 import InstantCommand
import math

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
        Keymap.Intake.A.whileTrue(
            command.RunIntake(Robot.intake)
        )
        Keymap.Intake.Y.whileTrue(
            command.EjectIntake(Robot.intake)
        )