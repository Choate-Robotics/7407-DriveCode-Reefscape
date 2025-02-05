import command.target
from utils import LocalLogger
from oi.keymap import Keymap
import command
from robot_systems import Robot

log = LocalLogger("OI")


class OI:
    @staticmethod
    def init() -> None:
        log.info("Initializing OI...")

    @staticmethod
    def map_controls():
        log.info("Mapping controls...")

        Keymap.Drivetrain.RESET_GYRO.onTrue(
            command.DrivetrainZero(Robot.drivetrain)) \
            .onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Scoring.SCORE_L1.onTrue(
            commands2.command.target.TargetCommand("L1")
        )
        )
