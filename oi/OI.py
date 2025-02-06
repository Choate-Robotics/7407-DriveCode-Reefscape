import command.target
from utils import LocalLogger
from oi.keymap import Keymap
import command
from robot_systems import Robot
import config

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
            command.target.Target(config.target_positions["L1"])
        )
        Keymap.Scoring.SCORE_L2.onTrue(
            command.target.Target(config.target_positions["L2"])
        )
        Keymap.Scoring.SCORE_L3.onTrue(
            command.target.Target(config.target_positions["L4"])
        )
        Keymap.Scoring.SCORE_L4.onTrue(
            command.target.Target(config.target_positions["L4"])
        )

