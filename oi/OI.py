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
        
        # SCORING
        Keymap.Scoring.SCORE_L1.onTrue(
            command.Target(
                Robot.elevator, 
                Robot.wrist, 
                Robot.intake, 
                config.target_positions["L1"]
            )
        ).onFalse(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["IDLE"]
            )
        )

        Keymap.Scoring.SCORE_L2.onTrue(
            command.Target(
                Robot.elevator, 
                Robot.wrist, 
                Robot.intake, 
                config.target_positions["L2"]
            )
        ).onFalse(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["IDLE"]
            )
        )

        Keymap.Scoring.SCORE_L3.onTrue(
            command.Target(
                Robot.elevator, 
                Robot.wrist, 
                Robot.intake, 
                config.target_positions["L3"]
            )
        ).onFalse(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["IDLE"]
            )
        )

        Keymap.Scoring.SCORE_L4.onTrue(
            command.Target(
                Robot.elevator, 
                Robot.wrist, 
                Robot.intake, 
                config.target_positions["L4"]
            )
        ).onFalse(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["IDLE"]
            )
        )

        # INTAKING
        Keymap.Intake.INTAKE_CORAL.onTrue(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["STATION_INTAKING"]
            )
        ).onFalse(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["IDLE"]
            )
        )


