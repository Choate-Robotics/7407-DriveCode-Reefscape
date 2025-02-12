import command.target
from utils import LocalLogger
from oi.keymap import Keymap
import command
from robot_systems import Robot
import config
import commands2

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
        Keymap.Wrist.EXTAKE_CORAL.onTrue(
            command.FeedOut(Robot.wrist)
        ).onFalse(commands2.InstantCommand(lambda: Robot.wrist.feed_stop()))
        
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


        # Algae
        # discuss whether dealgae should be a tap or a hold
        Keymap.Wrist.REMOVE_ALGAE.onTrue(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["DEALGAE_LOW"]
            )
        ).onFalse(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["IDLE"]
            )
        )
        Keymap.Intake.INTAKE_ALGAE.onTrue(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["INTAKE_ALGAE"]
            )
        ).onFalse(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["IDLE"]
            )
        )

