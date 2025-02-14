import command.target
from utils import LocalLogger
from oi.keymap import Keymap
import command
from robot_systems import Robot, Field
import config
import commands2
import command.drivetrain

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
        
        Keymap.Drivetrain.X_MODE.onTrue(
            command.DrivetrainXMode(Robot.drivetrain)
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Drivetrain.DRIVE_TO_RIGHT_POSE.onTrue(
            command.DriveToPose(Robot.drivetrain, Field.branch.get_right_branches())
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))

        Keymap.Drivetrain.DRIVE_TO_LEFT_POSE.onTrue(
            command.DriveToPose(Robot.drivetrain, Field.branch.get_left_branches())
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
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

        Keymap.Scoring.SCORE_BARGE.onTrue(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["SCORE_BARGE"]
            ).andThen(command.FeedOut(Robot.wrist))
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
        ).onFalse(
            commands2.InstantCommand(lambda: Robot.wrist.feed_stop())
        )
        
        # INTAKING
        Keymap.Intake.INTAKE_CORAL.onTrue(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["STATION_INTAKING"]
            ).andThen(
                command.IntakeCoral(Robot.intake, Robot.wrist)
            )
        ).onFalse(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["IDLE"]
            )
        )

        # Pivots intake down and runs intake out. Equivalent to scoring algae in processor from intake.
        Keymap.Intake.INTAKE_EJECT.onTrue(
            command.PivotIntake(
                Robot.intake,
                False # Ground intake, will need to change to angle at some point
            ).andThen(
                command.EjectIntake(Robot.intake)
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
        Keymap.Wrist.REMOVE_ALGAE.and_(lambda: not Robot.wrist.coral_in_feed).onTrue(
            commands2.ConditionalCommand(
                command.Target(
                    Robot.elevator,
                    Robot.wrist,
                    Robot.intake,
                    config.target_positions["DEALGAE_HIGH"]
                ),
                command.Target(
                    Robot.elevator,
                    Robot.wrist,
                    Robot.intake,
                    config.target_positions["DEALGAE_LOW"]
                ),
                lambda: (Field.odometry.getPose().nearest(Field.reef_face.get_faces()) in Field.reef_face.get_high_algae())
            ).andThen(command.FeedIn(Robot.wrist))
        ).onFalse(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["IDLE"]
            )
        )
        
        Keymap.Intake.INTAKE_ALGAE.and_(lambda: not Robot.intake.intake_running).onTrue(
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

        Keymap.Wrist.EXTAKE_ALGAE.onTrue(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["SCORE_PROCESSOR_WRIST"]
            )
        ).onFalse(
            command.Target(
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                config.target_positions["IDLE"]
            )
        )

        