import commands2.instantcommand
from command import target_command_generator
from utils import LocalLogger
from oi.keymap import Keymap
import command
from robot_systems import Robot, Field
import config
from commands2 import InstantCommand, ConditionalCommand, SequentialCommandGroup, ParallelCommandGroup
from wpimath.geometry import Pose2d

log = LocalLogger("OI")


class OI:
    @staticmethod
    def init() -> None:
        log.info("Initializing OI...")

    @staticmethod
    def map_controls():
        log.info("Mapping controls...")

        Keymap.Drivetrain.RESET_GYRO.onTrue(
            command.DrivetrainZero(Robot.drivetrain)
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Drivetrain.X_MODE.onTrue(
            command.DrivetrainXMode(Robot.drivetrain)
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))

        Keymap.Drivetrain.DRIVE_TO_RIGHT_POSE.onTrue(
            command.DriveToPose(Robot.drivetrain, Field.branch.get_right_branches())
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))

        Keymap.Drivetrain.DRIVE_TO_LEFT_POSE.onTrue(
            command.DriveToPose(Robot.drivetrain, Field.branch.get_left_branches())
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Drivetrain.CORAL_STATION_ALIGN.onTrue(
            ConditionalCommand(
                command.DriveSwerveAim(Robot.drivetrain, Field.coral_station.rightCenterFace.rotation().radians()),
                command.DriveSwerveAim(Robot.drivetrain, Field.coral_station.leftCenterFace.rotation().radians()),
                lambda: Field.odometry.getPose().nearest([Field.coral_station.leftCenterFace, Field.coral_station.rightCenterFace]) == Field.coral_station.rightCenterFace
            )
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))

        # Scoring on reef
        Keymap.Scoring.SCORE_L1.onTrue(
            target_command_generator(config.target_positions["L1"])
        ).onFalse(target_command_generator(config.target_positions["IDLE"]))

        Keymap.Scoring.SCORE_L2.onTrue(
            target_command_generator(config.target_positions["L2"])
        ).onFalse(target_command_generator(config.target_positions["IDLE"]))

        Keymap.Scoring.SCORE_L3.onTrue(
            target_command_generator(config.target_positions["L3"])
        ).onFalse(target_command_generator(config.target_positions["IDLE"]))

        Keymap.Scoring.SCORE_L4.onTrue(
            target_command_generator(config.target_positions["L4"])
        ).onFalse(target_command_generator(config.target_positions["IDLE"]))

        # Score algae in barge
        Keymap.Scoring.SCORE_BARGE.onTrue(
            target_command_generator(config.target_positions["SCORE_BARGE"]).andThen(
                command.FeedOut(Robot.wrist)
            )
        ).onFalse(target_command_generator(config.target_positions["IDLE"]))

        # Scoring coral
        Keymap.Wrist.EXTAKE_CORAL.onTrue(command.FeedOut(Robot.wrist)).onFalse(
            commands2.InstantCommand(lambda: Robot.wrist.feed_stop())
        )

        # Intake coral from station
        Keymap.Intake.INTAKE_CORAL.whileTrue(
            commands2.ParallelCommandGroup(
                target_command_generator(config.target_positions["STATION_INTAKING"]),
                command.SetPivot(
                    Robot.intake,
                    config.target_positions["STATION_INTAKING"].intake_angle,
                ),
            ).andThen(command.IntakeCoral(Robot.intake, Robot.wrist))
        ).onFalse(target_command_generator(config.target_positions["IDLE"]))

        # Eject coral from wrist and intake
        Keymap.Intake.EJECT_CORAL.whileTrue(
            commands2.ParallelCommandGroup(
                target_command_generator(config.target_positions["STATION_INTAKING"]),
                command.SetPivot(Robot.intake, config.intake_coral_station_angle),
            ).andThen(command.EjectCoral(Robot.intake, Robot.wrist))
        ).onFalse(target_command_generator(config.target_positions["IDLE"]))

        # De-algae
        Keymap.Wrist.REMOVE_ALGAE.and_(lambda: not Robot.wrist.coral_in_feed).onTrue(
            commands2.ConditionalCommand(
                target_command_generator(config.target_positions["DEALGAE_HIGH"]),
                target_command_generator(config.target_positions["DEALGAE_LOW"]),
                lambda: (
                    Field.odometry.getPose().nearest(Field.reef_face.get_faces())
                    in Field.reef_face.get_high_algae()
                ),
            ).andThen(command.FeedIn(Robot.wrist))
        ).onFalse(target_command_generator(config.target_positions["IDLE"]))

        # Intaking algae with ground intake
        Keymap.Intake.INTAKE_ALGAE.onTrue(
            commands2.ParallelCommandGroup(
                target_command_generator(config.target_positions["INTAKE_ALGAE"]),
                command.SetPivot(
                    Robot.intake, config.target_positions["INTAKE_ALGAE"].intake_angle
                ),
            ).andThen(
                command.RunIntake(Robot.intake).onlyIf(
                    lambda: not Robot.intake.intake_running
                )
            )
        ).onFalse(
            target_command_generator(config.target_positions["IDLE"]).andThen(
                commands2.InstantCommand(lambda: Robot.intake.stop())
            )
        )

        # Score algae in processor
        Keymap.Wrist.EXTAKE_ALGAE.onTrue(
            target_command_generator(
                config.target_positions["SCORE_PROCESSOR_WRIST"]
            ).andThen(command.FeedOut(Robot.wrist))
        ).onFalse(target_command_generator(config.target_positions["IDLE"]))