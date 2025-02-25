import commands2.instantcommand
from utils import LocalLogger
from oi.keymap import Keymap
import command
from robot_systems import Robot, Field
import config
from commands2 import InstantCommand, ConditionalCommand, SequentialCommandGroup, ParallelCommandGroup
from wpimath.geometry import Pose2d
from wpimath.geometry import Translation2d, Rotation2d
import math

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
            # command.DriveToPose(Robot.drivetrain, Field.branch.get_right_branches()),
            command.DriveToPose(Robot.drivetrain, Field.get_branches().get_right_branches())
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))

        Keymap.Drivetrain.DRIVE_TO_LEFT_POSE.onTrue(
            # command.DriveToPose(Robot.drivetrain, Field.branch.get_left_branches()),
            command.DriveToPose(Robot.drivetrain, Field.get_branches().get_left_branches())
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Drivetrain.CORAL_STATION_ALIGN.onTrue(
            command.DriveToPose(Robot.drivetrain, [Field.coral_station.leftCenterFace, Field.coral_station.rightCenterFace], 4, 7)
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))

        Keymap.Drivetrain.RIGHT_AUTO_START_POSE.onTrue(
            command.DriveToPose(Robot.drivetrain, [Pose2d(Translation2d(7.385, 1.46), Rotation2d.fromDegrees(90))], 0.8)
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        # Scoring on reef
        # Keymap.Scoring.SCORE_L1.onTrue(
        #     command.Target(config.target_positions["L1"], Robot.wrist, Robot.elevator)
        # ).onFalse(command.Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator))

        Keymap.Scoring.SCORE_L1.onTrue(
            command.SetPivot(Robot.intake, config.intake_l1_angle).andThen(command.EjectIntake(Robot.intake))
        ).onFalse(command.SetPivot(Robot.intake, config.intake_coral_station_angle))

        Keymap.Scoring.SCORE_L2.onTrue(
            command.Target(config.target_positions["L2"], Robot.wrist, Robot.elevator)
        ).onFalse(command.Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator))

        Keymap.Scoring.SCORE_L3.onTrue(
            command.Target(config.target_positions["L3"], Robot.wrist, Robot.elevator)
        ).onFalse(command.Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator))

        Keymap.Scoring.SCORE_L4.onTrue(
            command.Target(config.target_positions["L4"], Robot.wrist, Robot.elevator)
        ).onFalse(command.Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator))

        # # Score algae in barge
        # Keymap.Scoring.SCORE_BARGE.onTrue(
        #     target_command_generator(config.target_positions["SCORE_BARGE"]).andThen(
        #         command.FeedOut(Robot.wrist)
        #     )
        # ).onFalse(target_command_generator(config.target_positions["IDLE"]))

        # # Scoring coral
        Keymap.Wrist.EXTAKE_CORAL.and_(
            lambda: not Robot.elevator.elevator_moving
        ).and_(
            lambda: not Robot.wrist.wrist_angle_moving
        ).and_(
            lambda: not Robot.intake.intake_running
        ).whileTrue(command.FeedOut(Robot.wrist))

        # Intake coral from station
        Keymap.Intake.INTAKE_CORAL.and_(lambda: not Robot.wrist.coral_in_feed).whileTrue(
            commands2.ParallelCommandGroup(
                command.Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator),
                command.SetPivot(
                    Robot.intake,
                    config.target_positions["STATION_INTAKING"].intake_angle,
                ),
            ).andThen(command.IntakeCoral(Robot.intake, Robot.wrist))
        ).onFalse(command.Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator).onlyIf(lambda: Robot.wrist.coral_in_feed))

        # Eject coral from wrist and intake
        # Keymap.Intake.EJECT_CORAL.whileTrue(
        #     commands2.ParallelCommandGroup(
        #         target_command_generator(config.target_positions["STATION_INTAKING"]),
        #         command.SetPivot(Robot.intake, config.intake_coral_station_angle),
        #     ).andThen(command.EjectCoral(Robot.intake, Robot.wrist))
        # ).onFalse(target_command_generator(config.target_positions["IDLE"]))

        # De-algae
        Keymap.Wrist.REMOVE_ALGAE.and_(lambda: not Robot.wrist.coral_in_feed).onTrue(
            commands2.ConditionalCommand(
                command.Target(config.target_positions["DEALGAE_HIGH"], Robot.wrist, Robot.elevator),
                command.Target(config.target_positions["DEALGAE_LOW"], Robot.wrist, Robot.elevator),
                lambda: (
                    Field.odometry.getPose().nearest(Field.reef_face.get_faces())
                    in Field.reef_face.get_high_algae()
                ),
            ).andThen(command.WristAlgaeIn(Robot.wrist))
        ).onFalse(command.Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator))

        # Keymap.Wrist.REMOVE_ALGAE.and_(lambda: not Robot.wrist.coral_in_feed).whileTrue(
        #     command.Target(config.target_positions["DEALGAE_LOW"], Robot.wrist, Robot.elevator).andThen(
        #         command.WristAlgaeIn(Robot.wrist)
        #     )
        # ).onFalse(command.Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator))

        # Intaking algae with ground intake
        Keymap.Intake.INTAKE_ALGAE.whileTrue(
            command.SetPivot(
                Robot.intake, config.target_positions["INTAKE_ALGAE"].intake_angle
            ).andThen(
                command.IntakeAlgae(Robot.intake)
            )
        )

        # Score algae in processor
        Keymap.Wrist.EXTAKE_ALGAE.onTrue(
            command.ExtakeAlgae(Robot.intake)
        ).onFalse(command.SetPivot(Robot.intake, config.target_positions["IDLE"].intake_angle))
