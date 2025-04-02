import commands2.instantcommand
from utils import LocalLogger
from oi.keymap import Keymap
import command
from robot_systems import Robot, Field
import config
from commands2 import InstantCommand, ConditionalCommand, SequentialCommandGroup, ParallelCommandGroup
from wpimath.geometry import Pose2d
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
            command.DriveToPose(Robot.drivetrain, Field.branch.get_right_branches()),
            # command.DriveToPose(Robot.drivetrain, Field.get_branches().get_right_branches())
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))

        Keymap.Drivetrain.DRIVE_TO_LEFT_POSE.onTrue(
            command.DriveToPose(Robot.drivetrain, Field.branch.get_left_branches()),
            # command.DriveToPose(Robot.drivetrain, Field.get_branches().get_left_branches())
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Drivetrain.ALGAE_ALIGN.onTrue(
            command.DriveToPose(Robot.drivetrain, Field.reef_face.get_faces())
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))

        # Scoring on reef
        # Keymap.Scoring.SCORE_L1.onTrue(
        #     command.Target(config.target_positions["L1"], Robot.wrist, Robot.elevator)
        # ).onFalse(command.Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator))

        Keymap.Scoring.SCORE_L1.onTrue(
            command.SetPivot(Robot.intake, config.intake_l1_angle).andThen(command.EjectL1(Robot.intake))
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

        # # De-Algae based on face and then score
        Keymap.Scoring.DEALGAE_SCORE.onTrue(
            commands2.ConditionalCommand(
                commands2.InstantCommand(lambda: Robot.wrist.algae_in()).andThen(command.Target(config.target_positions["DEALGAE_HIGH"], Robot.wrist, Robot.elevator)),
                commands2.InstantCommand(lambda: Robot.wrist.algae_in()).andThen(command.Target(config.target_positions["DEALGAE_LOW"], Robot.wrist, Robot.elevator)),
                lambda: (
                    Field.odometry.getPose().nearest(Field.reef_face.get_faces())
                    in Field.reef_face.get_high_algae()
                ),
            )
        ).onFalse(command.Target(config.target_positions["L3"], Robot.wrist, Robot.elevator).andThen(commands2.InstantCommand(lambda: Robot.wrist.algae_stop())))

        # Extake into barge
        Keymap.Scoring.SCORE_BARGE.onTrue(
            command.Target(config.target_positions["BARGE"], Robot.wrist, Robot.elevator)
        ).onFalse(command.Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator))

        # # Scoring coral
        Keymap.Wrist.EXTAKE_CORAL.and_(
            lambda: not Robot.elevator.elevator_moving
        ).and_(
            lambda: not Robot.wrist.wrist_angle_moving
        ).and_(
            lambda: not Robot.intake.intake_running
        ).whileTrue(command.FeedOut(Robot.wrist, config.wrist_extake_speed_teleop))

        # Intake coral from station
        Keymap.Intake.INTAKE_CORAL.whileTrue(
            commands2.ParallelCommandGroup(
                command.Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator),
                command.SetPivot(
                    Robot.intake,
                    config.target_positions["STATION_INTAKING"].intake_angle,
                ),
            ).onlyIf(lambda: not Robot.wrist.coral_in_feed).andThen(command.IntakeCoral(Robot.intake, Robot.wrist))
        ).onFalse(command.Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator).onlyIf(lambda: Robot.wrist.coral_in_feed))

        Keymap.Intake.INTAKE_L1.whileTrue(
            commands2.SequentialCommandGroup(
                commands2.ParallelCommandGroup(
                    command.Target(config.target_positions["INTAKE_L1"], Robot.wrist, Robot.elevator),
                    command.SetPivot(Robot.intake, config.intake_coral_station_angle)
                ),
                command.RunIntake(Robot.intake)
            )
        )

        # De-algae
        Keymap.Wrist.REMOVE_ALGAE.onTrue(
            commands2.ConditionalCommand(
                commands2.InstantCommand(lambda: Robot.wrist.algae_in()).andThen(command.Target(config.target_positions["DEALGAE_HIGH"], Robot.wrist, Robot.elevator)),
                commands2.InstantCommand(lambda: Robot.wrist.algae_in()).andThen(command.Target(config.target_positions["DEALGAE_LOW"], Robot.wrist, Robot.elevator)),
                lambda: (
                    Field.odometry.getPose().nearest(Field.reef_face.get_faces())
                    in Field.reef_face.get_high_algae()
                ),
            )
        ).onFalse(commands2.InstantCommand(lambda: Robot.wrist.algae_stop()).andThen(command.Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator)))

        # Intaking algae with ground intake
        Keymap.Intake.INTAKE_ALGAE.or_(Keymap.Intake.INTAKE_ALGAE_DRIVER).whileTrue(
            command.SetPivot(
                Robot.intake, config.target_positions["INTAKE_ALGAE"].intake_angle
            ).andThen(
                command.IntakeAlgae(Robot.intake)
            )
        )

        # Extake into processor
        Keymap.Wrist.EXTAKE_ALGAE_DRIVER.whileTrue(
            commands2.ParallelCommandGroup(
                command.ExtakeAlgae(Robot.intake).onlyIf(lambda: Robot.intake.get_pivot_angle() >= math.radians(40)),
                command.WristAlgaeOut(Robot.wrist)
            )
        ).onFalse(command.SetPivot(Robot.intake, config.target_positions["IDLE"].intake_angle))

        Keymap.Climb.CLIMB_UNLOCK.onTrue(
            commands2.SequentialCommandGroup(
                commands2.ParallelCommandGroup(
                    command.DeployClimb(Robot.climber),
                    command.Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator),
                ),
                command.SetPivot(Robot.intake, config.intake_climb_angle)
            )
        )

        Keymap.Climb.CLIMB.whileTrue(
            command.Climb(Robot.climber, config.climb_speed)
        )

        Keymap.Climb.MANUAL_CLIMB_DEPLOY.whileTrue(
            command.DeployClimb(Robot.climber, config.manual_climber_speed)
        )

        Keymap.Climb.MANUAL_CLIMB.whileTrue(
            command.Climb(Robot.climber, config.manual_climber_speed, config.manual_lower_bound)
        )