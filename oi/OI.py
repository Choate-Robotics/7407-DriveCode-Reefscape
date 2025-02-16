import command.drivetrain
from utils import LocalLogger
from oi.keymap import Keymap
import command
import constants
import math
from robot_systems import Robot, Field
from commands2 import InstantCommand, ConditionalCommand
from wpimath.geometry import Pose2d

log = LocalLogger("OI")


class OI:
    @staticmethod
    def init() -> None:
        log.info("Initializing OI...")

    @staticmethod
    def map_controls():
        log.info("Mapping controls...")
        pass

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

        Keymap.Drivetrain.CORAL_STATION_ALIGN.onTrue(
            ConditionalCommand(
                command.DriveSwerveAim(Robot.drivetrain, Field.coral_station.rightCenterFace.rotation().radians()),
                command.DriveSwerveAim(Robot.drivetrain, Field.coral_station.leftCenterFace.rotation().radians()),
                lambda: Field.odometry.getPose().nearest([Field.coral_station.leftCenterFace, Field.coral_station.rightCenterFace]) == Field.coral_station.rightCenterFace
            )
        )