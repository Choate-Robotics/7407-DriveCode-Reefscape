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
        
        #INTAKE
        Keymap.Intake.INTAKE_CORAL.whileTrue(
            command.RunIntake(Robot.intake))
        
        Keymap.Intake.EJECT_CORAL.whileTrue(
            command.EjectIntake(Robot.intake))
        
        #WRIST
        Keymap.Wrist.EXTAKE_CORAL.whileTrue(
            command.FeedOut(Robot.wrist))
