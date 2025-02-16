from utils import LocalLogger
from oi.keymap import Keymap
import command
from robot_systems import Robot
import constants

log = LocalLogger("OI")


class OI:
    @staticmethod
    def init() -> None:
        log.info("Initializing OI...")

    @staticmethod
    def map_controls():
        log.info("Mapping controls...")
        pass

        # Keymap.Drivetrain.RESET_GYRO.onTrue(
        #     command.DrivetrainZero(Robot.drivetrain)) \
        #     .onFalse(command.DriveSwerveCustom(Robot.drivetrain))

        Keymap.Elevator.A.onTrue(
            command.SetElevator(Robot.elevator, constants.elevator_max_height/2)
        ).onFalse(command.SetElevator(Robot.elevator, 0))
        Keymap.Elevator.Y.onTrue(
            command.SetElevator(Robot.elevator, constants.elevator_max_height)
        ).onFalse(command.SetElevator(Robot.elevator, 0))