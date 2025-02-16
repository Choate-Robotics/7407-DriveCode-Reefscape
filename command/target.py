from subsystem import Intake, Wrist
from config import TargetData
import command
import commands2
from commands2 import SequentialCommandGroup
from robot_systems import Robot


def target_command_generator(target: TargetData) -> SequentialCommandGroup:
    """
    Generates a command group to move the elevator and wrist to the target position,
    in an order that keeps the wrist from colliding with the elevator.

    Args:
        target: The target data to move to

    Returns:
        A command group to move the elevator and the wrist to the target position

    """
    elevator = Robot.elevator
    wrist = Robot.wrist
    target_elevator_height = target.elevator_height
    target_wrist_angle = target.wrist_angle
    target_command = SequentialCommandGroup()
    # Set elevator and wrist
    if not elevator.is_at_position(target_elevator_height):
        # Set wrist idle if necessary before setting the elevator
        if not wrist.is_at_angle(0):
            target_command.addCommands(command.SetWrist(wrist, 0))
        target_command.addCommands(
            command.SetElevator(elevator, target_elevator_height)
        )

    target_command.addCommands(command.SetWrist(wrist, target_wrist_angle))

    return target_command


class IntakeCoral(commands2.ParallelRaceGroup):
    def __init__(self, intake: Intake, wrist: Wrist):
        super().__init__(command.RunIntake(intake), command.FeedIn(wrist))


class EjectCoral(commands2.ParallelCommandGroup):
    def __init__(self, intake: Intake, wrist: Wrist):
        super().__init__(command.EjectIntake(intake), command.FeedOut(wrist))
