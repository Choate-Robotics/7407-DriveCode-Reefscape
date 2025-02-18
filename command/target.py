from subsystem import Intake, Wrist, Elevator
from config import TargetData
import command
import commands2


class Target(commands2.SequentialCommandGroup):
    def __init__(self, target: TargetData, wrist: Wrist, elevator: Elevator):
        elevator_height = target.elevator_height
        wrist_angle = target.wrist_angle

        super().__init__(
            commands2.ConditionalCommand(
                commands2.SequentialCommandGroup(
                    command.SetWrist(wrist, 0),
                    command.SetElevator(elevator, elevator_height)
                ),
                commands2.WaitCommand(0),
                lambda: not elevator.is_at_position(elevator_height)
            ),
            command.SetWrist(wrist, wrist_angle)
        )


class IntakeCoral(commands2.ParallelRaceGroup):
    def __init__(self, intake: Intake, wrist: Wrist):
        super().__init__(command.RunIntake(intake), command.FeedIn(wrist))


class EjectCoral(commands2.ParallelCommandGroup):
    def __init__(self, intake: Intake, wrist: Wrist):
        super().__init__(command.EjectIntake(intake), command.FeedOut(wrist))
