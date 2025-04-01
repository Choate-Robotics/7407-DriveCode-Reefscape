from subsystem import Intake, Wrist, Elevator
from config import TargetData, wrist_idle_angle
import command
import commands2


class Target(commands2.SequentialCommandGroup):
    def __init__(self, target: TargetData, wrist: Wrist, elevator: Elevator):
        elevator_height = target.elevator_height
        wrist_angle = target.wrist_angle

        super().__init__(
            commands2.ConditionalCommand(
                commands2.SequentialCommandGroup(
                    command.SetWrist(wrist, wrist_idle_angle),
                    command.SetElevator(elevator, elevator_height)
                ),
                commands2.WaitCommand(0),
                lambda: not elevator.is_at_position(elevator_height)
            ),
            command.SetWrist(wrist, wrist_angle)
        )


class IntakeCoral(commands2.ParallelRaceGroup):
    def __init__(self, intake: Intake, wrist: Wrist):
        super().__init__(command.RunIntake(intake).unless(lambda: wrist.coral_in_feed), command.FeedIn(wrist))


class EjectCoral(commands2.ParallelCommandGroup):
    def __init__(self, intake: Intake, wrist: Wrist):
        super().__init__(command.EjectIntake(intake), command.FeedOut(wrist))
