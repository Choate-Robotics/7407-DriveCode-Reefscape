from subsystem import (
    Elevator,
    Intake,
    Wrist
)
from toolkit.command import SubsystemCommand
from config import TargetData
import command
import commands2
from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand

class Target(commands2.Command):
    def __init__(
        self,
        elevator: Elevator,
        wrist: Wrist,
        intake: Intake,
        target: TargetData
    ):  
        self.elevator = elevator
        self.wrist = wrist
        self.intake = intake
        
        self.target = target
        self.finished = False
    
    def finish(self):
        self.finished = True

    def initialize(self):
        target_elevator_height = self.target.elevator_height
        target_wrist_angle = self.target.wrist_angle

        self.giraffe = SequentialCommandGroup()


        # Set elevator and wrist
        if not self.elevator.is_at_position(target_elevator_height):
            # Set wrist idle if necessary before setting the elevator
            if not self.wrist.is_at_angle(0):
                self.giraffe.addCommands(command.SetWrist(self.wrist, 0))
            self.giraffe.addCommands(command.SetElevator(self.elevator, target_elevator_height))
        
        self.giraffe.addCommands(command.SetWrist(self.wrist, target_wrist_angle))

        # Intake commands
        self.intake_command = SequentialCommandGroup()
        self.intake_command.addCommands(command.PivotIntake(self.intake, self.target.intake_idle))

        # if self.target.intake_in_run:
        #     self.intake_command.addCommands(command.RunIntake(self.intake))
        # elif self.target.intake_out_run:
        #     self.intake_command.addCommands(command.EjectIntake(self.intake))

        commands2.CommandScheduler.getInstance()(
            ParallelCommandGroup(
                self.intake_command, 
                self.giraffe
            ),
            InstantCommand(lambda: self.finish()),
        )
    
    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.finished
    
    def end(self) -> None:
        pass