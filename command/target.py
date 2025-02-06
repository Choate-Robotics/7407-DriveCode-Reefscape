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

class Target(SubsystemCommand[Elevator]):
    def __init__(
        self,
        elevator: Elevator,
        wrist: Wrist,
        intake: Intake,
        target: TargetData
    ):
        super().__init__(elevator)
        super().addRequirements(wrist)
        super().addRequirements(intake)
        
        self.elevator = elevator
        self.wrist = wrist
        self.intake = intake
        
        self.target = target
        self.finished = False
    
    def finish(self):
        self.finished = True

    def initialize(self):
        elevator_height = self.target.elevator_height
        wrist_angle = self.target.wrist_angle

        # INTAKE
        self.intake_command = ParallelCommandGroup()

        # Enabling
        if self.target.intake_enabled and not self.intake.intake_up:
            self.intake_command.addCommands(*[command.PivotIntake(self.intake)])

        # Running
        if self.target.intake_in_run and not self.intake.intake_rolling_in:
            self.intake_command.addCommands(*[command.RunIntake(self.intake)])
        elif self.target.intake_out_run and not self.intake.intake_rolling_out:
            self.intake_command.addCommands(*[command.EjectIntake(self.intake)])

        
        # ELEVATOR
        # TODO: Check if elevator is already at height and if elevator is moving
        self.elevator_command = command.SetElevator(
            self.elevator, 
            elevator_height
        )


        # WRIST
        # TODO: Double check logic of wrist to avoid collisions
        self.wrist_command = SequentialCommandGroup()

        # Pivot
        if not self.wrist.wrist_angle_moving:
            self.wrist_command.addCommands(*[
                command.SetWrist(
                    self.wrist,
                    wrist_angle
                )
            ])

        # Feed
        if self.target.wrist_feed_on and not self.wrist.wrist_feeding:
            self.wrist_command.addCommands(*[command.FeedIn(self.wrist)])
        elif self.target.wrist_score_on and not self.wrist.wrist_ejecting:
            self.wrist_command.addCommands(*[command.FeedOut(self.wrist)])

        # this is very subject to change
        commands2.CommandScheduler.getInstance()(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    self.intake_command, 
                    SequentialCommandGroup(
                        self.elevator_command,
                        self.wrist_command
                    )
                ),
                InstantCommand(lambda: self.finish()),
            )
        )
    
    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.finished
    
    def end(self) -> None:
        pass