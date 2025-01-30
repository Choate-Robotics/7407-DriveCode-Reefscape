from toolkit.command import SubsystemCommand
import config
from subsystem import Intake

class RunIntake(SubsystemCommand[Intake]):
    """
    Runs until coral is detected
    """
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.roll_in()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        # we are assuming that another command will interrupt
        return False
    
    def end(self, interrupted) -> None:
        self.subsystem.stop()
        self.subsystem.coral_in_intake = True


class EjectIntake(SubsystemCommand[Intake]):
    """
    Eject coral from intake
    """
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.roll_out()
        self.subsystem.coral_in_intake = True

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        # we are assuming that another command will interrupt
        return False

    def end(self, interrupted) -> None:
        self.subsystem.stop()
        self.subsystem.coral_in_intake = False

class PivotIntake(SubsystemCommand[Intake]):
    """
    Pivots the intake to opposite position
    """
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.intake_pivoting = True

        if self.subsystem.is_pivot_up():
            self.subsystem.pivot_down()
            self.subsystem.target_position = False
        else:
            self.subsystem.pivot_up()
            self.subsystem.target_position = True

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.subsystem.is_pivot_up() == self.subsystem.target_position:
            return True

    def end(self, interrupted) -> None:
        self.subsystem.intake_pivoting = False