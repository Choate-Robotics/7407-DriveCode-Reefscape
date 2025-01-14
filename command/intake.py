from toolkit.command import SubsystemCommand
import config
from subsystem import Intake

class RunIntake(SubsystemCommand[Intake]):
    """
    Runs until coral is detected
    """
    def initialize(self) -> None:
        self.subsystem.roll_in()
        self.subsystem.intake_running = True

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        self.subsystem.coral_in_intake = self.detect_coral()
        return self.subsystem.coral_in_intake
    
    def end(self, interrupted) -> None:
        self.subsystem.stop()
        self.subsystem.coral_in_intake = True
        self.subsystem.intake_running = False


class EjectIntake(SubsystemCommand[Intake]):
    """
    Eject coral from intake
    """
    def initialize(self) -> None:
        self.subsystem.roll_out()
        self.subsystem.coral_in_intake = True
        self.subsystem.intake_running = True

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        self.subsystem.coral_in_intake = self.detect_coral()
        return not self.subsystem.coral_in_intake

    def end(self, interrupted) -> None:
        self.subsystem.stop()
        self.subsystem.intake_running = False
        self.subsystem.coral_in_intake = False