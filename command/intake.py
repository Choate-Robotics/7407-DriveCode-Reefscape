from toolkit.command import SubsystemCommand
import config
from subsystem import Intake
from utils import LocalLogger

log = LocalLogger("Intake command")


class RunIntake(SubsystemCommand[Intake]):
    """
    Runs intake
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


class EjectIntake(SubsystemCommand[Intake]):
    """
    Eject coral from intake
    """

    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.roll_out()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        # we are assuming that another command will interrupt
        return False

    def end(self, interrupted) -> None:
        self.subsystem.stop()


class PivotIntake(SubsystemCommand[Intake]):
    """
    Pivots the intake to opposite position
    Note: True means pivot is up and vice versa
    """

    def __init__(self, subsystem: Intake, target_intake_position: bool):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.subsystem.target_intake_position = target_intake_position

    def initialize(self) -> None:
        self.subsystem.intake_pivoting = True

        if self.subsystem.target_intake_position:
            self.subsystem.pivot_down()
        else:
            self.subsystem.pivot_up()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.subsystem.target_intake_position:
            # trying to go up
            return self.subsystem.is_pivot_up()
        else:
            # trying to go down
            return self.subsystem.is_pivot_down()

    def end(self, interrupted) -> None:
        if not interrupted:
            self.subsystem.intake_pivoting = False
        else:
            log.warn("Intake pivot interrupted")
