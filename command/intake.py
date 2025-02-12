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
        # TODO: fix comparing bool to float
        if self.subsystem.is_pivot_up() == self.subsystem.target_intake_position:
            return True

    def end(self, interrupted) -> None:
        self.subsystem.intake_pivoting = False

