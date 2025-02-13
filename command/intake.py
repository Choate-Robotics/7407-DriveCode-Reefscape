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


class SetPivot(SubsystemCommand[Intake]):
    """
    sets the intake pivot to a target position
    """

    def __init__(self, subsystem: Intake, target_angle: float):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.subsystem.target_angle = target_angle

    def initialize(self) -> None:
        self.subsystem.intake_pivoting = True
        self.subsystem.set_pivot_angle(self.subsystem.target_angle)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.subsystem.get_pivot_angle() == self.subsystem.target_angle

    def end(self, interrupted) -> None:
        if not interrupted:
            self.subsystem.intake_pivoting = False
        else:
            log.warn("Intake pivot interrupted")

class ZeroPivot(SubsystemCommand[Intake]):
    """
    Zero the intake pivot
    """

    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.zero_pivot()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.subsystem.pivot_zeroed

    def end(self, interrupted) -> None:
        if not interrupted:
            log.info("Intake pivot zeroed")
        else:
            log.warn("Intake pivot zeroing interrupted")