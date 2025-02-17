from toolkit.command import SubsystemCommand

import config
from subsystem import Intake
from utils import LocalLogger
from wpimath.filter import Debouncer

from units.SI import radians

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

    def __init__(self, subsystem: Intake, target_angle: radians):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.target_angle = target_angle

    def initialize(self) -> None:
        self.subsystem.intake_pivoting = True
        self.subsystem.set_pivot_angle(self.target_angle)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.subsystem.is_at_angle(self.target_angle)

    def end(self, interrupted) -> None:
        if interrupted:
            self.subsystem.stop_pivot()
            log.warn("Intake pivot interrupted")
        self.subsystem.intake_pivoting = False


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

class IntakeAlgae(SubsystemCommand[Intake]):

    def __init__(self, subsystem):
        super().__init__(subsystem)
        self.subsystem = subsystem 

    def initialize(self):
        self.subsystem.intake_algae()

        # self.debouncer = Debouncer(config.intake_current_time_threshold, Debouncer.DebounceType.kRising)
        
    def execute(self):
        pass 

    def isFinished(self):                       
        # return self.debouncer.calculate(
        #     self.subsystem.get_horizontal_motor_current()
        #     > config.intake_current_threshold
        # )
        return False

    def end(self, interrupted):
        self.subsystem.stop()
        self.subsystem.algae_in_intake = True

class ExtakeAlgae(SubsystemCommand[Intake]):

    def __init__(self, subsystem):
        super().__init__(subsystem)
        self.subsystem = subsystem 

    def initialize(self):
        self.subsystem.extake_algae()

    def execute(self):
        pass 

    def isFinished(self):                       
        return False
    
    def end(self):
        self.subsystem.algae_in_intake = False
        self.subsystem.stop()
