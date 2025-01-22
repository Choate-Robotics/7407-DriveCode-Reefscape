from abc import ABC, abstractmethod
from toolkit.subsystem import Subsystem

class Intake(Subsystem, ABC):
    
    @abstractmethod
    def roll_in(self):
        pass

    @abstractmethod
    def roll_out(self):
        pass

    @abstractmethod
    def intake_stop(self):
        pass
