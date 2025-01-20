import utils 
import constants, config

from toolkit.command import SubsystemCommand
from subsystem import Elevator 

class SetElevator(SubsystemCommand[Elevator]):
    """
    Set elevator to specified length.
    param length to set elevator to (float)
    in meters
    """
    def __init__(self, subsystem: Elevator, length: float):
        super().__init__(subsystem)
        self.length: float = length

    def initialize(self):
        
        self.length = self.subsystem.limit_length(self.length)

        self.subsystem.set_length(self.length,0)
        self.subsystem.elevator_moving = True

    def execute(self):
        pass

    def isFinished(self):
        # Rounding to make sure it's not too precise (will cause err)
        return round(self.subsystem.get_length(), 2) == round(self.length, 2)

    def end(self, interrupted: bool):
        """
        stops moving
        """
        self.subsystem.elevator_moving = False

    

        