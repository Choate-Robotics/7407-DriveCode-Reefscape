import utils 
import constants, config

from toolkit.command import SubsystemCommand
from subsystem import Elevator 
from units.SI import meters

class SetElevator(SubsystemCommand[Elevator]):
    """
    Set elevator to specified length.
    param length to set elevator to (float)
    in meters
    """
    def __init__(self, subsystem: Elevator, length: meters):
        super().__init__(subsystem)
        self.length: meters = length

    def initialize(self):
        
        self.length = self.subsystem.limit_length(self.length)

        self.subsystem.set_position(self.length,0)
        self.subsystem.elevator_moving = True

    def execute(self):
        pass

    def isFinished(self):
        self.subsystem.is_at_position(self.length)

    def end(self, interrupted: bool):
        """
        stops moving
        """
        self.subsystem.elevator_moving = False

    

        