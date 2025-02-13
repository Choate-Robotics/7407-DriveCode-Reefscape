import utils 
import constants, config

from toolkit.command import SubsystemCommand
from subsystem import Elevator 
from units.SI import meters

class SetElevator(SubsystemCommand[Elevator]):
    """
    Set elevator to specified height.
    param height to set elevator to (float)
    in meters
    """
    def __init__(self, subsystem: Elevator, height: meters):
        super().__init__(subsystem)
        self.height: meters = height
        self.subsystem = subsystem

    def initialize(self):
        
        self.height = self.subsystem.limit_height(self.height)

        self.subsystem.set_position(self.height)
        self.subsystem.elevator_moving = True

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.subsystem.is_at_position(self.height)

    def end(self, interrupted: bool):
        """
        stops moving
        """
        if interrupted: 
            self.subsystem.stop()
        
        self.subsystem.elevator_moving = False

class ZeroElevator(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: Elevator):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.set_elevator_climb_down()

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.subsystem.magsensor.get()
    
    def end(self, interrupted: bool):
        self.subsystem.stop()
        self.subsystem.zero()
