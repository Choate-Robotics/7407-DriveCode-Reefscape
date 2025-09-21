import utils
import config

from toolkit.command import SubsystemCommand
from subsystem import Elevator
from units.SI import meters

log = utils.LocalLogger("Elevator Commands")


class SetElevator(SubsystemCommand[Elevator]):
    """
    Set elevator to specified height.
    param height to set elevator to (float)
    in meters
    """

    def __init__(self, subsystem: Elevator, height: meters, tolerance: meters = config.elevator_height_threshold):
        super().__init__(subsystem)
        self.height: meters = height
        self.subsystem = subsystem
        self.tolerance = tolerance

    def initialize(self):
        self.height = self.subsystem.limit_height(self.height)

        self.subsystem.set_position(self.height)
        self.subsystem.elevator_moving = True

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.subsystem.is_at_position(self.height, self.tolerance)

    def end(self, interrupted: bool):
        """
        stops moving
        """
        if interrupted: 
            # self.subsystem.stop()
            log.warn("Elevator command interrupted")

        self.subsystem.elevator_moving = False
