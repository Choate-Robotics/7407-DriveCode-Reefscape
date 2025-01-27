from toolkit.command import SubsystemCommand
import config
from subsystem import Wrist

from math import radians


class SetWrist(SubsystemCommand[Wrist]):
    """
    Set the wrist to a specific angle.
    """
    def __init__(self, subsystem: Wrist, angle: radians):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.angle = angle

    def initialize(self) -> None:
        self.subsystem.set_wrist_angle(self.angle)
        self.subsystem.wrist_moving = True
    
    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.is_at_angle(self.angle)

    def end(self, interrupted) -> None:
        self.subsystem.wrist_moving = False


class FeedIn(SubsystemCommand[Wrist]):
    """
    run the feed intake in until there is coral detected in the feed
    """
    def __init__(self, subsystem: Wrist, angle: radians):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.feed_in()

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.coral_detected()
        

    def end(self, interrupted):
        self.subsystem.feed_stop()
        return self.subsystem.coral_in_feed


class FeedOut(SubsystemCommand[Wrist]):
    """
    run the feed out until coral is no longer detected in the feed
    """
    def __init__(self, subsystem: Wrist, angle: radians):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.feed_out()
    
    def execute(self):
        pass

    def isFinished(self):
        return not self.subsystem.coral_detected()

    def end(self, interrupted) -> bool:
        self.subsystem.feed_stop()
        return not self.subsystem.coral_in_feed