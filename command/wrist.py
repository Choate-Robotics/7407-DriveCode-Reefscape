from toolkit.command import SubsystemCommand
import config
from subsystem import Wrist

from math import radians


class setWrist(SubsystemCommand[Wrist]):

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
     
    def __init__(self, subsystem: Wrist, angle: radians):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.feed_in()
        self.subsystem.coral_in_feed = False
    
    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.coral_in_feed

    def end(self, interrupted) -> None:
        self.subsystem.feed_stop()


class FeedOut(SubsystemCommand[Wrist]):
     
    def __init__(self, subsystem: Wrist, angle: radians):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.feed_out()
        self.subsystem.coral_in_feed = True
    
    def execute(self):
        pass

    def isFinished(self):
        return not self.subsystem.coral_in_feed

    def end(self, interrupted) -> None:
        self.subsystem.feed_stop()