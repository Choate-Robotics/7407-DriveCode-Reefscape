from toolkit.command import SubsystemCommand
import config
from subsystem import Wrist

from math import radians
from wpilib import Timer

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
        self.subsystem.wrist_angle_moving = True
    
    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.subsystem.is_at_angle(self.angle)

    def end(self, interrupted) -> None:
        self.subsystem.wrist_angle_moving = False


class ZeroWrist(SubsystemCommand[Wrist]):
    """
    Zero the wrist encoder
    """
    def __init__(self, subsystem: Wrist):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.initial_zero()
    
    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.subsystem.wrist_zeroed

    def end(self, interrupted) -> None:
        pass


class FeedIn(SubsystemCommand[Wrist]):
    """
    run the feed intake in until there is coral detected in the feed
    checks if the current is over the threshold for a period of time
    """
    def __init__(self, subsystem: Wrist):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.feed_in()
        self.subsystem.in_timer = Timer()
        self.subsystem.wrist_feeding = True

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.subsystem.feed_motor.get_motor_current() > config.back_current_threshold:
            
            if not self.subsystem.in_timer.isRunning():
                self.subsystem.in_timer.start()
            
            self.subsystem.coral_in_feed = self.subsystem.in_timer.hasElapsed(config.current_time_threshold)
            
        else:
            self.subsystem.in_timer.stop()
            self.subsystem.in_timer.reset()
            self.subsystem.coral_in_feed = False

        return self.subsystem.coral_in_feed

    def end(self, interrupted) -> None:
        self.subsystem.feed_stop()
        self.subsystem.wrist_feeding = False


class FeedOut(SubsystemCommand[Wrist]):
    """
    run the feed out until coral is no longer detected in the feed
    """
    def __init__(self, subsystem: Wrist):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.feed_out()
        self.subsystem.out_timer = Timer()
        self.subsystem.wrist_ejecting = True
    
    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.subsystem.feed_motor.get_motor_current() < config.out_current_threshold:
            
            if not self.subsystem.out_timer.isRunning():
                self.subsystem.out_timer.start()
            
            self.subsystem.coral_in_feed = not self.subsystem.out_timer.hasElapsed(config.current_time_threshold)
            
        else:
            self.subsystem.out_timer.stop()
            self.subsystem.out_timer.reset()
            self.subsystem.coral_in_feed = True

        return self.subsystem.coral_in_feed
    
    def end(self, interrupted) -> None:
        self.subsystem.feed_stop()
        self.subsystem.wrist_ejecting = False

