from toolkit.command import SubsystemCommand
import config
from subsystem import Wrist

from units.SI import radians
from wpimath.filter import Debouncer
from utils import LocalLogger

log = LocalLogger("Wrist Command")


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
        if not interrupted:
            self.subsystem.wrist_angle_moving = False
        else:
            log.error("Wrist command interrupted")


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
        if interrupted:
            log.error("Wrist zeroing command interrupted")


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
        self.subsystem.wrist_feeding = True

        self.debouncer = Debouncer(
            config.current_time_threshold, Debouncer.DebounceType.kRising
        )

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.debouncer.calculate(
            self.subsystem.feed_motor.get_motor_current()
            > config.back_current_threshold
        )

    def end(self, interrupted) -> None:
        if interrupted:
            log.warn("Feed in command interrupted")
        if not interrupted:
            self.subsystem.coral_in_feed = True
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
        self.subsystem.wrist_ejecting = True

        # self.debouncer = Debouncer(
        #     config.current_time_threshold, Debouncer.DebounceType.kRising)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        # return self.debouncer.calculate(
        #     self.subsystem.feed_motor.get_motor_current() 
        #     < config.out_current_threshold
        #     )
        return False

    def end(self, interrupted) -> None:
        if interrupted:
            log.warn("Feed out command interrupted")
        self.subsystem.coral_in_feed = False
        self.subsystem.feed_stop()
        self.subsystem.wrist_ejecting = False

class FeedOutForDistance(SubsystemCommand[Wrist]):
    """
    run out the feed motor for a set amount of rotations
    """
    def __init__(self, subsystem: Wrist):
        super().__init__(subsystem)
        self.subsystem = subsystem
        

    def initialize(self) -> None:
        self.subsystem.feed_out()
        self.subsystem.wrist_ejecting = True
        self.subsystem.cummulative_extake_distance = self.subsystem.feed_motor.get_sensor_position()
        

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.subsystem.feed_motor.get_sensor_position()  - self.subsystem.cummulative_extake_distance < config.wrist_extake_distance

    def end(self, interrupted) -> None:
        self.subsystem.feed_stop()
        self.subsystem.wrist_ejecting = False
        self.subsystem.cummulative_extake_distance = self.subsystem.feed_motor.get_sensor_position()

class WristAlgaeIn(SubsystemCommand[Wrist]):
    """
    run the feed motor to get the algae
    """

    def __init__(self, subsystem: Wrist):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.algae_in()
        self.subsystem.algae_running_in = True

        # self.debouncer = Debouncer(
        #     config.current_time_threshold, Debouncer.DebounceType.kRising
        # )

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        # return self.debouncer.calculate(
        #     self.subsystem.feed_motor.get_motor_current()
        #     > config.back_current_threshold
        # )
        return False

    def end(self, interrupted) -> None:
        if interrupted:
            log.warn("Algae in command interrupted")
        self.subsystem.feed_stop()
        self.subsystem.algae_running_in = False
        # self.subsystem.algae_in_wrist = True


class WristAlgaeOut(SubsystemCommand[Wrist]):
    """
    run the feed motor to get the algae
    """

    def __init__(self, subsystem: Wrist):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.feed_out()
        self.subsystem.algae_running_out = True

        self.debouncer = Debouncer(
            config.current_time_threshold, Debouncer.DebounceType.kRising
        )

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.debouncer.calculate(
            self.subsystem.feed_motor.get_motor_current()
            < config.wrist_algae_time_threshold
        )

    def end(self, interrupted) -> None:
        if interrupted:
            log.warn("Algae out command interrupted")
        self.subsystem.feed_stop()
        self.subsystem.algae_running_out = False
        self.subsystem.algae_in_wrist = False
