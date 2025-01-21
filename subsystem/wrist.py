from toolkit.subsystem import Subsystem
from toolkit.motors.ctre_motors import TalonFX
from phoenix6.hardware import cancoder

import math
from math import pi
from units.SI import radians
from toolkit.utils.toolkit_math import bounded_angle_diff

import config
import constants

class Wrist(Subsystem):
    def __init__(self):
        super().__init__()
        self.feed_motor: TalonFX = TalonFX(
            config.wrist_feed_id,
            config.foc_active,
            inverted=False,
            config=config.WRIST_FEED_CONFIG
        )
        self.wrist_motor: TalonFX = TalonFX(
            config.wrist_id,
            config.foc_active,
            inverted=False,
            config=config.WRIST_CONFIG
        )

        self.coral_in_wrist: bool = False
        self.wrist_angle: radians = 0
        self.target_angle: radians = 0
        self.wrist_moving: bool = False
        self.coral_in_feed: bool = False

    def init(self):
        self.feed_motor.init()
        self.wrist_motor.init()

#side roller put it in, side roller kick it out after releasing it, have

# feed

    def feed_in(self):
        """
        spin feed motors in, used in command to stop
        """
        self.feed_motor.set_raw_output(1)

    def feed_out(self):
        """
        spin feed motors out, used in command to stop
        """
        self.feed_motor.set_raw_output(-1)

    def feed_stop(self):
        """
        stop the feed motors
        """
        self.feed_motor.set_target_velocity(0)

    def coral_detected(self):
        """
        check if there is coral in the feed
        """
        pass

# wrist

    def limit_angle(self, angle: radians) -> radians:
        """
        limits if the given angle in radians is within the range of the wrist
        if it is out of range, it returns the min or max angle in radians
        otherwise it returns the given angle

        takes in angle in radians
        """
        if angle <= config.wrist_min_angle:
            return config.wrist_min_angle
        elif angle >= config.wrist_max_angle:
            return config.wrist_max_angle
        return angle

    def set_wrist_angle(self, angle: radians):
        """
        move to motor until the wrist is at given angle
        """
        angle = self.limit_angle(angle)
        self.target_angle = angle

        self.wrist_motor.set_target_position(
            (angle / 2*math.pi ) * constants.wrist_gear_ratio
        )

    def get_wrist_angle(self) -> radians:
        """
        get the current angle of the wrist
        A single full rotation corresponds to 2π radians
        returns angle in radians
        """
        return (
                (self.wrist_motor.get_sensor_position() / constants.wrist_gear_ratio)
                * pi
                * 2
        )
        

    def is_at_angle(self, angle: radians):
        """
        check if the wrist angle is at the given angle
        """
        return abs(bounded_angle_diff(self.get_wrist_angle(), angle)) < config.angle_threshold

