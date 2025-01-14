from toolkit.subsystem import Subsystem
from toolkit.motors.ctre_motors import TalonFX
from phoenix6.hardware import cancoder

import math
from math import pi
from units.SI import radians

import config
import constants

class Wrist(Subsystem):
    def __init__(self):
        super().__init__()
        self.feed_motor: TalonFX = TalonFX(
            config.wrist_feed_id,
            config.foc_active,
            inverted = False,
            config = config.WRIST_FEED_CONFIG
        )
        self.wrist_motor: TalonFX = TalonFX(
            config.wrist_id,
            config.foc_active,
            inverted = False,
            config = config.WRIST_CONFIG
        )

        self.coral_in_wrist: bool = False
        self.wrist_angle: radians = 0
        self.target_angle: radians = 0

    def init(self):
        self.feed_motor.init()
        self.wrist_motor.init()

#side roller put it in, side roller kick it out after releasing it, have

# feed

    def feed_in(self):
        pass

    def feed_out(self):
        pass

    def feed_stop(self):
        pass

    def coral_detected(self):
        pass

# wrist

    def limit_angle(self, angle: radians) -> radians:
        if angle <= constants.wrist_min_angle:
            return constants.wrist_min_angle
        elif angle >= constants.wrist_max_angle:
            return constants.wrist_max_angle
        return angle

    def get_wrist_angle(self):
        pass

    def set_wrist_angle(self, angle: radians):

        angle = self.limit_angle(angle)
        self.target_angle = angle

        self.wrist_motor.set_target_position(
            ( angle / 2*math.pi ) * constants.wrist_gear_ratio
        )

