from phoenix6 import StatusSignal
from toolkit.subsystem import Subsystem
from toolkit.motors.ctre_motors import TalonFX

import ntcore
import math
from math import pi
from units.SI import radians
from toolkit.utils.toolkit_math import bounded_angle_diff
from phoenix6.hardware import CANcoder
import config
import constants

import wpilib
from wpilib import Timer


class Wrist(Subsystem):
    def __init__(self):
        super().__init__()
        self.feed_motor: TalonFX = TalonFX(
            config.wrist_feed_id,
            config.foc_active,
            inverted=False,
            config=config.WRIST_FEED_CONFIG,
        )
        self.wrist_motor: TalonFX = TalonFX(
            config.wrist_id,
            config.foc_active,
            inverted=False,
            config=config.WRIST_CONFIG,
        )

        self.encoder: CANcoder = CANcoder(config.wrist_cancoder_id)

        self.wrist_angle: radians = 0
        self.target_angle: radians = 0
        self.wrist_angle_moving: bool = False
        self.wrist_feeding: bool = False
        self.wrist_ejecting: bool = False
        self.coral_in_feed: bool = False
        self.wrist_zeroed: bool = False

        self.in_timer = Timer()
        self.out_timer = Timer()

    def init(self):
        self.feed_motor.init()
        self.wrist_motor.init()
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("wrist")

    def initial_zero(self) -> None:
        """
        zero the wrist encoder
        """

        self.wrist_motor.set_sensor_position(
            self.encoder.get_absolute_position().value
            * constants.encoder_gear_ratio
            / constants.wrist_gear_ratio
        )
        self.wrist_angle = (
            self.encoder.get_absolute_position().value
            * constants.encoder_gear_ratio
            * 2
            * pi
        )
        self.wrist_zeroed = True

    # feed

    def feed_in(self) -> None:
        """
        spin feed motors in, used in command to stop
        """
        self.feed_motor.set_raw_output(1)

    def feed_out(self) -> None:
        """
        spin feed motors out, used in command to stop
        """
        self.feed_motor.set_raw_output(-1)

    def feed_stop(self) -> None:
        """
        stop the feed motors
        """
        self.feed_motor.set_raw_output(0)

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

    def set_wrist_angle(self, angle: radians) -> None:
        """
        move to motor until the wrist is at given angle
        """
        angle = self.limit_angle(angle)
        self.target_angle = angle

        ff = config.wrist_max_ff * math.cos(angle - config.wrist_ff_offset)

        self.wrist_motor.set_target_position(
            (angle / 2 * math.pi) * constants.wrist_gear_ratio,
            ff
        )

    def get_wrist_angle(self) -> radians:
        """
        get the current angle of the wrist
        A single full rotation corresponds to 2π radians
        returns angle in radians
        """
        return (
            (
                self.wrist_motor.get_sensor_position()
                * constants.wrist_gear_ratio
                / constants.encoder_gear_ratio
            )
            * pi
            * 2
        )

    def is_at_angle(self, angle: radians) -> bool:
        """
        check if the wrist angle is at the given angle
        """
        return (
            abs(bounded_angle_diff(self.get_wrist_angle(), angle))
            < config.angle_threshold
        )

    def update_table(self) -> None:
        """
        update the network table with the wrist data
        """

        self.table.putNumber("wrist angle", math.degrees(self.get_wrist_angle()))
        self.table.putNumber("target angle", math.degrees(self.target_angle))
        self.table.putBoolean("wrist moving", self.wrist_angle_moving)
        self.table.putBoolean("wrist feeding", self.wrist_feeding)
        self.table.putBoolean("wrist ejecting", self.wrist_ejecting)
        self.table.putNumber("feed current", self.feed_motor.get_motor_current())
        self.table.putBoolean("wrist zeroed", self.wrist_zeroed)

    def periodic(self) -> None:
        if config.NT_WRIST:
            self.update_table()
