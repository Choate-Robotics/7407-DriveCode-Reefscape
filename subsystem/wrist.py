from phoenix6 import StatusSignal
from toolkit.subsystem import Subsystem
from toolkit.motors.ctre_motors import TalonFX

import ntcore
import math
from units.SI import radians
from toolkit.utils.toolkit_math import bounded_angle_diff
from phoenix6.hardware import CANcoder
import config
import constants
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
            inverted=True,
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
        self.table = None

        self.algae_in_wrist: bool = False
        self.algae_running_in: bool = False
        self.algae_running_out: bool = False

    def init(self):
        self.feed_motor.init()
        self.wrist_motor.init()
        self.initial_zero()
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("wrist")

    def initial_zero(self) -> None:
        """
        Zeros the wirst.

        """
        self.wrist_angle = (
            (self.encoder.get_absolute_position().value - config.wrist_encoder_zero)
            / constants.wrist_encoder_gear_ratio
            * 2
            * math.pi
        )

        self.wrist_motor.set_sensor_position(
            self.wrist_angle
            * constants.wrist_gear_ratio / 2 / math.pi
        )
        
        self.wrist_zeroed = True

    # feed

    def feed_in(self) -> None:
        """
        Runs feed motors in to send coral from intake to wrist.
        
        """
        self.feed_motor.set_raw_output(config.wrist_intake_speed)

    def algae_in(self) -> None:
        self.feed_motor.set_raw_output(config.wrist_algae_speed)

    def feed_out(self) -> None:
        """
        Runs feed motors out to score or send coral from wrist to intake.
        
        """
        self.feed_motor.set_raw_output(config.wrist_extake_speed)

    def feed_stop(self) -> None:
        """
        Stops feed motors.
        
        """

        self.feed_motor.set_raw_output(0)

    # wrist

    def limit_angle(self, angle: radians) -> radians:
        """
        Limits given angle within the range of wrist. 
        Returns the minimum or maximum wrist angle if input angle is outside range.

        Args:
            angle (radians): target wrist angle
        
        Returns:
            radians: new limited angle
        """

        if angle <= config.wrist_min_angle:
            return config.wrist_min_angle
        elif angle >= config.wrist_max_angle:
            return config.wrist_max_angle
        return angle


    def set_wrist_angle(self, angle: radians) -> None:
        """
        Sets the wrist at target angle.

        Args:
            radians: target wrist angle
        """

        self.target_angle = self.limit_angle(angle)

        ff = config.wrist_max_ff * math.cos(angle - config.wrist_ff_offset)

        self.wrist_motor.set_target_position(
            (angle / (2 * math.pi)) * constants.wrist_gear_ratio,
            ff
        )

    def get_wrist_angle(self) -> radians:
        """
        Gets the current wrist angle.

        """
        
        return (
            (
                self.wrist_motor.get_sensor_position()
                / constants.wrist_gear_ratio
            )
            * math.pi
            * 2
        )

    def is_at_angle(self, angle: radians) -> bool:
        """
        Checks if the wrist angle is at an input angle.

        """
        return abs(self.get_wrist_angle() - angle) < config.angle_threshold

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
        self.table.putNumber("wrist absolute position", self.encoder.get_absolute_position().value)
        self.table.putNumber("wrist absolute angle", (self.encoder.get_absolute_position().value - config.wrist_encoder_zero)
            / constants.wrist_encoder_gear_ratio
            * 2
            * math.pi)
        self.table.putNumber("calculated kG", config.wrist_max_ff * math.cos(self.get_wrist_angle() - config.wrist_ff_offset))
        self.table.putNumber("wrist applied output", self.wrist_motor.get_applied_output())
        self.table.putNumber("wrist current", self.wrist_motor.get_motor_current())
        self.table.putBoolean("coral in feed", self.coral_in_feed)

    def periodic(self) -> None:
        if config.NT_WRIST:
            self.update_table()
