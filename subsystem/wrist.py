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
            config=config.WRIST_FEED_CONFIG
        )
        self.wrist_motor: TalonFX = TalonFX(
            config.wrist_id,
            config.foc_active,
            inverted=False,
            config=config.WRIST_CONFIG
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

        self.in_timer = Timer()
        self.out_timer = Timer()

    def init(self):
        self.feed_motor.init()
        self.wrist_motor.init()
        self.table = ntcore.NetworkTableInstance.getDefault().getTable('wrist')

    def initial_zero(self) -> None:
        """
        Zeros the wirst.

        """
        
        self.wrist_motor.set_sensor_position(
            self.encoder.get_absolute_position() / constants.wrist_gear_ratio
        )

        self.wrist_angle = (
            self.encoder.get_absolute_position() / constants.wrist_gear_ratio
            * math.pi
            * 2
        )
        self.wrist_zeroed = True

    def feed_in(self) -> None:
        """
        Runs feed motors in to send coral from intake to wrist.
        
        """

        self.feed_motor.set_raw_output(config.wrist_feed_in_speed)

    def feed_out(self) -> None:
        """
        Runs feed motors out to score or send coral from wrist to intake.
        
        """

        self.feed_motor.set_raw_output(config.wrist_feed_out_speed)

    def feed_stop(self) -> None:
        """
        Stops feed motors.
        
        """

        self.feed_motor.set_raw_output(0)

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

        self.wrist_motor.set_target_position(
            (self.target_angle / 2 * math.pi ) 
            * constants.wrist_gear_ratio
        )

    def get_wrist_angle(self) -> radians:
        """
        Gets the current wrist angle.

        """
        
        return (
            (self.wrist_motor.get_sensor_position() / constants.wrist_gear_ratio)
            * math.pi
            * 2
        )
        

    def is_at_angle(self, angle: radians) -> bool:
        """
        Checks if the wrist angle is at an input angle.

        """
        
        return abs(bounded_angle_diff(self.get_wrist_angle(), angle)) < config.wrist_angle_threshold
    
    def update_table(self) -> None:
        self.table = ntcore.NetworkTableInstance.getDefault().getTable('wrist')

        self.table.putNumber('Wrist Angle', math.degrees(self.get_wrist_angle()))
        self.table.putNumber('Target Angle', math.degrees(self.target_angle))
        self.table.putBoolean('Wrist Moving', self.wrist_angle_moving)
        self.table.putBoolean('Wrist Feeding', self.wrist_feeding)
        self.table.putBoolean('wrist Ejecting', self.wrist_ejecting)
        self.table.putNumber('Feed Current', self.feed_motor.get_motor_current())
        self.table.putBoolean('Wrist Zeroed Initially', self.wrist_zeroed)
