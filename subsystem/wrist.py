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
        self.wrist_moving: bool = False
        self.coral_in_feed: bool = False

        self.in_timer = Timer()
        self.out_timer = Timer()

    def init(self):
        self.feed_motor.init()
        self.wrist_motor.init()
        self.table = ntcore.NetworkTableInstance.getDefault().getTable('wrist')

    def initial_zero(self):
        """
        zero the wrist encoder
        """
        self.wrist_motor.set_sensor_position(
            self.encoder.get_absolute_position()/constants.wrist_gear_ratio
        )
        self.wrist_angle = (self.encoder.get_absolute_position()/constants.wrist_gear_ratio
        * pi
        * 2
        )
        

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

    # def coral_in_wrist(self) -> bool:
    #     """
    #     check if there is coral in the feed 
    #     checks if the current is over the threshold for a period of time
    #     """
    #     if self.feed_motor.get_motor_current() > config.current_threshold:
            
    #         if not self.timer.isRunning():
    #             self.timer.start()
            
    #         self.coral_in_feed = self.timer.hasElapsed(config.current_time_threshold)

    #     else:
    #         self.timer.stop()
    #         self.timer.reset()
    #         self.coral_in_feed = False
        
    #     return self.coral_in_feed  
    
    # def coral_in_back(self) -> bool:
    #     """
    #     checks if the coral is in in the back of the wrist feed
    #     checks if the highest current is held for a period of time
    #     """
    #     if self.feed_motor.get_motor_current() > config.back_current_threshold:
    #         if not self.back_timer.isRunning():
    #             self.back_timer.start()

    #         self.coral_in_back_feed = self.back_timer.hasElapsed(config.current_time_threshold)
    #         self.coral_in_feed = self.coral_in_back_feed

    #     else:
    #         self.back_timer.stop()
    #         self.back_timer.reset()
        
    #     return self.coral_in_back_feed 


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
        

    def is_at_angle(self, angle: radians) -> radians:
        """
        check if the wrist angle is at the given angle
        """
        return abs(bounded_angle_diff(self.get_wrist_angle(), angle)) < config.angle_threshold
    
    def update_table(self) -> None:
        """
        update the network table with the wrist data
        """
        table = ntcore.NetworkTableInstance.getDefault().getTable('wrist')

        self.table.putNumber('wrist angle', math.degrees(self.get_wrist_angle()))
        self.table.putNumber('target angle', math.degrees(self.target_angle))
        self.table.putBoolean('wrist moving', self.wrist_moving)
        self.table.putNumber('feed current', self.feed_motor.get_motor_current())
        self.table.putBoolean('coral in feed', self.coral)

    def periodic(self) -> None:
        
        if config.NT_WRIST:

            self.update_table()