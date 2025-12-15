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
            inverted=True,
            config=config.WRIST_FEED_CONFIG,
        )
        self.wrist_motor: TalonFX = TalonFX(
            config.wrist_id,
            config.foc_active,
            inverted=True,
            config=config.WRIST_CONFIG,
        )
        self.algae_motor: TalonFX = TalonFX(
            config.wrist_algae_id,
            config.foc_active,
            inverted=False,
            config=config.WRIST_ALGAE_CONFIG
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
        self.algae_motor.init()
        self.initial_zero()
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("wrist")
        self.wrist_angle = self.table.getDoubleTopic("wrist angle").publish() 
        self.target_angle = self.table.getDoubleTopic("target angle").publish()
        self.wrist_angle_moving = self.table.getBooleanTopic("target angle moving").publish()
        self.wrist_feeding = self.table.getBooleanTopic("wrist feeding").publish()
        self.wrist_ejecting = self.table.getBooleanTopic("wrist ejecting").publish()
        self.feed_current = self.table.getDoubleTopic("feed current").publish()
        self.wrist_zeroed = self.table.geBooleanTopic("wrist zeroed").publish()
        self.wrist_abs_pos = self.table.getDoubleTopic("wrist absolute position").publish()
        self.wrist_abs_ang = self.table.getDoubleTopic("wrist absolute angle").publish()
        self.calc_kg = self.table.getDoubleTopic("calculated kg").publish()
        self.wrist_applied_out = self.table.getDoubleTopic("wrist applied output").publish()
        self.wrist_current = self.table.getDoubleTopic("wrist current").publish()
        self.coral_in_feed = self.table.getDoubleTopic("coral in feed").publish()
        self.wrist_angle_diff = self.table.getDoubleTopic("wrist angle difference")
        self.algae_in_wrist = self.table.getBooleanTopic("algae in wrist").publish()
        self.algae_motor_current = self.table.getDoubleTopic("algae motor current").publish()

        
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

    def feed_out(self, speed) -> None:
        """
        Runs feed motors out to score or send coral from wrist to intake.
        
        """
        self.feed_motor.set_raw_output(speed)

    def feed_stop(self) -> None:
        """
        Stops feed motors.
        
        """

        self.feed_motor.set_raw_output(0)

    def set_coral(self, is_there_a_coral_in_the_feed: bool):
        """
        Says coral is in the feed

        """
        self.coral_in_feed = is_there_a_coral_in_the_feed

    def algae_in(self) -> None:
        self.algae_motor.set_voltage(config.wrist_algae_voltage)
    
    def algae_out(self) -> None:
        self.algae_motor.set_voltage(config.wrist_algae_extake_voltage)

    def algae_stop(self) -> None:
        self.algae_motor.set_voltage(0)

    def hold_algae(self, voltage = config.wrist_algae_hold_volts) -> None:
        self.algae_motor.set_voltage(voltage)

    def set_algae(self, algae_in_feed: bool) -> None:
        self.algae_in_wrist = algae_in_feed

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

    def is_at_angle(self, angle: radians, tolerance: radians = config.angle_threshold) -> bool:
        """
        Checks if the wrist angle is at an input angle.

        """
        self.table.putNumber("wrist angle tolerance", tolerance)
        return abs(self.get_wrist_angle() - angle) < tolerance

    def update_table(self) -> None:
        """
        update the network table with the wrist data
        """
        self.wrist_angle.set(math.degrees(self.get_wrist_angle))
        self.target_angle.set(math.degrees(self.target_angle))
        self.wrist_angle_moving.set(self.wrist_angle_moving)
        self.wrist_feeding.set(self.wrist_feeding)
        self.wrist_ejecting.set(self.wrist_ejecting)
        self.feed_current.set(self.feed_current.get_motor_current)
        self.wrist_zeroed.set(self.wrist_zeroed)
        self.wrist_abs_pos.set(self.encoder.get_absolute_position().value)
        self.wrist_abs_ang.set((math.degrees(self.encoder.get_absolute_position().value - config.wrist_encoder_zero)
            / constants.wrist_encoder_gear_ratio
            * 2
            * math.pi))
        self.calc_kg.set(config.wrist_max_ff * math.cos(self.get_wrist_angle() - config.wrist_ff_offset))
        self.wrist_applied_out.set(self.wrist_motor.get_applied_output())
        self.wrist_current.set(self.wrist_motor.get_motor_current())
        self.coral_in_feed.set(self.coral_in_feed)
        self.wrist_angle_diff.set((math.degrees(self.encoder.get_absolute_position().value - config.wrist_encoder_zero)                   
            / constants.wrist_encoder_gear_ratio
            * 2
            * math.pi)-self.get_wrist_angle())
        self.algae_in_wrist.set(self.algae_in_wrist)
        self.algae_motor_current.set(self.algae_motor.get_motor_current())
        
    def periodic(self) -> None:
        if config.NT_WRIST:
            self.update_table()
