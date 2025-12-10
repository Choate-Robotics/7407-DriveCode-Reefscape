from toolkit.subsystem import Subsystem

import ntcore
import math
from units.SI import radians
from toolkit.utils.toolkit_math import bounded_angle_diff
from phoenix6 import hardware, configs, signals, StatusCode, controls
import config
import constants


class Wrist(Subsystem):
    def __init__(self):
        super().__init__()

        self.feed_motor: hardware.TalonFX = hardware.TalonFX(config.wrist_feed_id, "rio")
        self.feed_cfg = configs.TalonFXConfiguration()
        self.feed_cfg.current_limits.supply_current_limit = 60
        self.feed_cfg.slot0.k_p = 1.0
        self.feed_cfg.motor_output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.feed_duty_cycle = controls.DutyCycleOut(0)


        self.wrist_motor: hardware.TalonFX = hardware.TalonFX(config.wrist_id, "rio")
        self.wrist_cfg = configs.TalonFXConfiguration()
        self.wrist_cfg.slot0.k_p = 48.0
        self.wrist_cfg.slot0.k_s = 0.06
        self.wrist_cfg.slot0.k_g = 0.17
        self.wrist_cfg.motion_magic.motion_magic_cruise_velocity = 97.75
        self.wrist_cfg.motion_magic.motion_magic_acceleration = 350
        self.wrist_cfg.feedback.sensor_to_mechanism_ratio = 45
        self.wrist_cfg.feedback.feedback_rotor_offset = math.radians(30)
        self.wrist_cfg.motor_output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.wrist_control = controls.MotionMagicVoltage(0.0)

        self.algae_motor: hardware.TalonFX = hardware.TalonFX(config.wrist_algae_id, "rio")
        self.algae_cfg = configs.TalonFXConfiguration()
        self.algae_cfg.current_limits.supply_current_limit = 40
        self.algae_cfg.slot0.k_p = 1.0
        self.algae_cfg.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        self.algae_duty_cycle = controls.DutyCycleOut(0)
        self.algae_volts = controls.VoltageOut(0.0)

        self.encoder: hardware.CANcoder = hardware.CANcoder(config.wrist_cancoder_id, "rio")

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
        self._apply_with_retry(self.feed_motor, self.feed_cfg, "feed")
        self._apply_with_retry(self.wrist_motor, self.wrist_cfg, "wrist")
        self._apply_with_retry(self.algae_motor, self.algae_cfg, "algae")

        self.initial_zero()
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("wrist")

    def _apply_with_retry(self, motor: hardware.TalonFX, cfg: configs.TalonFXConfiguration, name: str):
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(5):
            status = motor.configurator.apply(cfg)
            if status.is_ok():
                return
        # If we get here, it never succeeded
        print(f"[WristSubsystem] Could not apply config for {name} motor, error code: {status.name}")

    def initial_zero(self) -> None:
        """
        Zeros the wirst.

        """
        self.wrist_angle = (
            (self.encoder.get_position() - config.wrist_encoder_zero)
            / constants.wrist_encoder_gear_ratio
            * 2
            * math.pi
        )

        self.wrist_motor.get_position(
            self.wrist_angle
            * constants.wrist_gear_ratio / 2 / math.pi
        )
        
        self.wrist_zeroed = True

    # feed

    def feed_in(self) -> None:
        """
        Runs feed motors in to send coral from intake to wrist.
        
        """
        self.feed_motor.set_control(self.feed_duty_cycle.with_output(config.wrist_intake_speed))

    def feed_out(self, speed) -> None:
        """
        Runs feed motors out to score or send coral from wrist to intake.
        
        """
        self.feed_motor.set_control(self.feed_duty_cycle.with_output(speed))

    def feed_stop(self) -> None:
        """
        Stops feed motors.
        
        """

        self.feed_motor.set_control(self.feed_duty_cycle.with_output(0))

    def set_coral(self, is_there_a_coral_in_the_feed: bool):
        """
        Says coral is in the feed

        """
        self.coral_in_feed = is_there_a_coral_in_the_feed

    def algae_in(self) -> None:
        self.algae_motor.set_control(self.algae_duty_cycle.with_output(config.wrist_algae_speed))
    
    def algae_out(self) -> None:
        self.algae_motor.set_control(self.algae_duty_cycle.with_output(config.wrist_algae_extake_speed))

    def algae_stop(self) -> None:
        self.algae_motor.set_control(self.algae_duty_cycle.with_output(0))

    def hold_algae(self, voltage = config.wrist_algae_hold_volts) -> None:
        self.algae_motor.set_control(self.algae_volts.with_output(voltage))

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
        self.req = self.wrist_control.with_position(self.target_angle)
        
        self.wrist_motor.set_control(self.req)

    def get_wrist_angle(self) -> radians:
        """
        Gets the current wrist angle.

        """
        
        return (
            (
                self.wrist_motor.get_position()
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
        self.table.putNumber("wrist absolute angle", (math.degrees(self.encoder.get_absolute_position().value - config.wrist_encoder_zero)
            / constants.wrist_encoder_gear_ratio
            * 2
            * math.pi))
        self.table.putNumber("calculated kG", config.wrist_max_ff * math.cos(self.get_wrist_angle() - config.wrist_ff_offset))
        self.table.putNumber("wrist applied output", self.wrist_motor.get_applied_output())
        self.table.putNumber("wrist current", self.wrist_motor.get_motor_current())
        self.table.putBoolean("coral in feed", self.coral_in_feed)
        self.table.putNumber("wrist angle difference", (math.degrees(self.encoder.get_absolute_position().value - config.wrist_encoder_zero)
            / constants.wrist_encoder_gear_ratio
            * 2
            * math.pi)-self.get_wrist_angle())
        
        self.table.putBoolean("algae in wrist", self.algae_in_wrist)
        self.table.putNumber("algae motor current", self.algae_motor.get_motor_current())

    def periodic(self) -> None:
        if config.NT_WRIST:
            self.update_table()
