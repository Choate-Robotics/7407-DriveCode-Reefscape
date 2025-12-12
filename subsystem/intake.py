import config
import constants
from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals 
import math
from units.SI import radians
import commands2

class Intake(commands2.subsystem):
    
    pivot_motor_current: StatusSignal
    pivot_motor_pos: StatusSignal
    horizontal_motor_current: StatusSignal
    horizontal_motor_pos: StatusSignal

    def __init__(self):
        super().__init__()

        self.horizontal_motor = hardware.TalonFX(config.horizontal_id)
        self.pivot_motor = hardware.TalonFX(config.intake_pivot_id)
        self.horizontal_motor_out = controls.DutyCycleOut(0)
       
        self.horizontal_motor_configs = (
            configs.TalonFXConfiguration()
            .with_motor_output(
                configs.MotorOutputConfigs()
                .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
                .with_neutral_mode(signals.NeutralModeValue.BRAKE)
            ).with_feedback(

                
            )
            
            )

        self.pivot_motor_configs = (
            configs.TalonFXConfiguration()
            .with_motor_output(
                configs.MotorOutputConfigs()
                .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
                .with_neutral_mode(signals.NeutralModeValue.BRAKE)
            ).with_feedback(
                configs.FeedbackConfigs()
                .with_feedback_remote_sensor_id(signals.FeedbackSensorSourceValue.FUSED_CANCODER)
            ).with_motion_magic(
                configs.MotionMagicConfigs()
                .with_motion_magic_cruise_velocity(97)
                
            ).with_slot1(
                configs.Slot1Configs()
                .with_k_p(2)
                .with_k_i(0)
                .with_k_d(0)
                .with_k_s(-0.195)
                .with_k_v(0)
                .with_k_a(0)
                .with_gravity_type(signals.GravityTypeValue.ARM_COSINE)

            )
            
            )
        
        self.intake_running: bool = False

        self.pivot_angle = math.radians(0)
        self.intake_pivoting: bool = False
        self.target_angle: radians = 0
        self.pivot_zeroed: bool = False

        self.algae_in_intake = False

        self.encoder: CANcoder = CANcoder(config.intake_cancoder_id)


    def init(self):
        self.horizontal_motor.configurator.apply(self.horizontal_motor_configs)
        self.pivot_motor.init()
        self.pivot_motor.configurator.apply(self.pivot_motor_configs)
        self.zero_pivot()
        self._motion_magic_voltage = controls.MotionMagicVoltage(0)

        
    def roll_in(self) -> None:
        """
        spin the motors inwards to collect the coral
        """

        self.horizontal_motor.set_control(self.control.with_output(config.horizontal_intake_speed))
        """    self.horizontal_motor.set_raw_output(
            config.horizontal_intake_speed
        )
            """
        
        self.intake_running = True 
    
    def intake_algae(self) -> None:

        self.horizontal_motor.set_control(self.control.with_output(-config.horizontal_intake_speed))
        self.intake_running = True

    def stop(self) -> None:
        """
        stop the motors
        """
        self.horizontal_motor.set_control(self._duty_cycle_out.with_output(0)),
        f"raw output: {0}",
        
        self.intake_running = False

    def roll_out(self, speed: float = config.horizontal_intake_speed) -> None:
        """
        eject coral in the intake
        """

        ""
        self.horizontal_motor.set_control(self.control.with_output(config.speed))
        self.intake_running = True

    def extake_algae(self) -> None:
        self.horizontal_motor.set_control(self.control.with_output(config.extake_algae_speed))
        self.intake_running = True

    def get_horizontal_motor_current(self) -> float:
        self.horizontal_motor_current.refresh()
        return self.horizontal_motor_current.value

    def get_pivot_motor_current(self) -> float:
        self.pivot_motor_current.refresh()
        return self.pivot_motor_current.value

    
    def limit_angle(self, angle: radians) -> radians:
        """
        limits if the given angle in radians is within the range of the wrist
        if it is out of range, it returns the min or max angle in radians
        otherwise it returns the given angle

        takes in angle in radians
        """
        if angle <= config.intake_min_angle:
            return config.intake_min_angle
        elif angle >= config.intake_max_angle:
            return config.intake_max_angle
        return angle

    def zero_pivot(self) -> None:
        """
        zero the pivot encoder
        """

        self.pivot_angle = (
            (self.encoder.get_absolute_position().value - config.intake_encoder_zero) / constants.intake_encoder_gear_ratio * 2 * math.pi
        )
        pos = self.pivot_angle * constants.intake_pivot_gear_ratio / (2 * math.pi)
        self.pivot_motor.set_position(pos), f"sensor position: {pos}"

        self.pivot_zeroed = True

    def get_pivot_angle(self):
        "returns current angle of pivot"
        self.pivot_motor_pos.refresh()
        self.pivot_angle = (
            self.pivot_motor_pos.value / constants.intake_pivot_gear_ratio * math.pi * 2
        )
        return self.pivot_angle
    
    def is_at_angle(self, angle: radians) -> bool:
        return abs(self.get_pivot_angle() - angle) < config.intake_angle_threshold

    def set_pivot_angle(self, angle: radians) -> None:
        """
        setting the angle of the pivot
        """

        ff = config.intake_max_ff * math.cos(config.intake_ff_offset - angle)
        rotations = angle / (2 * math.pi) * constants.intake_pivot_gear_ratio

        self.target_angle = angle
        self.moto
        self.error_check(
            self.pivot_motor.set_control(self._motion_magic_voltage.with_position(rotations),
            f"target position: {rotations}, arbFF: {ff}",)
        )
        

    def stop_pivot(self) -> None:
        self.pivot_motor.set_control(self.control.with_output(0))

    def periodic(self) -> None:
        #if config.NT_INTAKE:
            #self.update_table()
        pass
        