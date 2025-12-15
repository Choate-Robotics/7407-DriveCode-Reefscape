import config
import constants
from toolkit.motors.ctre_motors import TalonFX
from toolkit.subsystem import Subsystem
from phoenix6.hardware import CANcoder
import ntcore

import math
from units.SI import radians



class Intake(Subsystem):

    def __init__(self):
        super().__init__()
        self.horizontal_motor: TalonFX = TalonFX(
            config.horizontal_id,
            config.foc_active,
            inverted=False,
            config=config.INTAKE_CONFIG,
        )
        self.pivot_motor: TalonFX = TalonFX(
            config.intake_pivot_id,
            config.foc_active,
            inverted=False,
            config=config.INTAKE_PIVOT_CONFIG,
        )
        self.intake_running: bool = False

        self.pivot_angle = math.radians(0)
        self.intake_pivoting: bool = False
        self.target_angle: radians = 0
        self.pivot_zeroed: bool = False

        self.algae_in_intake = False

        self.encoder: CANcoder = CANcoder(config.intake_cancoder_id)


    def init(self):
        self.horizontal_motor.init()
        self.pivot_motor.init()

        self.zero_pivot()

        self.table = ntcore.NetworkTableInstance.getDefault().getTable("intake")
        self.intake_running_pub = self.table.getBooleanTopic("intake running").publish()
        self.horizontal_current_pub = self.table.getNumber("horizontal current").publish()
        self.pivot_current_pub = self.table.getNumber("pivot current").publish()
        self.pivot_applied_output_pub = self.table.getNumber("pivot applied output").publish()
        self.pivot_angle_pub = self.table.getNumber("pivot angle").publish()
        self.pivot_absolute_angle_pub = self.table.getNumber("pivot absolute angle").publish()
        self.absolute_position_pub = self.table.getNumber("absolute position").publish()
        self.pivot_moving_pub = self.table.getBoolean("pivot moving").publish()
        self.pivot_zeroed_pub = self.table.getBoolean("pivot zeroed").publish()
        self.pivot_target_angle_pub = self.table.getNumber("pivot target angle").publish()
        self.pivot_velocity_pub= self.table.getNumber("pivot velocity").publish()
        self.pivot_acceleration_pub = self.table.getNumber("pivot acceleration").publish()


    def roll_in(self) -> None:
        """
        spin the motors inwards to collect the coral
        """
        self.horizontal_motor.set_raw_output(
            config.horizontal_intake_speed
        )
        self.intake_running = True
    
    def intake_algae(self) -> None:

        self.horizontal_motor.set_raw_output(
            -config.intake_algae_speed
        )
        self.intake_running = True

    def stop(self) -> None:
        """
        stop the motors
        """
        self.horizontal_motor.set_raw_output(0)
        self.intake_running = False

    def roll_out(self, speed: float = config.horizontal_intake_speed) -> None:
        """
        eject coral in the intake
        """
        self.horizontal_motor.set_raw_output(
            -speed
        )
        self.intake_running = True

    def extake_algae(self) -> None:

        self.horizontal_motor.set_raw_output(
            config.extake_algae_speed
        )

        self.intake_running = True

    def get_horizontal_motor_current(self) -> float:
        return self.horizontal_motor.get_motor_current()

    def get_pivot_motor_current(self) -> float:
        return self.pivot_motor.get_motor_current()
    
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

        self.pivot_motor.set_sensor_position(
            self.pivot_angle * constants.intake_pivot_gear_ratio / (2 * math.pi)
        )

        self.pivot_zeroed = True

    def get_pivot_angle(self):
        "returns current angle of pivot"
        self.pivot_angle = (
            self.pivot_motor.get_sensor_position()
            / constants.intake_pivot_gear_ratio
            * math.pi
            * 2
        )
        return self.pivot_angle
    
    def is_at_angle(self, angle: radians) -> bool:
        return abs(self.get_pivot_angle() - angle) < config.intake_angle_threshold

    def set_pivot_angle(self, angle: radians) -> None:
        """
        setting the angle of the pivot
        """

        ff = config.intake_max_ff * math.cos(config.intake_ff_offset - angle)

        self.target_angle = angle
        self.pivot_motor.set_target_position(
            (angle / (2 * math.pi)) * constants.intake_pivot_gear_ratio,
            ff
        )

    def stop_pivot(self) -> None:
        self.pivot_motor.set_raw_output(0)

    def update_table(self) -> None:
        table = ntcore.NetworkTableInstance.getDefault().getTable("intake")

        self.intake_running_pub.set(self.intake_running)
        self.horizontal_current_pub.set(self.get_horizontal_motor_current())
        self.pivot_current_pub.set(self.pivot_motor.get_motor_current())
        self.pivot_applied_output_pub.set(self.pivot_motor.get_applied_output())
        self.pivot_angle_pub.set(math.degrees(self.get_pivot_angle()))
        self.pivot_absolute_angle_pub.set(math.degrees((self.encoder.get_absolute_position().value - config.intake_encoder_zero) / constants.intake_encoder_gear_ratio * 2 * math.pi))
        self.absolute_position_pub.set(self.encoder.get_absolute_position().value)
        self.pivot_moving_pub.set(self.intake_pivoting)
        self.pivot_zeroed_pub.set(self.pivot_zeroed)
        self.pivot_target_angle_pub.set(math.degrees(self.target_angle))
        self.pivot_velocity_pub.set(self.pivot_motor.get_sensor_velocity())
        self.pivot_acceleration_pub(self.pivot_motor.get_sensor_acceleration())

    def periodic(self) -> None:
        if config.NT_INTAKE:
            self.update_table()

