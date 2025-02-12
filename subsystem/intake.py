import config
import constants
from toolkit.motors.ctre_motors import TalonFX
from toolkit.subsystem import Subsystem
import ntcore

import math
from math import pi

from wpilib import Timer


class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.horizontal_motor: TalonFX = TalonFX(
            config.horizontal_id,
            config.foc_active,
            inverted=False,
            # config=config.HORIZONTAL_CONFIG,
        )
        self.vertical_motor: TalonFX = TalonFX(
            config.vertical_id,
            config.foc_active,
            inverted=False,
            # config=config.VERTICAL_CONFIG,
        )
        self.pivot_motor: TalonFX = TalonFX(
            config.intake_pivot_id,
            config.foc_active,
            inverted=False,
            config=config.INTAKE_PIVOT_CONFIG,
        )
        self.intake_running: bool = False

        self.pivot_angle = math.radians(90)
        self.intake_pivoting: bool = False
        self.intake_up: bool = True  # True is up, False is down
        self.target_intake_position: bool = True  # True is up, False is down

        self.encoder: CANcoder = CANcoder(config.intake_cancoder_id)


    def init(self):
        self.horizontal_motor.init()
        self.vertical_motor.init()
        self.pivot_motor.init()

    def roll_in(self) -> None:
        """
        spin the motors inwards to collect the coral
        """
        self.horizontal_motor.set_raw_output(
            config.intake_speed * constants.horizontal_gear_ratio
        )
        self.vertical_motor.set_raw_output(
            config.intake_speed * constants.vertical_gear_ratio
        )
        self.intake_running = True

    def stop(self) -> None:
        """
        stop the motors
        """
        self.vertical_motor.set_raw_output(0)
        self.horizontal_motor.set_raw_output(0)
        self.intake_running = False

    def roll_out(self) -> None:
        """
        eject coral in the intake
        """
        self.horizontal_motor.set_raw_output(
            -config.intake_speed * constants.horizontal_gear_ratio
        )
        self.vertical_motor.set_raw_output(
            -config.intake_speed * constants.vertical_gear_ratio
        )
        self.intake_running = True

    def get_vertical_motor_current(self) -> float:
        return self.vertical_motor.get_motor_current()

    def get_horizontal_motor_current(self) -> float:
        return self.horizontal_motor.get_motor_current()

    def get_pivot_motor_current(self) -> float:
        return self.pivot_motor.get_motor_current()

    def get_pivot_angle(self):
        "returns current angle of pivot"
        self.pivot_angle = (
            self.pivot_motor.get_sensor_position()
            / constants.intake_pivot_gear_ratio
            * pi
            * 2
        )
        return self.pivot_angle

    def is_pivot_up(self) -> bool:
        "returns a bool for if the pivot is in up position"
        self.pivot_angle = self.get_pivot_angle()
        self.intake_up = (
            self.pivot_angle >= config.intake_max_angle - config.intake_angle_threshold
        )

        return self.intake_up

    def is_pivot_down(self) -> bool:
        "returns a bool for if the pivot is in down position"
        self.pivot_angle = self.get_pivot_angle()
        self.intake_down = (
            self.pivot_angle < config.intake_min_angle + config.intake_angle_threshold
        )

        return self.intake_down

    def pivot_up(self) -> None:
        """
        pivot the intake up
        """
        self.pivot_motor.set_target_position(
            (config.intake_max_angle / 2 * math.pi) * constants.intake_pivot_gear_ratio
        )

    def pivot_down(self) -> None:
        """
        pivot the intake down
        """
        self.pivot_motor.set_target_position(
            (config.intake_min_angle / 2 * math.pi) * constants.intake_pivot_gear_ratio
        )

    def update_table(self) -> None:
        table = ntcore.NetworkTableInstance.getDefault().getTable("intake")

        table.putBoolean("intake running", self.intake_running)
        table.putNumber("horizontal current", self.get_horizontal_motor_current())
        table.putNumber("vertical current", self.get_vertical_motor_current())
        table.putNumber("pivot current", self.get_vertical_motor_current())
        table.putNumber("pivot angle", self.get_pivot_angle())
        table.putBoolean("pivot moving", self.intake_pivoting)

    def periodic(self) -> None:
        if config.NT_INTAKE:
            self.update_table()

